#include <thread>
#include "FWSInterface.h"
#include "JSBSim.h"
#include "initialization/FGTrim.h"
#include "FGFDMExec.h"
#include "input_output/FGXMLFileRead.h"

#if !defined(__GNUC__) && !defined(sgi) && !defined(_MSC_VER)
#  include <time>
#else
#  include <time.h>
#endif

#if defined(_MSC_VER)
#  include <float.h>
#elif defined(__GNUC__) && !defined(sgi)
#  include <fenv.h>
#endif

#if defined(__BORLANDC__) || defined(_MSC_VER) || defined(__MINGW32__)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#  include <mmsystem.h>
#  include <regstr.h>
#  include <sys/types.h>
#  include <sys/timeb.h>
#else
#  include <sys/time.h>
#endif

#include <iostream>
#include <cstdlib>


namespace JSBSim {

	std::thread Start(int argc, char * argv[])
	{
		std::thread thread = std::thread(JSBSim::real_main(argc, argv));
		return thread;
	}

	int Stop(std::thread thread)
	{
		thread.detach();
		return 0;
	}
	int real_main(int argc, char* argv[])
	{
		// *** INITIALIZATIONS *** //

		ScriptName = "";
		AircraftName = "";
		ResetName = "";
		LogOutputName.clear();
		LogDirectiveName.clear();
		bool result = false, success;
		bool was_paused = false;

		double frame_duration;

		double new_five_second_value = 0.0;
		double actual_elapsed_time = 0;
		double initial_seconds = 0;
		double current_seconds = 0.0;
		double paused_seconds = 0.0;
		double sim_lag_time = 0;
		double cycle_duration = 0.0;
		double override_sim_rate_value = 0.0;
		long sleep_nseconds = 0;

		realtime = false;
		play_nice = false;
		suspend = false;
		catalog = false;
		nohighlight = false;

		// *** PARSE OPTIONS PASSED INTO THIS SPECIFIC APPLICATION: JSBSim *** //
		success = options(argc, argv);
		if (!success) {
			PrintHelp();
			exit(-1);
		}

		// *** SET UP JSBSIM *** //
		FDMExec = new JSBSim::FGFDMExec();
		FDMExec->SetRootDir(RootDir);
		FDMExec->SetAircraftPath(SGPath("aircraft"));
		FDMExec->SetEnginePath(SGPath("engine"));
		FDMExec->SetSystemsPath(SGPath("systems"));
		FDMExec->GetPropertyManager()->Tie("simulation/frame_start_time", &actual_elapsed_time);
		FDMExec->GetPropertyManager()->Tie("simulation/cycle_duration", &cycle_duration);

		if (nohighlight) FDMExec->disableHighLighting();

		if (simulation_rate < 1.0)
			FDMExec->Setdt(simulation_rate);
		else
			FDMExec->Setdt(1.0 / simulation_rate);

		if (override_sim_rate) override_sim_rate_value = FDMExec->GetDeltaT();

		// SET PROPERTY VALUES THAT ARE GIVEN ON THE COMMAND LINE and which are for the simulation only.

		for (unsigned int i = 0; i < CommandLineProperties.size(); i++) {

			if (CommandLineProperties[i].find("simulation") != std::string::npos) {
				if (FDMExec->GetPropertyManager()->GetNode(CommandLineProperties[i])) {
					FDMExec->SetPropertyValue(CommandLineProperties[i], CommandLinePropertyValues[i]);
				}
			}
		}

		// *** OPTION A: LOAD A SCRIPT, WHICH LOADS EVERYTHING ELSE *** //
		if (!ScriptName.isNull()) {

			result = FDMExec->LoadScript(ScriptName, override_sim_rate_value, ResetName);

			if (!result) {
				cerr << "Script file " << ScriptName << " was not successfully loaded" << endl;
				delete FDMExec;
				exit(-1);
			}

			// *** OPTION B: LOAD AN AIRCRAFT AND A SET OF INITIAL CONDITIONS *** //
		}
		else if (!AircraftName.empty() || !ResetName.isNull()) {

			if (catalog) FDMExec->SetDebugLevel(0);

			if (!FDMExec->LoadModel(SGPath("aircraft"),
				SGPath("engine"),
				SGPath("systems"),
				AircraftName)) {
				cerr << "  JSBSim could not be started" << endl << endl;
				delete FDMExec;
				exit(-1);
			}

			if (catalog) {
				FDMExec->PrintPropertyCatalog();
				delete FDMExec;
				return 0;
			}

			JSBSim::FGInitialCondition *IC = FDMExec->GetIC();
			if (!IC->Load(ResetName)) {
				delete FDMExec;
				cerr << "Initialization unsuccessful" << endl;
				exit(-1);
			}

		}
		else {
			cout << "  No Aircraft, Script, or Reset information given" << endl << endl;
			delete FDMExec;
			exit(-1);
		}

		// Load output directives file[s], if given
		for (unsigned int i = 0; i < LogDirectiveName.size(); i++) {
			if (!LogDirectiveName[i].isNull()) {
				if (!FDMExec->SetOutputDirectives(LogDirectiveName[i])) {
					cout << "Output directives not properly set in file " << LogDirectiveName[i] << endl;
					delete FDMExec;
					exit(-1);
				}
			}
		}

		// OVERRIDE OUTPUT FILE NAME. THIS IS USEFUL FOR CASES WHERE MULTIPLE
		// RUNS ARE BEING MADE (SUCH AS IN A MONTE CARLO STUDY) AND THE OUTPUT FILE
		// NAME MUST BE SET EACH TIME TO AVOID THE PREVIOUS RUN DATA FROM BEING OVER-
		// WRITTEN.
		for (unsigned int i = 0; i < LogOutputName.size(); i++) {
			string old_filename = FDMExec->GetOutputFileName(i);
			if (!FDMExec->SetOutputFileName(i, LogOutputName[i])) {
				cout << "Output filename could not be set" << endl;
			}
			else {
				cout << "Output filename change from " << old_filename << " from aircraft"
					" configuration file to " << LogOutputName[i] << " specified on"
					" command line" << endl;
			}
		}

		// SET PROPERTY VALUES THAT ARE GIVEN ON THE COMMAND LINE

		for (unsigned int i = 0; i < CommandLineProperties.size(); i++) {

			if (!FDMExec->GetPropertyManager()->GetNode(CommandLineProperties[i])) {
				cerr << endl << "  No property by the name " << CommandLineProperties[i] << endl;
				delete FDMExec;
				exit(-1);
			}
			else {
				FDMExec->SetPropertyValue(CommandLineProperties[i], CommandLinePropertyValues[i]);
			}
		}

		FDMExec->RunIC();

		// PRINT SIMULATION CONFIGURATION
		FDMExec->PrintSimulationConfiguration();

		// Dump the simulation state (position, orientation, etc.)
		FDMExec->GetPropagate()->DumpState();

		// Perform trim if requested via the initialization file
		JSBSim::TrimMode icTrimRequested = (JSBSim::TrimMode)FDMExec->GetIC()->TrimRequested();
		if (icTrimRequested != JSBSim::TrimMode::tNone) {
			trimmer = new JSBSim::FGTrim(FDMExec, icTrimRequested);
			try {
				trimmer->DoTrim();

				if (FDMExec->GetDebugLevel() > 0)
					trimmer->Report();

				delete trimmer;
			}
			catch (string& msg) {
				cerr << endl << msg << endl << endl;
				exit(1);
			}
		}

		cout << endl << JSBSim::FGFDMExec::fggreen << JSBSim::FGFDMExec::highint
			<< "---- JSBSim Execution beginning ... --------------------------------------------"
			<< JSBSim::FGFDMExec::reset << endl << endl;

		result = FDMExec->Run();  // MAKE AN INITIAL RUN

		if (suspend) FDMExec->Hold();

		// Print actual time at start
		char s[100];
		time_t tod;
		time(&tod);
		strftime(s, 99, "%A %B %d %Y %X", localtime(&tod));
		cout << "Start: " << s << " (HH:MM:SS)" << endl;

		frame_duration = FDMExec->GetDeltaT();
		if (realtime) sleep_nseconds = (long)(frame_duration*1e9);
		else          sleep_nseconds = (sleep_period)*1e9;           // 0.01 seconds

		tzset();
		current_seconds = initial_seconds = getcurrentseconds();

		// *** CYCLIC EXECUTION LOOP, AND MESSAGE READING *** //
		while (result && FDMExec->GetSimTime() <= end_time) {

			FDMExec->ProcessMessage(); // Process messages, if any.

									   // Check if increment then hold is on and take appropriate actions if it is
									   // Iterate is not supported in realtime - only in batch and playnice modes
			FDMExec->CheckIncrementalHold();

			// if running realtime, throttle the execution, else just run flat-out fast
			// unless "playing nice", in which case sleep for a while (0.01 seconds) each frame.
			// If suspended, then don't increment cumulative realtime "stopwatch".

			if (!FDMExec->Holding()) {
				if (!realtime) {         // ------------ RUNNING IN BATCH MODE

					result = FDMExec->Run();

					if (play_nice) sim_nsleep(sleep_nseconds);

				}
				else {                    // ------------ RUNNING IN REALTIME MODE

										  // "was_paused" will be true if entering this "run" loop from a paused state.
					if (was_paused) {
						initial_seconds += paused_seconds;
						was_paused = false;
					}
					current_seconds = getcurrentseconds();                      // Seconds since 1 Jan 1970
					actual_elapsed_time = current_seconds - initial_seconds;    // Real world elapsed seconds since start
					sim_lag_time = actual_elapsed_time - FDMExec->GetSimTime(); // How far behind sim-time is from actual
																				// elapsed time.
					for (int i = 0; i < (int)(sim_lag_time / frame_duration); i++) {  // catch up sim time to actual elapsed time.
						result = FDMExec->Run();
						cycle_duration = getcurrentseconds() - current_seconds;   // Calculate cycle duration
						current_seconds = getcurrentseconds();                    // Get new current_seconds
						if (FDMExec->Holding()) break;
					}

					if (play_nice) sim_nsleep(sleep_nseconds);

					if (FDMExec->GetSimTime() >= new_five_second_value) { // Print out elapsed time every five seconds.
						cout << "Simulation elapsed time: " << FDMExec->GetSimTime() << endl;
						new_five_second_value += 5.0;
					}
				}
			}
			else { // Suspended
				was_paused = true;
				paused_seconds = getcurrentseconds() - current_seconds;
				sim_nsleep(sleep_nseconds);
				result = FDMExec->Run();
			}

		}

		// PRINT ENDING CLOCK TIME
		time(&tod);
		strftime(s, 99, "%A %B %d %Y %X", localtime(&tod));
		cout << "End: " << s << " (HH:MM:SS)" << endl;

		// CLEAN UP
		delete FDMExec;

		return 0;
	}
}