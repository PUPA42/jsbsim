
#ifndef FWS_INTERFACE_H
#define FWS_INTERFACE_H
#include <thread>
namespace JSBSim {
	extern "C" _declspec(dllexport) std::thread Start(int argc, char* argv[]);
	extern "C" _declspec(dllexport) int Stop(std::thread thread);
}

#endif