// AeroPropTransport.cpp -- Implements a Aeromatic Prop Transport Aircraft type.
//
// Based on Aeromatic2 PHP code by David P. Culp
// Started June 2003
//
// C++-ified and modulized by Erik Hofman, started October 2015.
//
// Copyright (C) 2003, David P. Culp <davidculp2@comcast.net>
// Copyright (C) 2015 Erik Hofman <erik@ehofman.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#include <Systems/Systems.h>
#include <Systems/Propulsion.h>
#include <Systems/Controls.h>
#include "Aircraft.h"


namespace Aeromatic
{

PropTransport::PropTransport(Aeromatic *p) : Aircraft(p)
{
    _description = "Propeller Transport";

    _subclasses.push_back("Propeller Airliner");
    _subclasses.push_back("Propeller Transport");

    _systems.push_back(new Propulsion(_aircraft));
    _systems.push_back(new Controls(_aircraft));
    _systems.push_back(new LandingGear(_aircraft));
    _systems.push_back(new Flaps(_aircraft));
}

void PropTransport::set_lift()
{
    // estimate slope of lift curve based on airplane type
    // units: per radian
    _aircraft->_CLalpha = _CLalpha_t[_subtype][_engines];

    // estimate CL at zero alpha
    _aircraft->_CL0 = _CL0_t[_subtype][_engines];

    // estimate stall CL, based on airplane type
    _aircraft->_CLmax = _CLmax_t[_subtype][_engines];

    // estimate lift due to elevator deflection
    _aircraft->_CLde = 0.2f;
}

void PropTransport::set_drag()
{
    // estimate drag at zero lift, based on airplane type
    // NOT including landing gear
    _aircraft->_CD0 = _CD0_t[_subtype][_engines];

    // estimate induced drag coefficient K
    _aircraft->_K = _K_t[_subtype][_engines];

    _aircraft->_CDde = 0.04f;      // elevator deflection
    _aircraft->_CDbeta = 0.2f;     // sideslip

    // estimate critical mach, based on airplane type
    _aircraft->_CDMcrit = _Mcrit_t[_subtype][_engines];
}

void PropTransport::set_side()
{
    _aircraft->_CYbeta = -1.0f;
}

void PropTransport::set_roll()
{
    // estimate roll coefficients
    _aircraft->_Clbeta = -0.1f;    // sideslip
    _aircraft->_Clp = -0.4f;       // roll rate
    _aircraft->_Clr = 0.15f;       // yaw rate
    _aircraft->_Cldr = 0.01f;      // rudder deflection

    // aileron
    _aircraft->_Clda = _Clda_t[_subtype][_engines];
}

void PropTransport::set_pitch()
{
    // per radian alpha
    _aircraft->_Cmalpha = _Cmalpha_t[_subtype][_engines];

    // elevator deflection
    _aircraft->_Cmde = _Cmde_t[_subtype][_engines];

    // pitch rate
    _aircraft->_Cmq = _Cmq_t[_subtype][_engines];

    // alpha-dot
    _aircraft->_Cmadot = _Cmadot_t[_subtype][_engines];
}

void PropTransport::set_yaw()
{
    _aircraft->_Cnbeta = 0.12f;    // sideslip
    _aircraft->_Cnr = -0.15f;      // yaw rate
    _aircraft->_Cndr = -0.10f;        // rudder deflection

    // adverse yaw
    _aircraft->_Cnda = _Cnda_t[_subtype][_engines];
}

// ----------------------------------------------------------------------------

float const PropTransport::_wing_loading_t[1][5] =
{
    {  57.0f,  57.0f,  57.0f,  57.0f,  57.0f }
};

float const PropTransport::_htail_area_t[1][5] =
{
    { 0.16f, 0.16f, 0.16f, 0.16f, 0.16f }
};

float const PropTransport::_htail_arm_t[1][5] =
{
    { 0.50f, 0.50f, 0.50f, 0.50f }
};

float const PropTransport::_vtail_area_t[1][5] =
{
    { 0.18f, 0.18f, 0.18f, 0.18f, 0.18f }
};

float const PropTransport::_vtail_arm_t[1][5] =
{ 
    { 0.50f, 0.50f, 0.50f, 0.50f, 0.50f }
};

float const PropTransport::_empty_weight_t[1][5] =
{
    { 0.60, 0.60, 0.60, 0.60, 0.60 }
};

float const PropTransport::_roskam_t[1][5][3] =
{
    {				
        { 0.32f, 0.35f, 0.47f },
        { 0.32f, 0.35f, 0.47f },
        { 0.32f, 0.35f, 0.47f },
        { 0.32f, 0.35f, 0.47f },
        { 0.32f, 0.35f, 0.47f }
    }
};

float const PropTransport::_eyept_loc_t[1][5][3] =
{ 
    {				
        { 0.08f, -24.00f, 65.00f },
        { 0.08f, -24.00f, 65.00f },
        { 0.08f, -24.00f, 65.00f },
        { 0.08f, -24.00f, 65.00f },
        { 0.08f, -24.00f, 65.00f }
     }
};

float const PropTransport::_gear_loc_t[1][5] =
{
    {  0.11f, 0.11f, 0.11f, 0.11f, 0.11f }
};

float const PropTransport::_fuel_weight_t[1][5] =
{
    { 0.254f, 0.254f, 0.254f, 0.254f, 0.254f }
};

float const PropTransport::_CLalpha_t[1][5] =
{
    { 4.9f, 4.9f, 4.9f, 4.9f, 4.9f }
};

float const PropTransport::_CL0_t[1][5] =
{
    { 0.24f, 0.24f, 0.24f, 0.24f, 0.24f }
};

float const PropTransport::_CLmax_t[1][5] =
{
    { 1.40f, 1.40f, 1.40f, 1.40f, 1.40f }
};

float const PropTransport::_CD0_t[1][5] =
{
    { 0.025f, 0.025f, 0.025f, 0.025f, 0.025f }
};

float const PropTransport::_K_t[1][5] =
{
    { 0.039f, 0.039f, 0.039f, 0.039f, 0.039f }
};

float const PropTransport::_Mcrit_t[1][5] =
{
    { 0.70f, 0.70f, 0.70f, 0.70f, 0.70 }
};

float const PropTransport::_Cmalpha_t[1][5] =
{
    { -0.4f, -0.4f, -0.4f, -0.4f, -0.4f }
};

float const PropTransport::_Cmde_t[1][5] =
{
    { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f }
};

float const PropTransport::_Cmq_t[1][5] =
{
    { -22.0f, -22.0f, -22.0f, -22.0f, -22.0f }
};

float const PropTransport::_Cmadot_t[1][5] =
{
    {  -8.0f,  -8.0f,  -8.0f,  -8.0f,  -8.0f }
};

float const PropTransport::_Clda_t[1][5] =
{
    { 0.15f, 0.15f, 0.15f, 0.15f, 0.15f }
};

float const PropTransport::_Cnda_t[1][5] =
{
    { -0.008f, -0.008f, -0.008f, -0.008f, -0.008f }
};

} /* namespace Aeromatic */
