/*****************************************************************
File: python_pibot.cpp
Version: 1.0

Author: Milan Neskovic 2016-2018, milan@pi-supply.com

Description:
	Declares Python extension Module based on C++ interface to PiBot
	Robot.

Copyright:

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; version 3 of the
	License.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
******************************************************************/

#include <boost/python.hpp>
#include "../driver_cpp/pibot.h"

using namespace boost::python;
using namespace pibot;

BOOST_PYTHON_MODULE(pibot)
{
    class_<PiBot>("PiBot")
		.def("Enable", &PiBot::Enable)
		.def("InitMotorDriver", &PiBot::InitMotorDriver)
        .def("SetMotorDrive", &PiBot::SetMotorDrive)
        .def("SetPWM", &PiBot::SetPWM)
		.def("SetServoControl", &PiBot::SetServoControl)
    ;
    class_<ADConverter>("ADConverter")
        .def("ConvertToVolts", &ADConverter::ConvertToVolts)
    ;
	enum_<DriverOutput>("DriverOutput")
		.value("M1", M1)
		.value("M2", M2)
		.value("M3", M3)
		.value("M4", M4)
		.export_values()
    ;
	enum_<DriverId>("DriverId")
		.value("DRIVER_M_1_2", DRIVER_M_1_2)
		.value("DRIVER_M_3_4", DRIVER_M_3_4)
		.export_values()
    ;
	enum_<DeacayMode>("DeacayMode")
		.value("SLOW", SLOW)
		.value("FAST", FAST)
		.export_values()
    ;
}
