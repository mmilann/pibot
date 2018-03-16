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
        .def("SetMotorDrive", &PiBot::SetMotorDrive)
        .def("SetPWM", &PiBot::SetPWM)
		.add_property("adc", &PiBot::adc)
    ;
    class_<ADConverter>("ADConverter")
        .def("Convert", &ADConverter::Convert)
    ;
	enum_<DriverOutput>("DriverOutput")
		.value("M1", M1)
		.value("M2", M2)
		.value("M3", M3)
		.export_values()
		.value("M4", M4)
    ;
}
