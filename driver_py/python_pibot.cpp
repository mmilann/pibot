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
#include "pibot.h"

using namespace boost::python;

BOOST_PYTHON_MODULE(pibot)
{
    class_<PiBot>("PiBot")
        .def("SetSpeed", &PiBot::SetSpeed)
        .def("SetPWM", &PiBot::SetPWM)
    ;
}
