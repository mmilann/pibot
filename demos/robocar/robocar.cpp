/*****************************************************************
File: bottest.cpp
Version: 1.0

Author: Milan Neskovic 2016-2018, milan@pi-supply.com

Description:
	C++ example code for PiBot Boards based Robot.

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

#include "pibot.h"
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <sys/ioctl.h>
#include <atomic>
#include <signal.h>
#include <cstring>
#include <termios.h>

using namespace std;
using namespace pibot;

std::atomic<bool> quit(false);    // signal flag

struct termios orig_ttystate;

void kb_nonblock(bool enable)
{
    struct termios ttystate;
    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);
 
    if (enable) {
		// take copiy for restore on exit
		tcgetattr(0, &orig_ttystate);
        //turn off canonical mode
        ttystate.c_lflag &= (~ICANON & ~ECHO);
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
		//cfmakeraw(&ttystate);
		tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
		system("setterm -cursor off");
    } else {
		system("setterm -cursor on");
        //turn on canonical mode
		tcsetattr(0, TCSANOW, &orig_ttystate);
    }
}

int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void quit_signal(int)
{
    quit.store(true);
}

void FaultEvent(void) {
	//printf("Fault \n");
}

int main(void) {
	
	PiBot pibot;
	Encoder enc1(20, 5), enc2(20, 16);
	ADConverter adc;
	MagAcc magacc;
	Barometer bar;
	char c[3];
	int lightLevel = 1;
	float targetSpeed = 3, speedWheelLeft, speedWheelRight;
	float turnSpeed = 0;
	int16_t driveWheelLeft = 60, driveWheelRight = 60; 
	int prevCnt[2];
    struct sigaction sa;
	
    memset( &sa, 0, sizeof(sa) ); 
    sa.sa_handler = quit_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL); 
	
	kb_nonblock(true);
	
	pinMode(24,  INPUT);
	pullUpDnControl(24, PUD_UP);
	wiringPiISR(24, INT_EDGE_FALLING, FaultEvent);
	
	usleep(10000);
	
	pibot.InitMotorDriver(DRIVER_M_1_2);
	pibot.InitSonar(1);
	pibot.InitSonar(2);
	pibot.InitSonar(3);
	
	pinMode(17,  OUTPUT); // zero encoder current drive on/off
	digitalWrite(17, HIGH);
	
	//pibot.SetCurrentLimit(10);
	
	//pibot.SetPWM(1, 2048); 

	pibot.SetCurrentDrive(16, 10);
	
	PiBot::Enable();

	while (1) {
		std::system("clear");
		//k=kbhit();
        while (kbhit()!=0)
        {
			c[2] = c[1];
			c[1] = c[0];
            c[0]=fgetc(stdin);
			switch (c[0]) {
				case 'q': quit.store(true); break;
				case '\033': break;
				case '[': break;
				case 'A':
					if (c[2] == '\033' && c[1] == '[') {
						// arrow up, speed up
						if ((speedWheelLeft-targetSpeed) > -1 && (speedWheelRight-targetSpeed) > -1) targetSpeed += 1;
						//if ((speedWheelRight-targetSpeed) > -0.1) targetSpeed += 0.1;
					}
					break;
				case 'B':
					if (c[2] == '\033' && c[1] == '[') {
						// arrow down, slow down
						if ((speedWheelLeft-targetSpeed) < 1 && (speedWheelRight-targetSpeed) < 1) targetSpeed -= 1;
					}
					break;
				case 'C':
					if (c[2] == '\033' && c[1] == '[') {
						// arrow right
						turnSpeed = 0.5;
					}
					break;
				case 'D':
					if (c[2] == '\033' && c[1] == '[') {
						// arrow left
						turnSpeed = -0.5;
					}
					break;
				case 'l': 
					if (lightLevel < 4096) lightLevel *= 2;
					break;
				case 'k':
					if (lightLevel>=2) lightLevel /= 2; 
					break;
				default:
					break;
			}	
        }
		 
		//pibot.SetPWM(16, lightLevel-1);

		//cout << "Light Level: "<<lightLevel<<endl;
		cout << "magX: " << magacc.GetMagX()<< " magY: " << magacc.GetMagY()<< " magZ: " << magacc.GetMagZ() << endl;
		cout << "accX: " << magacc.GetAccX()<< " accY: " << magacc.GetAccY()<< " accZ: " << magacc.GetAccZ() << endl;
		cout << "pressure: " << bar.GetPressure() << endl;
		cout << "range1 [cm]: " << pibot.SonarDistance(1) << "  range2 [cm]: " << pibot.SonarDistance(2) << "  range3 [cm]: " << pibot.SonarDistance(3) << endl;
		pibot.SonarTrigger();
		speedWheelLeft =  enc1.AngularSpeed()*(driveWheelLeft<0?-1:1); // float(enc2.counter - prevCnt[1]) / 2; //
		speedWheelRight = enc2.AngularSpeed()*(driveWheelRight<0?-1:1); // float(enc1.counter - prevCnt[0]) / 2; //
		cout << "drive: "<< (int)driveWheelLeft <<", "<< (int)driveWheelRight<< " target: "<< targetSpeed << ", speed left "<< speedWheelLeft<< ", speed right "<< speedWheelRight << endl;
		if (speedWheelLeft > (targetSpeed+turnSpeed) ) {
			if (driveWheelLeft >= -250) driveWheelLeft -= 5;
		} else {
			if (driveWheelLeft <= 250) driveWheelLeft += 5;
		}
		if (speedWheelRight > (targetSpeed-turnSpeed) ) {
			if (driveWheelRight >= -250) driveWheelRight -= 5;
		} else {
			if (driveWheelRight <= 250) driveWheelRight += 5;
		}
		turnSpeed = 0;
		
		pibot.SetMotorDrive(M1, driveWheelLeft);
		pibot.SetMotorDrive(M2, driveWheelRight);
		
		prevCnt[0] = enc1.counter;
		prevCnt[1] = enc2.counter;
		
		if (pibot.IsPowerLow()) {
			cout << "Low power: " << endl;
			break;
		}
		
		adc.Convert();
		usleep(100000);
		PiBot::Enable(); 
		
		if( quit.load() ) break;    // exit normally after SIGINT
	}
	kb_nonblock(false);
	return 0;
}