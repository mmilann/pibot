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
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cerrno>
#include <unistd.h>
#include <wiringPi.h>
#include <sys/time.h>
#include <wiringPiI2C.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <atomic>
#include <signal.h>
#include <cstring>
#include <chrono>
#include <termios.h>
#include <sys/select.h>

#define STEPPER_PERIOD	20000

using namespace std;
//using namespace bot;

std::atomic<bool> quit(false);    // signal flag

struct termios orig_termios;

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}
#define NB_ENABLE	1
#define NB_DISABLE	0
void nonblock(int state)
{
    struct termios ttystate;
 
    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);
 
    if (state==NB_ENABLE)
    {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state==NB_DISABLE)
    {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
 
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

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

void got_signal(int)
{
    quit.store(true);
}

void FaultEvent(void) {
	//printf("Fault \n");
}

void LowPower(void) {
	printf("Low power \n");
	PiBot::PowerControl(1);
}

int main(void) {
	PiBot pibot;
	MagAcc magacc;
	Barometer bar;
	ADConverter adc;
	//float magX = 0, magY = 0, magZ = 0;
	//float temp = 0;
	struct timeval cur_time1, cur_time2, tdiff;
	char c[3];
	int k = 0;
	int lightLevel = 1;
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL); 
	
	nonblock(NB_ENABLE);//set_conio_terminal_mode();
	
	printf("\n");
	
	cout << "high_resolution_clock" << endl;
	cout << chrono::high_resolution_clock::period::num << endl;
	cout << chrono::high_resolution_clock::period::den << endl;
	cout << "steady = " << boolalpha << chrono::high_resolution_clock::is_steady << endl << endl;
	
	cout << "steady_clock" << endl;
	cout << chrono::steady_clock::period::num << endl;
	cout << chrono::steady_clock::period::den << endl;
	cout << "steady = " << boolalpha << chrono::steady_clock::is_steady << endl << endl;

	
	pinMode(24,  INPUT);
	pullUpDnControl(24, PUD_UP);
	auto start = chrono::high_resolution_clock::now();
	wiringPiISR(24, INT_EDGE_FALLING, FaultEvent);
	auto end = chrono::high_resolution_clock::now();
	auto diff = end - start;
	cout << chrono::duration <double, nano> (diff).count() << " ns" << endl;
	cout << chrono::duration <double, milli> (diff).count() << " ms" << endl;
	 
	wiringPiISR(22, INT_EDGE_FALLING, LowPower);
	usleep(10000);
	//PiBot::PowerControl(1);
	
	pinMode(17,  OUTPUT); // zero encoder current drive on/off
	digitalWrite(17, HIGH);
	
	pibot.SetSpeed(1, 0, 200);
	pibot.SetSpeed(2, 1, 200);
	pibot.SetSpeed(3, 0, 255);
	pibot.SetSpeed(4, 0, 255);
	
	//pibot.SetCurrentLimit(10);

	pibot.SetPWM(15, 600); // OFF
	pibot.SetPWM(16, 600); // OFF
	
	//pibot.SetPWM(1, 0); // servo
	//pibot.SetPWM(2, 400); // servo
	
	pibot.SetPWM(1, 2048); 
	pibot.SetPWM(2, 2048); 
	pibot.SetPWM(3, 2048); 
	pibot.SetPWM(4, 2048); 
	pibot.SetPWM(5, 2048); 
	pibot.SetPWM(6, 2048); 
	pibot.SetPWM(7, 2048); 
	pibot.SetPWM(8, 2048); 
	pibot.SetPWM(9, 2048); 
	pibot.SetPWM(10, 2048); 
	pibot.SetPWM(11, 2048); 
	pibot.SetPWM(12, 2048); 
	pibot.SetPWM(13, 2048); 
	pibot.SetPWM(14, 2048); 
	//pibot.SetPWM(15, 2048); 
	//pibot.SetPWM(16, 400); 
	
	gettimeofday(&cur_time1, 0);
	// stepper 
	//cout<<pibot.DriveSteps(STEPPER_COIL_M3, STEPPER_COIL_M4, -2, 1000)<<endl;
	//cout<<pibot.DriveSteps(STEPPER_COIL_M3, STEPPER_COIL_M4, 200, 950)<<endl;
	//for (int i = 0; i < 200; i++) pibot.stepper[2][3]->DriveSteps(-1, 1000);
	gettimeofday(&cur_time2, 0);
	timersub(&cur_time2, &cur_time1, &tdiff);
	double dif = tdiff.tv_sec * 1000 + tdiff.tv_usec / 1000;
	cout << "stepper drive time: "<< dif << endl;
	
	bar.GetTemp(); 
	cout << "pressure: " << bar.GetPressure() << endl;
	
	int fd = -1;
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		printf("Failed to open the bus.");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE_FORCE, 0x1a) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return -1;
	}
	
	//pibot.CreateEncoders();
	//PiBot::encoders.push_back(Encoder(12, 16));
	
	//wiringPiI2CWrite(_i2cFd, 0x01);
	//int reg = I2cRead16(fd, 1);
	unsigned char buf[2];
	//wiringPiI2CWrite(fd, 4); 
	buf[0] = 0x01; buf[1] = 0x1f; // reg 0
	//write(fd, buf, 2);
	usleep(100);
	buf[0] = 0x08; buf[1] = 0x15; // reg 4
	write(fd, buf, 2);
	usleep(100);
	buf[0] = 0x0c; buf[1] = 0x00; // reg 6
	//write(fd, buf, 2);
	/*int res = buf[0];
	res <<= 8;
	res |= buf[1];
	cout << "reg: " << ret <<"  "<< res << endl;*/
	uint8_t motorEmf1 = 220, motorEmf2 = 220;
	int prevCnt[2];
	int i = 0;
	while (1) {
		
		k=kbhit();
        if (k!=0)
        {
			c[2] = c[1];
			c[1] = c[0];
            c[0]=fgetc(stdin);
            if (c[0]=='q') break;
			if (c[2] == '\033' && c[1] == '[') { // if the first value is esc
				//getch(); // skip the [
				switch(c[0]) { // the real value
					case 'A':
						// code for arrow up
						if (lightLevel < 4096) lightLevel *= 2; 
						break;
					case 'B':
						// code for arrow down
						if (lightLevel>=2) lightLevel /= 2; 
						break;
					case 'C':
						// code for arrow right
						break;
					case 'D':
						// code for arrow left
						break;
				}
			}
        }
        fprintf(stderr,"%d \n",k);
		//wiringPiI2CWriteReg8(_pca9685Fd, 0x3b, ++i&0x0F);
		//pibot.SetPWM(16, lightLevel-1);

		//cout << "Light Level: "<<lightLevel<<endl;
		
		//cout << "GPIO5: "<<digitalRead(5)<<endl;
		//cout << "GPIO16: "<<digitalRead(16)<<endl;
		//cout << "GPIO25: "<<digitalRead(25)<<endl;
	
		//cout << "magX: " << magacc.GetMagX()<< " magY: " << magacc.GetMagY()<< " magZ: " << magacc.GetMagZ() << endl;
		cout << "accX: " << magacc.GetAccX()<< " accY: " << magacc.GetAccY()<< " accZ: " << magacc.GetAccZ() << endl;
		bar.GetTemp();
		//cout << "range [cm]: " << pibot.GetRangeCm(26, 25, 340) << endl;
		cout << "range1 [cm]: " << pibot.dist1 << "  range2 [cm]: " << pibot.dist2 << "  range3 [cm]: " << pibot.dist3 << endl;
		pibot.SonarTrigger(26);
		//pibot.encoders[0]._UpdateIsrCb(pibot.encoders[0]);
		float speedW1 =  1000000000.0 / (20 * pibot.encoders[1]->pulsPeriodNs); // float(pibot.encoders[1]->counter - prevCnt[1]) / 2; //
		float speedW2 = 1000000000.0 / (20 * pibot.encoders[0]->pulsPeriodNs); // float(pibot.encoders[0]->counter - prevCnt[0]) / 2; //
		/*if ((i>>8)&8)*/ cout << "encoder: "<< (int)motorEmf1 <<", "<< (int)motorEmf2 << ", speedW1 "<< speedW1<< ", speedW2 "<< speedW2 << endl;
		if (speedW1 > 2 ) {
			if (motorEmf1 > 0) motorEmf1 --;
		} else {
			if (motorEmf1 < 255) motorEmf1 ++;
		}
		if (speedW2 > 2 ) {
			if (motorEmf2 > 0) motorEmf2 --;
		} else {
			if (motorEmf2 < 255) motorEmf2 ++;
		}
		pibot.SetSpeed(1, 0, motorEmf1);
		pibot.SetSpeed(2, 0, motorEmf2);
		
		prevCnt[0] = pibot.encoders[0]->counter;
		prevCnt[1] = pibot.encoders[1]->counter;
		
		adc.Convert();
		usleep(1000000);
		PiBot::PowerControl(1);
		//objCallback();
		
		if( quit.load() ) break;    // exit normally after SIGINT
	}
	nonblock(NB_DISABLE);
	return 0;
}