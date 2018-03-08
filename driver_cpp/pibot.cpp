/*****************************************************************
File: pibot.cpp
Version: 1.0

Author: Milan Neskovic 2016-2018, milan@pi-supply.com

Description:
	Implements C++ Classes for interfacing to PiBot Boards based 
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

#include "pibot.h"
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <math.h>
#include <sys/time.h> 

MotorDriver::MotorDriver(bool paralellMode){
	wiringPiSetupGpio(); // Initialize wiringPi
	pinMode(22,  OUTPUT); // deactivate enable
	digitalWrite(22, LOW);
	// Initialize PCA9634
	pca9634Fd = wiringPiI2CSetup(PCA9634_ADDR);
	wiringPiI2CWriteReg8(pca9634Fd, 0x00, 0x00);
	wiringPiI2CWriteReg8(pca9634Fd, 0x01, paralellMode?0x12:0x14);
	wiringPiI2CWriteReg8(pca9634Fd, 0x0c, 0x55);
	wiringPiI2CWriteReg8(pca9634Fd, 0x0d, 0x55);
	
	_motorStateRegs[0] = _motorStateRegs[1] = 0;
	
	usleep(5000);
	digitalWrite(22, HIGH); // enable power
	usleep(2000);
	pinMode(22,  INPUT);//
	pullUpDnControl(22, PUD_UP);
	usleep(5000);
	wiringPiI2CWriteReg8(pca9634Fd, 0x01, 0x14);
	std::cout<<"MotorDriver"<<std::endl;
}

MotorDriver::~MotorDriver(){
	close(pca9634Fd);
}

int MotorDriver::DriveOutput(uint8_t output, int16_t emfLevel) {
	uint8_t ind = (output - 1) & 0x03;
	int pwmReg1 = (ind << 1) + 0x02;
	int pwmReg2 = (ind << 1) + 0x03;
	int stateReg = (ind&0x02)?0x0d:0x0c;
	uint8_t stateRegShift1 = (ind&0x01)?4:0;
	uint8_t stateRegShift2 = stateRegShift1 + 2;
	//printf("stateRegShift1 = %d  \n", stateRegShift1);
	uint8_t &motorStateReg = _motorStateRegs[(ind&0x02)>>1];
	uint8_t prevState = motorStateReg;
	motorStateReg &= ~((0x03 << stateRegShift1) | (0x03 << stateRegShift2));
	uint8_t emfReg = emfLevel > 0 ? emfLevel : -emfLevel;
	if (emfLevel >= 0) { // forward
		motorStateReg |= ((0x02 << stateRegShift1) | (0x00 << stateRegShift2)); // first drive line pwm, second high state
		if (prevState != motorStateReg) wiringPiI2CWriteReg8(pca9634Fd, stateReg, motorStateReg );
		//std::cout << std::hex <<"motorStateReg: " << (int)motorStateReg<<std::endl;
		if (_motorSpeedReg1[ind] != emfReg) {
			wiringPiI2CWriteReg8(pca9634Fd, pwmReg1, emfReg );
			_motorSpeedReg1[ind] = emfReg;
		}
	} else {
		motorStateReg |= (0x00 << stateRegShift1) | (0x02 << stateRegShift2); // first drive high state, second pwm
		if (prevState != motorStateReg) wiringPiI2CWriteReg8(pca9634Fd, stateReg, motorStateReg );
		if (_motorSpeedReg2[ind] != emfReg) {
			wiringPiI2CWriteReg8(pca9634Fd, pwmReg2, emfReg );
			_motorSpeedReg2[ind] = emfReg;
		}
	}

	return 0;
}

StepperDriver::StepperDriver(MotorDriver& driver, const uint8_t &output1, const uint8_t &output2):
	_driver(driver),
	_out1(output1),
	_out2(output2)
{
}

int32_t StepperDriver::DriveHalfSteps(int32_t steps, uint32_t periodUs, uint8_t torque) {
	
	//SetSpeed(3, 0, 255);
	/*SetSpeed(4, 0, 0);
	usleep(periodUs);
	SetSpeed(4, 1, 255);
	usleep(periodUs);
	SetSpeed(3, 0, 0);
	usleep(periodUs);
	SetSpeed(3, 1, 255);
	usleep(periodUs);
	SetSpeed(4, 1, 0);
	usleep(periodUs);
	SetSpeed(4, 0, 255);
	usleep(periodUs);
	SetSpeed(3, 1, 0);
	usleep(periodUs);
	SetSpeed(3, 0, 255);
	usleep(periodUs);*/
	
	return 4;
}

int32_t StepperDriver::DriveSteps(int32_t steps, uint32_t periodUs, uint8_t torque) {
	int32_t absSteps = steps > 0 ? steps : -steps;
	struct timeval tv;
	
	for (int32_t i = 0; i < absSteps; i++) {
		gettimeofday(&tv, 0);
		uint32_t passedTime = (tv.tv_sec-_stepTime.tv_sec)*1000000 + tv.tv_usec-_stepTime.tv_usec;
		_stepTime = tv;
		if (i==0 && (_prevPeriod != periodUs || passedTime > periodUs*2) ) {
			refTime = tv;
			_nextTime = periodUs;
		} else {
			int32_t drive_time = (_stepTime.tv_sec-refTime.tv_sec)*1000000 + _stepTime.tv_usec-refTime.tv_usec;
			_nextTime += periodUs;
			if (drive_time < _nextTime) usleep(_nextTime - drive_time);
		}
		
		if (steps > 0) {
			_driver.DriveOutput(_out1, _step&0x02 ? torque : - torque);
			_driver.DriveOutput(_out2, (_step+1)&0x02 ? torque : - torque);
			_step++;
		} else {
			_driver.DriveOutput(_out1, (_step-1)&0x02 ? torque : - torque);
			_driver.DriveOutput(_out2, _step&0x02 ? torque : - torque);
			_step--;
		}
	}
	_prevPeriod = periodUs;
	return _step;
	/*for (int i = 1; i <= steps; i++) {
		SetSpeed(coil1, 0, 255);
		SetSpeed(coil2, 1, 255);
		usleep(periodUs);
		SetSpeed(coil1, dir, 255);
		SetSpeed(coil2, dir, 255);
		usleep(periodUs);
		SetSpeed(coil1, 1, 255);
		SetSpeed(coil2, 0, 255);
		usleep(periodUs);
		SetSpeed(coil1, !dir, 255);
		SetSpeed(coil2, !dir, 255);
		usleep(periodUs);
	}*/
}

//std::vector<Encoder> encoders;

void PiBot::UpdateEncoders()
{
    //for (uint8_t i = 0; i < encoders.size(); i++) encoders[i].Update();
}

std::function<void()> _objCbFunc[4];

void _PinCallback0() { _objCbFunc[0](); }
void _PinCallback1() { _objCbFunc[1](); }
void _PinCallback2() { _objCbFunc[2](); }
void _PinCallback3() { _objCbFunc[3](); }

static PinCallbackT _pinCbs[4] = {_PinCallback0, _PinCallback1, _PinCallback2, _PinCallback3};
static int _pinCbCount = 0;

void ObjWiringPiISR(int val, int mask, std::function<void()> callback)
//void Encoder::_EncWiringPiISR(int val, int mask)
{
  //_objCbFunc[n] = callback;
  //wiringPiISR(val, mask, &objCallback);
  if (_pinCbCount > 3) return;
  _objCbFunc[_pinCbCount] = callback;
  wiringPiISR(val, mask, _pinCbs[_pinCbCount]);
  _pinCbCount ++;
  std::cout<<"_pinCbCount "<<(int)_pinCbCount << ", " << val << std::endl;
}

Encoder::Encoder(int8_t pinA, int8_t pinB):
	pin_a(pinA),
	pin_b(pinB),
	counter(0),
	lastEncoded(0)
{
	if (pinA < 0) return;
    pinMode(pin_a, INPUT); 
	pullUpDnControl(pin_a, PUD_UP);
    
	if (pin_b >= 0) {
		pinMode(pin_b, INPUT);
		pullUpDnControl(pin_b, PUD_UP);
		//wiringPiISR(pin_a,INT_EDGE_BOTH, PiBot::UpdateEncoders);
		//objWiringPiISR(pin_a, INT_EDGE_BOTH, std::bind(&Encoder::_UpdateIsrCb, this));
		//wiringPiISR(pin_b,INT_EDGE_BOTH, PiBot::UpdateEncoders);
		//objWiringPiISR(pin_b, INT_EDGE_BOTH, std::bind(&Encoder::_UpdateIsrCb, this));
		ObjWiringPiISR(pin_a, INT_EDGE_BOTH, std::bind(&Encoder::_UpdateIsrCb, this));
		ObjWiringPiISR(pin_b, INT_EDGE_BOTH, std::bind(&Encoder::_UpdateIsrCb, this));
	} else {
		ObjWiringPiISR(pin_a, INT_EDGE_FALLING, std::bind(&Encoder::_UpdateCounterIsrCb, this));
		//_EncWiringPiISR(pin_a, INT_EDGE_FALLING);
	}
	//std::cout<<"this "<<(int)this<<std::endl;
	//objCallback();
}

void Encoder::_UpdateCounterIsrCb(Encoder *encoder) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	encoder->pulsPeriodNs = (tickNow - encoder->_tick).count();
	encoder->_tick = tickNow;
	encoder->counter++;
	//std::cout<<"encoder triggered "<<(int)encoder<<", "<<encoder->counter<<std::endl;
}

void Encoder::_UpdateIsrCb(Encoder *encoder) {
	int MSB = digitalRead(encoder->pin_a);
	int LSB = digitalRead(encoder->pin_b);

	int encoded = (MSB << 1) | LSB;
	int sum = (encoder->lastEncoded << 2) | encoded;

	if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder->counter++;
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder->counter--;

	encoder->lastEncoded = encoded;
	//if (MSB || LSB) std::cout<<"encoder triggered"<<MSB<<LSB<<std::endl;
	//std::cout<<"encoder triggered "<<(int)encoder<<", "<<encoder->counter<<std::endl;
}

/*void PiBot::CreateEncoders() {
	encoders.push_back(Encoder(12, 16));
}*/

PiBot::PiBot(bool paralellMode)
{
	//wiringPiSetupGpio(); // Initialize wiringPi

	/*pinMode(22,  OUTPUT); // deactivate enable
	digitalWrite(22, LOW);*/
	
	// Initialize PCA9634
	/*pca9634Fd = wiringPiI2CSetup(PCA9634_ADDR);
	wiringPiI2CWriteReg8(pca9634Fd, 0x00, 0x00);
	wiringPiI2CWriteReg8(pca9634Fd, 0x01, paralellMode?0x16:0x14);
	wiringPiI2CWriteReg8(pca9634Fd, 0x0c, 0xaa);
	wiringPiI2CWriteReg8(pca9634Fd, 0x0d, 0xaa);
	
	_motorStateRegs[0] = _motorStateRegs[1] = 0;*/
	
	// Initialize PCA9685
	if ((_pca9685Fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		printf("Failed to open the bus.");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return;
	}

	if (ioctl(_pca9685Fd, I2C_SLAVE_FORCE, PCA9685_ADDR) < 0) {
		printf("Failed to acquire bus access and/or talk to slave.\n");
		/* ERROR HANDLING; you can check errno to see what went wrong */
		return;
	}
	wiringPiI2CWriteReg8(_pca9685Fd, 0x00, 0x10); // set sleep mode to change prescaler
	usleep(5000);
	//wiringPiI2CWriteReg8(_pca9685Fd, 0xFE, 0x03); // set prescale to 1526 Hz
	wiringPiI2CWriteReg8(_pca9685Fd, 0xFE, 0x79); // set prescale to 50 Hz, 20mS required by servos
	wiringPiI2CWriteReg8(_pca9685Fd, 0x00, 0x00);
	wiringPiI2CWriteReg8(_pca9685Fd, 0x01, 0x04);
	wiringPiI2CWriteReg8(_pca9685Fd, 0x3f, 0); //  aref1 on time
	
	//wiringPiI2CWriteReg8(_pca9685Fd, 0x3b, 1); // LED
	//wiringPiI2CWriteReg8(_pca9685Fd, 0x3d, 0);
	
	//wiringPiI2CWriteReg8(_pca9685Fd, 0x37, 2);
	//wiringPiI2CWriteReg8(_pca9685Fd, 0x39, 5);
	
	stepper[2][3] = new StepperDriver(_driver, MOTOR_DRIVER_OUTPUT_M3, MOTOR_DRIVER_OUTPUT_M4);
	
	encoders.push_back(new Encoder(5));
	encoders.push_back(new Encoder(16));
	
	std::cout<<encoders.size()<<std::endl;
	
	pinMode(4, INPUT);
	pullUpDnControl(4, PUD_UP);
	pinMode(27, INPUT);  
	pullUpDnControl(27, PUD_UP);
	pinMode(25, INPUT);
	pullUpDnControl(25, PUD_UP);
	ObjWiringPiISR(4, INT_EDGE_RISING, std::bind(&PiBot::_EchoIsrCb1, this));
	ObjWiringPiISR(27, INT_EDGE_RISING, std::bind(&PiBot::_EchoIsrCb2, this));
	ObjWiringPiISR(25, INT_EDGE_RISING, std::bind(&PiBot::_EchoIsrCb3, this));
	/*digitalWrite(22, HIGH);//pinMode(22,  INPUT); // enable power
	usleep(2000);
	wiringPiI2CWriteReg8(pca9634Fd, 0x01, 0x14);*/
	//objCallback();
}

PiBot::~PiBot(){
	pinMode(22,  OUTPUT); // deactivate enable
	digitalWrite(22, LOW); // disable power
	close(pca9634Fd);
	close(_pca9685Fd);
	std::cout<<std::endl<<"PiBot is off"<<std::endl;
}

void PiBot::PowerControl(uint8_t onOff) {
	if (onOff) { 
		// enable power
		pullUpDnControl(22, PUD_UP);
		//pinMode(22,  INPUT);
	} else {
		// deactivate enable
		//digitalWrite(22, LOW);
		//pinMode(22,  OUTPUT);
		pullUpDnControl(22, PUD_DOWN);
		//pinMode(22,  INPUT);
	}
}

int PiBot::SetPWM(uint8_t channel, uint16_t level) {
	uint8_t reg = 0x06 + (channel-1) * 4;
	wiringPiI2CWriteReg8(_pca9685Fd, reg, 0); // LED
	uint8_t v = level >> 8;
	wiringPiI2CWriteReg8(_pca9685Fd, reg + 1, 0);
	wiringPiI2CWriteReg8(_pca9685Fd, reg + 2, level); // LED
	wiringPiI2CWriteReg8(_pca9685Fd, reg + 3, v);
	/*uint8_t data[5] = {0, 0, 0, 0};
	data[0] = 0x06 + (channel-1) * 4;
	write(_pca9685Fd, data, 1);
	data[0] = level & 0xFF;
	data[1] = (level >> 8) & 0x0F;
	write(_pca9685Fd, data, 4);*/
	return 0;
}

int PiBot::SetSpeed(unsigned char motor, unsigned char dir, unsigned char speed){
	return _driver.DriveOutput(motor, dir ? speed: -speed);
	/*uint8_t ind = (motor - 1) & 0x03;
	int pwmReg1 = (ind << 1) + 0x02;
	int pwmReg2 = (ind << 1) + 0x03;
	int stateReg = (ind&0x02)?0x0d:0x0c;
	uint8_t stateRegShift1 = (ind&0x01)?4:0;
	uint8_t stateRegShift2 = stateRegShift1 + 2;
	//printf("stateRegShift1 = %d  \n", stateRegShift1);
	uint8_t &motorStateReg = _motorStateRegs[(ind&0x02)>>1];
	uint8_t prevState = motorStateReg;
	motorStateReg &= ~((0x03 << stateRegShift1) | (0x03 << stateRegShift2));
	
	if (dir == 0) { // forward
		motorStateReg |= ((0x02 << stateRegShift1) | (0x00 << stateRegShift2)); // first drive line pwm, second high state
		if (prevState != motorStateReg) wiringPiI2CWriteReg8(pca9634Fd, stateReg, motorStateReg );
		//std::cout << std::hex <<"motorStateReg: " << (int)motorStateReg<<std::endl;
		if (_motorSpeedReg1[ind] != speed) {
			wiringPiI2CWriteReg8(pca9634Fd, pwmReg1, speed );
			_motorSpeedReg1[ind] = speed;
		}
	} else {
		motorStateReg |= (0x00 << stateRegShift1) | (0x02 << stateRegShift2); // first drive high state, second pwm
		if (prevState != motorStateReg) wiringPiI2CWriteReg8(pca9634Fd, stateReg, motorStateReg );
		if (_motorSpeedReg2[ind] != speed) {
			wiringPiI2CWriteReg8(pca9634Fd, pwmReg2, speed );
			_motorSpeedReg2[ind] = speed;
		}
	}

	return 0;*/
}

int PiBot::SetCurrentLimit(unsigned char value){
	uint8_t r = (float)value * 16 / 100;
	if (r == 16) {
		wiringPiI2CWriteReg8(_pca9685Fd, 0x43, 0x00);
		wiringPiI2CWriteReg8(_pca9685Fd, 0x3f, 0x10);
	} else {
		wiringPiI2CWriteReg8(_pca9685Fd, 0x43, r);
		wiringPiI2CWriteReg8(_pca9685Fd, 0x3f, 0x00);
	}
	return 0;
}

float PiBot::GetRangeCm(int triggerPin, int echoPin, float velocity) {
	pinMode(echoPin,  INPUT);
	pullUpDnControl(echoPin, PUD_UP);
	pinMode(triggerPin,  OUTPUT);
	digitalWrite(triggerPin, HIGH);   // trigger
	
	usleep(30);
	digitalWrite(triggerPin, LOW);
	
	struct timeval cur_time1, cur_time2;
	int cnt = 0;
	while( (digitalRead(echoPin)==1) && cnt++ < 100 ) usleep(10);
	gettimeofday(&cur_time1,NULL);
	cnt = 0;
	while( (digitalRead(echoPin)==0) && cnt++ < 35000 ) usleep(10);
	gettimeofday(&cur_time2,NULL);
	
	return ( (double)(cur_time2.tv_usec - cur_time1.tv_usec) / 1000000 + cur_time2.tv_sec - cur_time1.tv_sec ) * velocity / 2 * 100;
}

void PiBot::_EchoIsrCb1(PiBot *bot) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	double echoPeriodNs = (tickNow - bot->_triggerTime1).count();
	bot->dist1 = echoPeriodNs * 340 / 1000000000 / 2 * 100;
	//bot->_triggerTime1 = tickNow;
	//std::cout<<"echotime1 "<< echoPeriodNs << std::endl;
}

void PiBot::_EchoIsrCb2(PiBot *bot) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	double echoPeriodNs = (tickNow - bot->_triggerTime2).count();
	bot->dist2 = echoPeriodNs * 340 / 1000000000 / 2 * 100;
	//std::cout<<"echotime2 "<< echoPeriodNs << std::endl;
}

void PiBot::_EchoIsrCb3(PiBot *bot) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	double echoPeriodNs = (tickNow - bot->_triggerTime3).count();
	bot->dist3 = echoPeriodNs * 340 / 1000000000 / 2 * 100;
	std::cout<<"echotime3 "<< echoPeriodNs << std::endl;
}

void PiBot::SonarTrigger(int triggerPin) {
	pinMode(triggerPin,  OUTPUT);
	digitalWrite(triggerPin, HIGH);
	usleep(15);                
	digitalWrite(triggerPin, LOW); 
	_triggerTime1 = _triggerTime2 = _triggerTime3 = std::chrono::high_resolution_clock::now();
}

MagAcc::MagAcc() {
	_magI2cFd = wiringPiI2CSetup(MAG_ADDR);
	_accI2cFd = wiringPiI2CSetup(ACC_ADDR);
	// Setup control registers for reading
	wiringPiI2CWriteReg8(_accI2cFd, 0x20, 0b00100111); // CTRL_1, enable x y z axis data, 50 hz sampling
	wiringPiI2CWriteReg8(_accI2cFd, 0x23, 0x00); // CTRL_4, set +/- 2g full scale
	wiringPiI2CWriteReg8(_accI2cFd, 0x24, 0x01); // CTRL_5,  open drain interrupt signal
	//wiringPiI2CWriteReg8(_i2cFd, 0x24, 0b11100100); // CTRL_5, high resolution mode, thermometer off, 6.25hz ODR
	wiringPiI2CWriteReg8(_magI2cFd, 0x21, 0b01100011); // CTRL_2, set +/- 16 gauss full scale, reboot memory, reset mag registers
	wiringPiI2CWriteReg8(_magI2cFd, 0x20, 0b01011000); // CTRL_2, High-performance mode, 40Hz,
	wiringPiI2CWriteReg8(_magI2cFd, 0x22, 0x00); // CTRL_3, Continuous-conversion mode
	wiringPiI2CWriteReg8(_magI2cFd, 0x23, 0x08); // CTRL_4, get magnetometer out of low power mode
	
	_accFs = 0;
	_magFs = 1;
}

MagAcc::~MagAcc() {
	close(_accI2cFd);
	close(_magI2cFd);
}

float MagAcc::_accFsSensMap[] = {0.061, 0.122, 0.183, 0.244, 0.732};

int MagAcc::SetAccFs(unsigned char fs) {
	_accFs = fs<=4?fs:4;
	return wiringPiI2CWriteReg8(_accI2cFd, 0x23, _accFs << 3);
}

float MagAcc::_magFsSensMap[] = {0.080, 0.160, 0.320, 0.479};

int MagAcc::SetMagFs(unsigned char fs) {
	_magFs = fs&0x03;
	return wiringPiI2CWriteReg8(_magI2cFd, 0x21, _magFs << 5);
}

int GetTwosComp(uint8_t msb, uint8_t lsb) {
    int32_t twosComp = msb;
	twosComp <<= 8;
	twosComp += lsb;
    if (twosComp >= 32768)
        return twosComp - 65536;
    else
        return twosComp;
}

float MagAcc::GetMagX(){
	int mag = GetTwosComp(	wiringPiI2CReadReg8(_magI2cFd, 0x29), // msb byte
							wiringPiI2CReadReg8(_magI2cFd, 0x28) ); // lsb byte
	return mag * _magFsSensMap[_magFs] / 1000; // convert to gauss
}

float MagAcc::GetMagY(){
	int mag = GetTwosComp(	wiringPiI2CReadReg8(_magI2cFd, 0x2b), // msb byte
							wiringPiI2CReadReg8(_magI2cFd, 0x2a) ); // lsb byte
	return mag * _magFsSensMap[_magFs] / 1000; // convert to gauss
}

float MagAcc::GetMagZ(){
	int mag = GetTwosComp(	wiringPiI2CReadReg8(_magI2cFd, 0x2d), // msb byte
							wiringPiI2CReadReg8(_magI2cFd, 0x2c) ); // lsb byte
	return mag * _magFsSensMap[_magFs] / 1000; // convert to gauss
}

float MagAcc::GetAccX() {
	int acc = GetTwosComp(	wiringPiI2CReadReg8(_accI2cFd, 0x29), // msb byte
							wiringPiI2CReadReg8(_accI2cFd, 0x28) ); // lsb byte
	return acc * _accFsSensMap[_accFs] / 1000; // convert to g units
}

float MagAcc::GetAccY() {
	int acc = GetTwosComp(	wiringPiI2CReadReg8(_accI2cFd, 0x2b), // msb byte
							wiringPiI2CReadReg8(_accI2cFd, 0x2a) ); // lsb byte
	return acc * _accFsSensMap[_accFs] / 1000;
}

float MagAcc::GetAccZ() {
	int acc = GetTwosComp(	wiringPiI2CReadReg8(_accI2cFd, 0x2d), // msb byte
							wiringPiI2CReadReg8(_accI2cFd, 0x2c) ); // lsb byte
	return acc * _accFsSensMap[_accFs] / 1000;
}

float MagAcc::GetTemp() {
	return GetTwosComp(	wiringPiI2CReadReg8(_accI2cFd, 0x06),
						wiringPiI2CReadReg8(_accI2cFd, 0x05) ) / 8;
}

uint16_t I2cRead16(int fd, int reg) {
	uint8_t buf[2];
	wiringPiI2CWrite(fd, reg); 
	usleep(100);
	read(fd, buf, 2);
	int res = buf[0];
	res <<= 8;
	res |= buf[1];
	return res;
}

int I2cWrite16(int fd, uint8_t reg, uint16_t value) {
	uint8_t buf[3] = {reg, (uint8_t)(value>>8), (uint8_t)value};
	return write(fd, buf, 3);
}

uint32_t I2cRead24(int fd, int reg) {
	uint8_t buf[3];
	wiringPiI2CWrite(fd, reg); 
	usleep(100);
	read(fd, buf, 3);
	uint32_t res = buf[0] << 8;
	res |= buf[1];
	res <<= 8;
	res |= buf[2];
	return res;
}

Barometer::Barometer() {
	_i2cFd = wiringPiI2CSetup(BAR_ADDR);
	wiringPiI2CWriteReg8(_i2cFd, 0xE0, 0xB6);//wiringPiI2CWrite(_i2cFd, 0x1E); // reset
	usleep(100000);
	/*_c1 = I2cRead16(_i2cFd, 0xa2);
	_c2 = I2cRead16(_i2cFd, 0xa4);
	_c3 = I2cRead16(_i2cFd, 0xa6);
	_c4 = I2cRead16(_i2cFd, 0xa8);
	_c5 = I2cRead16(_i2cFd, 0xaa);
	_c6 = I2cRead16(_i2cFd, 0xac);*/
	for (int i = 0; i < 9; i++) 
		dig_P[i] = I2cRead16(_i2cFd, 0x8E + i*2);
	for (int i = 0; i < 3; i++) 
		dig_T[i] = I2cRead16(_i2cFd, 0x88 + i*2);
	
	wiringPiI2CWriteReg8(_i2cFd, 0xF4, 0x27); // control
	usleep(200000);
	std::cout << std::dec << "dig_T[0]: " << dig_T[0] << "  dig_T[1]: " << dig_T[1] << std::endl; 
}

Barometer::~Barometer() {
	close(_i2cFd);
}

int32_t Barometer::_Compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T[0]<<1))) * ((int32_t)dig_T[1])) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T[0])) * ((adc_T>>4) - ((int32_t)dig_T[0]))) >> 12) *
	((int32_t)dig_T[2])) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

float Barometer::GetTemp(){
	/*wiringPiI2CWrite(_i2cFd, 0x58); // Convert D2 (OSR=4096)
	usleep(10000);
	int val = I2cRead24(_i2cFd, 0x00); // read cmd
	
	int64_t dt = val - _c5 * (1L<<8);;
	float temp = (2000 + (dt * _c6) / (1L<<23));// * 0.012;
	// Second order temperature compensation 
	int64_t t2;
	if(temp >= 2000) {
		// High temperature 
		t2 = 5 * (dt * dt) / (1LL<<38);
	} else {
		// Low temperature 
		t2 = 3 * (dt * dt) / (1LL<<33);
	}

	float temperature = (float)(temp - t2) / 100;
	*/
	int val = I2cRead24(_i2cFd, 0xFA) >> 4;
	/*val = wiringPiI2CReadReg8(_i2cFd, 0xFA);
	val <<= 8;
	val |= wiringPiI2CReadReg8(_i2cFd, 0xFB);
	val <<= 8;
	val |= wiringPiI2CReadReg8(_i2cFd, 0xFC);
	val >>= 4;*/
	int32_t temperature = _Compensate_T_int32(val);
	//std::cout << std::hex << (int)buf[0] << " ," << (int)buf[1] <<" ," << (int)buf[2] << std::endl; 
	std::cout << std::dec << "raw temp:" << val <<"  bar temp: " << temperature << std::endl; 
	
	return temperature;
}

float Barometer::GetPressure(){
	wiringPiI2CWrite(_i2cFd, 0x58); // Convert D2 (OSR=4096)
	usleep(10000);
	int val = I2cRead24(_i2cFd, 0x00); // read cmd
	
	int64_t dt = val - _c5 * (1L<<8);;
	float temp = (2000 + (dt * _c6) / (1L<<23));// * 0.012;
	
	wiringPiI2CWrite(_i2cFd, 0x48); // Convert D1 (OSR=4096)
	usleep(10000);
	int32_t d1 = I2cRead24(_i2cFd, 0x00); // read cmd

    int64_t off = _c2 * (1LL<<17) + (_c4 * dt) / (1LL<<6);
    int64_t sens = _c1 * (1LL<<16) + (_c3 * dt) / (1LL<<7);

    /* Second order temperature compensation for pressure */
    if(temp < 2000) {
      /* Low temperature */
      int32_t tx = temp-2000;
      tx *= tx;
      int32_t off2 = 61 * tx / (1<<4);
      int32_t sens2 = 29 * tx / (1<<4);
      if(temp < -1500) {
        /* Very low temperature */
        tx = temp+1500;
        tx *= tx;
        off2 += 17 * tx;
        sens2 += 9 * tx;
      }
      off -= off2;
      sens -= sens2;
	}

    int32_t p = ((int64_t)d1 * sens/(1LL<<21) - off) / (1LL << 15);
    float pressure = (float)p / 100;
	return pressure;
}

ADConverter::ADConverter() {
	_i2cFd = wiringPiI2CSetup(ADC_ADDR);
	//wiringPiI2CWrite(_i2cFd, 0x01); // set pointer to config register
	//I2cWrite16(_i2cFd, 0x01, 0x9383);
	I2cWrite16(_i2cFd, 0x01, 0xE383);
	uint8_t data[3] = {0x00, 0x00, 0x00};
	data[0] = 0; data[1] = 0;
	usleep(2000);
	wiringPiI2CWrite(_i2cFd, 0x00);
	read(_i2cFd, data, 2);
	int val = I2cRead16(_i2cFd, 0x00);
	float vin = (val>>3); // mV
	std::cout << std::hex << "adch: " << (int)data[0] << "  adcl: " << (int)data[1] << std::endl; 
	std::cout << "vin: " << vin << std::endl; 
}

ADConverter::~ADConverter() {
	close(_i2cFd);
}

uint16_t ADConverter::GetRawConversion() {
	return I2cRead16(_i2cFd, 0x00);
}

float ADConverter::Convert() {
	I2cWrite16(_i2cFd, 0x01, 0xC383);
	usleep(2000);
	int val = I2cRead16(_i2cFd, 0x00);
	float vin = (val>>3); // mV
	std::cout << "vin: " << vin << std::endl; 
}
