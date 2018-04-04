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

using namespace pibot;

#define MOTOR_DRIVER_OUTPUT_A	0
#define MOTOR_DRIVER_OUTPUT_B	1

typedef void (*_PinCallbackT)();
void ObjWiringPiISR(int val, int mask, std::function<void()> callback);

std::function<void()> _objCbFunc[5];

void _PinCallback0() { _objCbFunc[0](); }
void _PinCallback1() { _objCbFunc[1](); }
void _PinCallback2() { _objCbFunc[2](); }
void _PinCallback3() { _objCbFunc[3](); }
void _PinCallback4() { _objCbFunc[4](); }

static _PinCallbackT _pinCbs[5] = {_PinCallback0, _PinCallback1, _PinCallback2, _PinCallback3, _PinCallback4};
static int _pinCbCount = 0;

void ObjWiringPiISR(int val, int mask, std::function<void()> callback)
{
  if (_pinCbCount > 4) return;
  _objCbFunc[_pinCbCount] = callback;
  wiringPiISR(val, mask, _pinCbs[_pinCbCount]);
  _pinCbCount ++;
}

PCA9634::PCA9634(int address) {
	// Initialize
	_fd = wiringPiI2CSetup(address);
	wiringPiI2CWriteReg8(_fd, 0x00, 0x00);
	wiringPiI2CWriteReg8(_fd, 0x01, 0x02);
	_states[0] = wiringPiI2CReadReg8(_fd, 0x0c);
	_states[1] = wiringPiI2CReadReg8(_fd, 0x0d);
}

PCA9634::~PCA9634() {
	wiringPiI2CWriteReg8(_fd, 0x00, 0x10); // Low power mode
	close(_fd);
} 

int PCA9634::Configure(bool inverted, outputDriveT outDrv, uint8_t outne) {
	uint8_t cfg = (inverted?0x10:0x00) | (uint8_t)outDrv | outne;
	//std::cout << "cfg " << (int)cfg << std::endl;
	wiringPiI2CWriteReg8(_fd, 0x01, cfg);
}

int PCA9634::SetState(uint8_t output, outputStateT state) {
	uint8_t groupId = (output&0x04)>>2;
	uint8_t stateRegShift = (output&0x03) * 2;
	uint8_t newStates = _states[groupId];
	newStates &= ~(0x03 << stateRegShift);
	newStates |= (uint8_t)state << stateRegShift;
	if (newStates != _states[groupId]) {
		wiringPiI2CWriteReg8(_fd, 0x0c+groupId, newStates);
		_states[groupId] = newStates;
	}
}

PCA9634::outputStateT PCA9634::GetState(uint8_t output){
	uint8_t s = output&0x04 ? (uint8_t)_states[1] >> ((output&0x03) * 2) 
		: (uint8_t)_states[0] >> ((output&0x03) * 2);
	return (PCA9634::outputStateT)s;
}

int PCA9634::SetGroupStates(uint8_t groupId, outputStateT s0, outputStateT s1, outputStateT s2, outputStateT s3) {
	uint8_t newStates  = (uint8_t)s0 | ((uint8_t)s1 << 2) | ((uint8_t)s2 << 4) | ((uint8_t)s3 << 6);
	//std::cout << "new states " << (int)newStates << std::endl;
	if (newStates != _states[groupId]) {
		wiringPiI2CWriteReg8(_fd, 0x0c+(groupId&1), newStates );
		_states[groupId] = newStates;
	}
}

int PCA9634::SetPulse(uint8_t output, uint8_t width) {
	if (_pw[output] != width) {
		wiringPiI2CWriteReg8(_fd, 0x02 + output, width );
		_pw[output] = width;
	}
}

PCA9685::PCA9685(int address) {
	// Initialize PCA9685 I2C
	_fd = wiringPiI2CSetup(address);
	wiringPiI2CWriteReg8(_fd, 0x00, 0x10); // set sleep mode to change prescaler
	usleep(5000);
	//wiringPiI2CWriteReg8(_pca9685Fd, 0xFE, 0x03); // set prescale to 1526 Hz
	wiringPiI2CWriteReg8(_fd, 0xFE, 0x79); // set prescale to 50 Hz, 20mS required by servos
	wiringPiI2CWriteReg8(_fd, 0x00, 0x00);
	wiringPiI2CWriteReg8(_fd, 0x01, 0x06); // totem-pole, high impedance when disabled
}

PCA9685::~PCA9685() {
	wiringPiI2CWriteReg8(_fd, 0x00, 0x10); // set sleep mode
	close(_fd);
}

int PCA9685::SetPulse(uint8_t channel, uint16_t timeOn, uint16_t timeOff) {
	uint8_t reg = 0x06 + (channel&0x0F) * 4;
	wiringPiI2CWriteReg8(_fd, reg, timeOn); //
	wiringPiI2CWriteReg8(_fd, reg + 1, timeOn >> 8);
	wiringPiI2CWriteReg8(_fd, reg + 2, timeOff); //
	wiringPiI2CWriteReg8(_fd, reg + 3, timeOff >> 8);
	return 0;
}

MotorDriver::MotorDriver(int id, PCA9634 &pwmDriver, bool paralellMode):
	_pwmDriver(pwmDriver)
{
	_id = id;
	pinMode(22,  OUTPUT); // deactivate enable
	digitalWrite(22, LOW);
	// Initialize PCA9634
	/*pca9634Fd = wiringPiI2CSetup(PCA9634_ADDR);
	wiringPiI2CWriteReg8(pca9634Fd, 0x00, 0x00);
	wiringPiI2CWriteReg8(pca9634Fd, 0x01, paralellMode?0x16:0x14);
	wiringPiI2CWriteReg8(pca9634Fd, 0x0c, 0x55);
	wiringPiI2CWriteReg8(pca9634Fd, 0x0d, 0x55);
	
	_motorStateRegs[0] = _motorStateRegs[1] = 0;*/
	if (paralellMode) {
		_pwmDriver.Configure(false, PCA9634::OPEN_DRAIN, 0x02);
		_pwmDriver.SetGroupStates(id, PCA9634::OFF, PCA9634::OFF, PCA9634::OFF, PCA9634::OFF); // high impedance
	} else {
		_pwmDriver.Configure(false, PCA9634::TOTEM_POLE, 0x00);
		_pwmDriver.SetGroupStates(id, PCA9634::OFF, PCA9634::OFF, PCA9634::OFF, PCA9634::OFF); // high state
	}

	usleep(5000);
	digitalWrite(22, HIGH); // enable power
	usleep(2000);
	pinMode(22,  INPUT);//
	pullUpDnControl(22, PUD_UP);
	usleep(5000);
	//wiringPiI2CWriteReg8(pca9634Fd, 0x01, 0x14);
	_pwmDriver.Configure(false, PCA9634::TOTEM_POLE, 0x00);
	_pwmDriver.SetGroupStates(id, PCA9634::OFF, PCA9634::OFF, PCA9634::OFF, PCA9634::OFF); // high state
}

MotorDriver::~MotorDriver(){
}

int MotorDriver::SetOutputLevel(DriverOutput output, int16_t level) {
	/*uint8_t ind = (output + _id*2) & 0x03;
	int pwmReg1 = (ind << 1) + 0x02;
	int pwmReg2 = (ind << 1) + 0x03;
	int stateReg = (ind&0x02)?0x0d:0x0c;
	uint8_t stateRegShift1 = (ind&0x01)?4:0;
	uint8_t stateRegShift2 = stateRegShift1 + 2;
	//printf("stateRegShift1 = %d  \n", stateRegShift1);
	uint8_t &motorStateReg = _motorStateRegs[(ind&0x02)>>1];
	uint8_t prevState = motorStateReg;
	motorStateReg &= ~((0x03 << stateRegShift1) | (0x03 << stateRegShift2));
	uint8_t emfReg = level > 0 ? level : -level;
	if (level >= 0) { // forward
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
	}*/
	int16_t pw;
	uint8_t out = (uint8_t)output;
	if (level >= 0) { // forward
		// first drive line pwm, second high state
		_inputStates[out*2] = PCA9634::OFF;
		_inputStates[out*2+1] = PCA9634::PWM;
		pw = level;
		_pwmDriver.SetPulse(_id*4+out*2+1, pw);
		 
	} else {
		// first drive high state, second pwm
		_inputStates[out*2] = PCA9634::PWM;
		_inputStates[out*2+1] = PCA9634::OFF;
		pw = -level;
		
		_pwmDriver.SetPulse(_id*4+out*2, pw);
	}
	_pwmDriver.SetGroupStates(_id, _inputStates[0], _inputStates[1], _inputStates[2], _inputStates[3]);
	return 0;
}

StepperDriver::StepperDriver(MotorDriver& driver):
	_driver(driver)/*,
	_out1(output1),
	_out2(output2)*/
{
}

int32_t StepperDriver::DriveSteps(int32_t steps, uint32_t periodUs, uint8_t driveLevel) {
	int32_t absSteps = steps > 0 ? steps : -steps;
	struct timeval tv;
	
	for (int32_t i = 0; i < absSteps; i++) {
		gettimeofday(&tv, 0);
		uint32_t passedTime = (tv.tv_sec-_stepTime.tv_sec)*1000000 + tv.tv_usec-_stepTime.tv_usec;
		_stepTime = tv;
		if (i==0 && (_prevPeriod != periodUs || passedTime > periodUs*2) ) {
			_refTime = tv;
			_nextTime = periodUs;
		} else {
			int32_t drive_time = (_stepTime.tv_sec-_refTime.tv_sec)*1000000 + _stepTime.tv_usec-_refTime.tv_usec;
			_nextTime += periodUs;
			if (drive_time < _nextTime) usleep(_nextTime - drive_time);
		}
		
		if (steps > 0) {
			_driver.SetOutputLevel(M1, _step&0x02 ? driveLevel : - driveLevel);
			_driver.SetOutputLevel(M2, (_step+1)&0x02 ? driveLevel : - driveLevel);
			_step++;
		} else {
			_driver.SetOutputLevel(M1, (_step-1)&0x02 ? driveLevel : - driveLevel);
			_driver.SetOutputLevel(M2, _step&0x02 ? driveLevel : - driveLevel);
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

/*void PiBot::UpdateEncoders()
{
    //for (uint8_t i = 0; i < encoders.size(); i++) encoders[i].Update();
}*/

Encoder::Encoder(int16_t countsPerRevolution, int8_t pinA, int8_t pinB):
	cpr(countsPerRevolution),
	pin_a(pinA),
	pin_b(pinB),
	counter(0),
	_lastEncoded(0)
{
	if (pinA < 0 || pinB < 0) return;
    pinMode(pin_a, INPUT); 
	pullUpDnControl(pin_a, PUD_UP);
	pinMode(pin_b, INPUT);
	pullUpDnControl(pin_b, PUD_UP);
	ObjWiringPiISR(pin_a, INT_EDGE_BOTH, std::bind(&Encoder::_UpdateIsrCb, this));
	ObjWiringPiISR(pin_b, INT_EDGE_BOTH, std::bind(&Encoder::_UpdateIsrCb, this));
}

Encoder::Encoder(int16_t countsPerRevolution, int8_t pin):
	cpr(countsPerRevolution),
	pin_a(pin),
	pin_b(-1),
	counter(0),
	_lastEncoded(0)
{
	if (pin < 0) return; 
    pinMode(pin_a, INPUT); 
	pullUpDnControl(pin_a, PUD_UP);
	ObjWiringPiISR(pin_a, INT_EDGE_FALLING, std::bind(&Encoder::_UpdateCounterIsrCb, this));
}

void Encoder::_UpdateCounterIsrCb(Encoder *encoder) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	encoder->_pulsePeriodNs = (tickNow - encoder->_tick).count();
	encoder->_tick = tickNow; 
	encoder->counter++;
	//std::cout<<"encoder triggered "<<(int)encoder<<", "<<encoder->counter<<std::endl;
}

void Encoder::_UpdateIsrCb(Encoder *encoder) {
	int MSB = digitalRead(encoder->pin_a);
	int LSB = digitalRead(encoder->pin_b);

	int encoded = (MSB << 1) | LSB;
	int sum = (encoder->_lastEncoded << 2) | encoded;

	if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder->counter++;
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder->counter--;

	encoder->_lastEncoded = encoded;
}

float Encoder::AngularSpeed() {
	return 1000000000.0 / (cpr * _pulsePeriodNs) * 2 * 3.14159;
}

float Encoder::LinearSpeed(float radius_mm) {
	return radius_mm * AngularSpeed() / 1000;
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
	float v = (val>>3); // mV
	//std::cout << "ain: " << v << std::endl; 
	return v;
}

SonarDriver::SonarDriver(uint8_t channel) {
	static int8_t echoPin[] = {4, 27, 25, 12, 17};
	int pin = echoPin[channel-1];
	pinMode(pin, INPUT);
	pullUpDnControl(pin, PUD_UP);
	ObjWiringPiISR(pin, INT_EDGE_RISING, std::bind(&SonarDriver::_EchoIsrCb, this));
}

void SonarDriver::Triggered() {
	_triggerTime = std::chrono::high_resolution_clock::now();
}
/*
SonarDriver::_EvalEcho(uint8_t id) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	double echoPeriodNs = (tickNow - sonar->_triggerTime[id]).count();
	sonar->dist[id] = echoPeriodNs * 340 / 1000000000 / 2 * 100;
	//bot->_triggerTime1 = tickNow;
	std::cout<<"echotime1 "<< echoPeriodNs << std::endl;
}*/

void SonarDriver::_EchoIsrCb(SonarDriver *sonar) {
	std::chrono::time_point<std::chrono::high_resolution_clock> tickNow = std::chrono::high_resolution_clock::now();
	double echoPeriodNs = (tickNow - sonar->_triggerTime).count();
	sonar->dist = echoPeriodNs * 340 / 1000000000 / 2 * 100;
	//bot->_triggerTime1 = tickNow;
	//std::cout<<"echotime1 "<< echoPeriodNs << std::endl;
}

PiBot::PiBot(bool watchdogMode)
{
	wiringPiSetupGpio(); // Initialize wiringPi
	pinMode(22,  OUTPUT); // deactivate enable
	digitalWrite(22, LOW);
	usleep(100);
	pinMode(22,  INPUT);
	pullUpDnControl(22, PUD_DOWN);
	
	_wdMode = watchdogMode;
	_lowPowerEvent = false;
	
	wiringPiISR(22, INT_EDGE_FALLING, _LowPowerCb);
	
	pinMode(26,  OUTPUT); // Sonars trigger
	
	_mDriver[0] = _mDriver[1] = NULL;
	_stepDrv[0] = _stepDrv[1] = NULL;
	//stepper[2][3] = new StepperDriver(*(_mDriver[1]), 2, 3);
	
	//encoders.push_back(new Encoder(5));
	//encoders.push_back(new Encoder(16));
	
	//std::cout<<encoders.size()<<std::endl;
	
	for (int i = 0; i < 5; i++ ) 
		_sonars[i] = NULL;
	
	//digitalWrite(22, HIGH);//pinMode(22,  INPUT); // enable power
}

PiBot::~PiBot(){
	Disable();
	delete _mDriver[0];
	delete _mDriver[1];
	delete _stepDrv[0];
	delete _stepDrv[1];
	for (int i = 0; i < 5; i++ )
		delete _sonars[i];
	
}

bool PiBot::_lowPowerEvent = false;
bool PiBot::_wdMode;

void PiBot::_LowPowerCb(void) {
	Disable();
	_lowPowerEvent = true;
	//printf("Low power \n");
	//PiBot::PowerControl(1);
}

void PiBot::Enable() {
	// discharge cap
	pinMode(22,  OUTPUT); 
	digitalWrite(22, HIGH);
	usleep(10); 
	// set to power monitor mode
	pinMode(22,  INPUT);
	if (_wdMode) {
		pullUpDnControl(22, PUD_OFF);
	} else {
		pullUpDnControl(22, PUD_UP);
	}
	if (digitalRead(22)==0) { 
		// unsuccessful try
		digitalWrite(22, LOW);
		pinMode(22,  OUTPUT);
		_lowPowerEvent = true;
	} else {
		_lowPowerEvent = false;
	}
}

void PiBot::Disable() {
	digitalWrite(22, LOW);
	pinMode(22,  OUTPUT); 
}

int PiBot::InitMotorDriver(DriverId driverId, bool paralellMode) {
	if (_mDriver[(uint8_t)driverId] != NULL) 
		delete _mDriver[(uint8_t)driverId];
	_mDriver[(uint8_t)driverId] = new MotorDriver(driverId, _pca9634);
	return 0;
}

int PiBot::InitStepperDriver(DriverId driverId) {
	if (_stepDrv[(uint8_t)driverId])
		delete _stepDrv[(uint8_t)driverId];
	_stepDrv[(uint8_t)driverId] = new StepperDriver(*(_mDriver[(uint8_t)driverId]));
	return 0;
}

StepperDriver& PiBot::Stepper(DriverId driverId) {
	return *_stepDrv[(uint8_t)driverId];
}

int PiBot::InitSonar(uint8_t channel) {
	if (_sonars[channel-1] != NULL)
		delete _sonars[channel-1];
	_sonars[channel-1] = new SonarDriver(channel);
}

float PiBot::SonarDistance(uint8_t channel) {
	return _sonars[channel-1]->GetDistance();
}

void PiBot::SonarTrigger() {
	digitalWrite(26, HIGH);
	usleep(15);                
	digitalWrite(26, LOW); 
	for (int i = 0; i < 5; i++ )
		if (_sonars[i]) _sonars[i]->Triggered();
}

int PiBot::SetPWM(uint8_t channel, float dutyCircle) {
	if (channel <= 16) {
		_pca9685.SetPulse(channel-1, 0, dutyCircle*4096);
	} else if (channel <= 20) {
		_pca9634.SetPulse(channel-17+4, dutyCircle*255.99);
		_pca9634.SetState(channel-17+4, PCA9634::PWM);
	}
	return 0;
}

int PiBot::SetLedDrive(uint8_t channel, float level) {
	if (channel <= 16) {
		_pca9685.SetPulse(channel-1, level*4096, 0);
	} else if (channel <= 20) {
		if (level == 0) {
			_pca9634.SetState(channel-17+4, PCA9634::ON);
		} else {
			_pca9634.SetState(channel-17+4, PCA9634::PWM);
			_pca9634.SetPulse(channel-17+4, 255 - level*255.99);
		}
		std::cout<<"level"<<(255 - level*255.99)<<std::endl;
	}
}

int PiBot::SetMotorDrive(DriverOutput output, int16_t level, DeacayMode deacayMode){
	return _mDriver[output/2]->SetOutputLevel((DriverOutput)(output%2), level);
}

int PiBot::SetCurrentDrive(uint8_t channel, float current_mA) {
	if (channel <= 16)
		_pca9685.SetPulse(channel-1, 0, current_mA * 4096 * 47 / 5000);
}

int PiBot::SetDriverLimit(DriverId driverId, float maxCurrent) {
	_pca9685.SetPulse(14+(uint8_t)driverId, maxCurrent, 0);
	return 0;
}

int PiBot::SetServoControl(uint8_t channel, uint16_t pulseWidthUs) {
	if (channel <= 16) {
		_pca9685.SetPulse(channel-1, 0, pulseWidthUs*4096/20000);
	}
}

/*float PiBot::GetRangeCm(int triggerPin, int echoPin, float velocity) {
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
}*/