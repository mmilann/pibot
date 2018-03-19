/*****************************************************************
File: pibot.h
Version: 1.0

Author: Milan Neskovic 2016-2018, milan@pi-supply.com

Description:
	Declares C++ Classes for interfacing to PiBot Boards based 
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
#ifndef PIBOT_H
#define PIBOT_H

#include <stdint.h>
#include <sys/time.h>
#include <vector>
#include <functional>
#include <chrono>

#define PCA9634_ADDR 0x3C
#define PCA9685_ADDR 0x40
#define MAG_ADDR 0x1E
#define ACC_ADDR 0x1D
#define BAR_ADDR 0x76
#define ADC_ADDR 0x49

#define PIBOT_M1	0
#define PIBOT_M2	1
#define PIBOT_M3	2
#define PIBOT_M4	3

#define MAX_ENCODERS	8

enum DriverOutput { M1=0, M2, M3, M4 };

void ObjWiringPiISR(int val, int mask, std::function<void()> callback);

namespace pibot {

typedef enum deacayModes {SLOW, FAST} deacayMode_t;

class PCA9634
{
public:
	enum outputDriveT { OPEN_DRAIN=0, TOTEM_POLE=4 };
	enum outputStateT { OFF=0, ON=1, PWM, PWM_GROUP };
	PCA9634(int address = PCA9634_ADDR);
	~PCA9634();
	int Configure(bool inverted, outputDriveT outDrv, uint8_t outne);
	int SetState(uint8_t output, outputStateT state);
	outputStateT GetState(uint8_t output);
	int SetGroupStates(uint8_t groupId, outputStateT s0, outputStateT s1, outputStateT s2, outputStateT s3);
	int SetPulse(uint8_t output, uint8_t width);
private:
	int _fd;
	uint8_t _states[2];
	uint8_t _pw[8];
};

class PCA9685
{
public:
	PCA9685(int address = PCA9685_ADDR);
	~PCA9685();
	int SetPulse(uint8_t channel, uint16_t timeOn, uint16_t timeOff);
private:
	int _fd;
};

class MotorDriver
{
public:
	MotorDriver(int id, PCA9634 &pwmDriver, bool paralellMode = false);
	~MotorDriver();
	int SetOutputLevel(uint8_t output, int16_t level);
	deacayMode_t decayMode;
private:
	int _id;
	//int pca9634Fd;
	PCA9634 &_pwmDriver;
	PCA9634::outputStateT _inputStates[4];
	/*unsigned char _motorStateRegs[2];
	unsigned char _motorSpeedReg1[2];
	unsigned char _motorSpeedReg2[2];*/
};

class StepperDriver 
{
public:
	StepperDriver(MotorDriver& driver, const uint8_t &output1, const uint8_t &output2);
	int32_t DriveSteps(int32_t steps, uint32_t periodUs, uint8_t torque = 255);
	int32_t DriveHalfSteps(int32_t steps, uint32_t periodUs, uint8_t torque = 255);
private:
	MotorDriver& _driver;
	const uint8_t _out1, _out2;
	int32_t _step;
	struct timeval _stepTime;
	struct timeval refTime;
	uint32_t _prevPeriod;
	int32_t _nextTime;
};
	
class Encoder
{
public:
	friend class PiBot;
	Encoder(int8_t pinA, int8_t pinB = -1);
	const int pin_a;
    const int pin_b; 
	int32_t counter;
	int32_t pulsPeriodNs;
private:
	static void _UpdateCounterIsrCb(Encoder *encoder);
	static void _UpdateIsrCb(Encoder *encoder);
	volatile int lastEncoded;
	std::chrono::time_point<std::chrono::high_resolution_clock> _tick;
};

class MagAcc
{
public:
	MagAcc();
	~MagAcc();
	int SetAccFs(unsigned char fs);
	int SetMagFs(unsigned char fs);
	float GetMagX();
	float GetMagY();
	float GetMagZ();
	float GetAccX();
	float GetAccY();
	float GetAccZ();
	float GetTemp();
private:
	int _magI2cFd;
	int _accI2cFd;
	unsigned char _accFs;
	static float _accFsSensMap[];
	unsigned char _magFs;
	static float _magFsSensMap[];
};

class Barometer
{
public:
	Barometer();
	~Barometer();
	float GetTemp();
	float GetPressure();
private:
	int32_t _Compensate_T_int32(int32_t adc_T);
	int _i2cFd;
	int _c1, _c2, _c3, _c4, _c5, _c6;
	int dig_T[3];
	int dig_P[10];
	int32_t t_fine;
};

class ADConverter
{
public:
	ADConverter();
	~ADConverter();
	uint16_t GetRawConversion();
	float Convert();
private:
	int _i2cFd;
};

class PiBot 
{
public:
	PiBot(bool paralellMode = false);
	~PiBot();
	int SetMotorDrive(DriverOutput output, int16_t level);
	//int SetWheelSpeed(unsigned char wheel, int16_t rpm);
	int SetCurrentLimit(uint8_t driverId, float maxCurrent);
	static void PowerControl(uint8_t onOff);
	//int32_t DriveSteps(uint8_t coil1, uint8_t coil2, int32_t steps, uint32_t periodUs, uint8_t torque = 255);
	//int32_t DriveHalfSteps(uint8_t coil1, uint8_t coil2, int32_t steps, uint32_t periodUs, uint8_t torque = 255);
	int SetPWM(uint8_t channel, float dutyCircle);
	int SetLedDrive(uint8_t channel, float level);
	float GetRangeCm(int triggerPin, int echoPin, float velocity = 340.0);
	void SonarTrigger(int triggerPin);
	StepperDriver *stepper[4][4];
	std::vector<Encoder*> encoders;
	ADConverter adc;
	MagAcc magacc;
	Barometer bar;
	PCA9634 _pca9634;
	float dist1, dist2, dist3;
private:
	static void _EchoIsrCb1(PiBot *bot);
	static void _EchoIsrCb2(PiBot *bot);
	static void _EchoIsrCb3(PiBot *bot);
	std::chrono::time_point<std::chrono::high_resolution_clock> _triggerTime1, _triggerTime2, _triggerTime3;
	
	PCA9685 _pca9685;
	//int pca9634Fd;
	//int _pca9685Fd;
	MotorDriver *_mDriver[2];
	/*int32_t _step[4][4];
	struct timeval _stepTime[4][4];
	struct timeval refTime[4][4];
	uint32_t prevPeriod[4][4];
	int32_t _nextT[4][4];*/
};

}
#endif // PIBOT_H
