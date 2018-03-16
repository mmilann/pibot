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

#define MOTOR_DRIVER_OUTPUT_M1	1
#define MOTOR_DRIVER_OUTPUT_M2	2
#define MOTOR_DRIVER_OUTPUT_M3	3
#define MOTOR_DRIVER_OUTPUT_M4	4

#define MAX_ENCODERS	8

typedef enum deacayModes {SLOW, FAST} deacayMode_t;

class MotorDriver
{
public:
	MotorDriver(bool paralellMode = true);
	~MotorDriver();
	int DriveOutput(uint8_t output, int16_t level);
	deacayMode_t decayMode;
private:
	int pca9634Fd;
	unsigned char _motorStateRegs[2];
	unsigned char _motorSpeedReg1[4];
	unsigned char _motorSpeedReg2[4];
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

typedef void (*PinCallbackT)();
void _PinCallback0();
void _PinCallback1();
void _PinCallback2();
void _PinCallback3();

void ObjWiringPiISR(int val, int mask, std::function<void()> callback);
	
class Encoder 
{
public:
	friend class PiBot;
	Encoder(int8_t pinA, int8_t pinB = -1);
	const int pin_a;
    const int pin_b; 
	int32_t counter;
	int32_t pulsPeriodNs;
	//static void _UpdateIsrCb(Encoder *encoder);
private:
	static void _UpdateCounterIsrCb(Encoder *encoder);
	//void _EncWiringPiISR(int val, int mask);
	static void _UpdateIsrCb(Encoder *encoder);
	//void Update();
	volatile int lastEncoded;
	std::chrono::time_point<std::chrono::high_resolution_clock> _tick;
};
void objCallback();
class PiBot 
{
public:
	PiBot(bool paralellMode = false);
	~PiBot();
	int SetSpeed(unsigned char motor, unsigned char dir, unsigned char speed);
	int SetWheelSpeed(unsigned char wheel, int16_t rpm);
	int SetCurrentLimit(unsigned char value);
	static void PowerControl(uint8_t onOff);
	//int32_t DriveSteps(uint8_t coil1, uint8_t coil2, int32_t steps, uint32_t periodUs, uint8_t torque = 255);
	//int32_t DriveHalfSteps(uint8_t coil1, uint8_t coil2, int32_t steps, uint32_t periodUs, uint8_t torque = 255);
	int SetPWM(uint8_t channel, uint16_t level);
	float GetRangeCm(int triggerPin, int echoPin, float velocity = 340.0);
	void SonarTrigger(int triggerPin);
	static void UpdateEncoders();
	//deacayMode_t decayMode;
	StepperDriver *stepper[4][4];
	//Encoder* encoders;
	 std::vector<Encoder*> encoders;
	//void CreateEncoders();
	float dist1, dist2, dist3;
private:
	static void _EchoIsrCb1(PiBot *bot);
	static void _EchoIsrCb2(PiBot *bot);
	static void _EchoIsrCb3(PiBot *bot);
	std::chrono::time_point<std::chrono::high_resolution_clock> _triggerTime1, _triggerTime2, _triggerTime3;
	int pca9634Fd;
	int _pca9685Fd;
	MotorDriver _driver;
	/*int32_t _step[4][4];
	struct timeval _stepTime[4][4];
	struct timeval refTime[4][4];
	uint32_t prevPeriod[4][4];
	int32_t _nextT[4][4];*/
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

#endif // PIBOT_H
