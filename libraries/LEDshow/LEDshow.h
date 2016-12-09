/*
	LED.h - Library for flashing LED.
Created by Yukai,Lin, April 24, 2015.
Released into the public domain.
*/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef LEDshow_h
#define LEDshow_h
//#include "WProgram.h"
class LED
{
public:
	LED(int pinD,int pinR, int pinS);
	void ledout(unsigned char chr);
	void ledshow(int num);
private:
	int DIO, RCLK, SCLK;
};
#endif
