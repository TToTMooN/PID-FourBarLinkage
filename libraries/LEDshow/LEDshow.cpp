/*
LED.h - Library for flashing LED.
Created by Yukai,Lin, April 24, 2015.
Released into the public domain.
*/

#include <string.h>
#include "LEDshow.h""
#include "stdlib.h"
#include "stdio.h"
unsigned char CHAR_MAP[] = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x8C, 0xBF, 0xC6, 0xA1, 0x86, 0xFF, 0xbf };
char CHAR_INDEX_MAP[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', '-' };

LED::LED(int pinD, int pinR, int pinS)
{
	pinMode(pinD, OUTPUT);
	pinMode(pinR, OUTPUT);
	pinMode(pinS, OUTPUT);
	DIO = pinD;
	RCLK = pinR;
	SCLK = pinS;

}

void LED::ledout(unsigned char chr)
{
	unsigned char i;
	for (i = 8; i >= 1; i--)
	{
		if (chr & 0x80) digitalWrite(DIO, 1); else digitalWrite(DIO, 0);
		chr <<= 1;
		digitalWrite(SCLK, 0);
		digitalWrite(SCLK, 1);
	}
}


void LED::ledshow( int num){
    char str[8];
    itoa(num,str,10);
	int len = strlen(str);
	int hasDot = 0;
	for (int i = len - 1, m = 0; i >= 0; i--) {
		char chr = str[i];
		if (chr == '.') {
			hasDot = 1;
			continue;
		}
		int n = 0;
		for (; n < 17; n++) {
			if (chr == CHAR_INDEX_MAP[n])
				break;
		}
		if (n != 17) {
			unsigned char chr1 = CHAR_MAP[n];
			if (hasDot)
				chr1 &= 0x7f; // 所有字符的最高位(左数第一位)都是1,代表小数点为暗
			// 因此只要将最高位改为0小数点就可以点亮啦
			ledout(chr1); // 显示字符
			ledout((int)ceil(pow(2, m))); // 位置
			digitalWrite(RCLK, 0);
			digitalWrite(RCLK, 1);
		}

		m++;
		hasDot = 0;
	}
}

