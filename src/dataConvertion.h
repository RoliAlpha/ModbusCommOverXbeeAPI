#pragma once

#include <Arduino.h>
#ifndef _dataConvertion
#define _dataConvertion
#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdio.h>
/*
1. float和unsigned long具有相同的数据结构长度
2. union据类型里的数据存放在相同的物理空间
*/
typedef union
{
	float fdata;
	unsigned long ldata;
}FloatLongType;
/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float f, unsigned char byte[]);
/*
将4个字节数据byte[4]转化为浮点数存放在*f中
*/
void Byte_to_Float(float *f, unsigned char byte[]);
/*
将unsigned int转化为2个字节数据存放在byte[2]中
*/
void Uint_to_Byte(unsigned int  i, unsigned char byte[]);
/*
将2个字节数据byte[2]转化为unsigned int存放在*i中
*/
void Byte_to_Uint(unsigned int * i, unsigned char byte[]);
#endif