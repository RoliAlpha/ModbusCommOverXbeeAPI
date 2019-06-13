#include"dataConvertion.h"
/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float f, unsigned char byte[])
{
	FloatLongType fl;
	fl.fdata = f;
	byte[0] = (unsigned char)fl.ldata;
	byte[1] = (unsigned char)(fl.ldata >> 8);
	byte[2] = (unsigned char)(fl.ldata >> 16);
	byte[3] = (unsigned char)(fl.ldata >> 24);
}
/*
将4个字节数据byte[4]转化为浮点数存放在*f中
*/
void Byte_to_Float(float *f, unsigned char byte[])
{
	FloatLongType fl;
	fl.ldata = 0;
	fl.ldata = byte[3];
	fl.ldata = (fl.ldata << 8) | byte[2];
	fl.ldata = (fl.ldata << 8) | byte[1];
	fl.ldata = (fl.ldata << 8) | byte[0];
	*f = fl.fdata;
}
/*
将unsigned int转化为2个字节数据存放在byte[2]中
*/
void Uint_to_Byte(unsigned int  i, unsigned char byte[])
{
	byte[0] = (unsigned char)i;
	byte[1] = (unsigned char)(i >> 8);
}
/*
将2个字节数据byte[2]转化为unsigned int存放在*i中
*/
void Byte_to_Uint(unsigned int * i, unsigned char byte[])
{
	unsigned int i1 = 0;
	i1 = byte[1];
	i1 = (i1 << 8) | byte[0];
	*i = i1;
}