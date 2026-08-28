#include	"Base.h"

//获得字节第nbit=1或0
/****n=0~7；
*////////////
uint8_t  get_byte_n(uint8_t  byte,uint8_t  n)
{
	uint8_t  B;
	switch(n)
	{
		case 0:
			B=byte & 0x01;
		break;
		case 1:
			B=byte & 0x02;
		break;
		case 2:
			B=byte & 0x04;
		break;
		case 3:
			B=byte & 0x08;
		break;
		case 4:
			B=byte & 0x10;
		break;
		case 5:
			B=byte & 0x20;
		break;
		case 6:
			B=byte & 0x40;
		break;
		case 7:
			B=byte & 0x80;
		break;	
	}
    return B;		
}
//置位字节第nbit=1
/****n=0~7；
*////////////
uint8_t  SET_byte_n_H(uint8_t  Byte,uint8_t  n)
{
	uint8_t  byte;
	byte=Byte;
	switch(n)
	{
				case 0:
		byte=byte | 0x01;//00000001
		break;
		case 1:
			byte=byte | 0x02;//00000010
		break;
		case 2:
			byte=byte | 0x04;//00000100
		break;
		case 3:
			byte=byte | 0x08;//00001000
		break;
		case 4:
			byte=byte | 0x10;//00010000
		break;
		case 5:
			byte=byte | 0x20;//00100000
		break;
		case 6:
			byte=byte | 0x40;//01000000
		break;
		case 7:
			byte=byte | 0x80;//10000000
		break;
	}
	return byte;
}
//置位字节第nbit=0
/****n=0~7；
*////////////
uint8_t  SET_byte_n_L(uint8_t  Byte,uint8_t n)
{
	uint8_t  byte;
	byte=Byte;
	switch(n)
	{
		case 0:
			byte=byte & 0xFE;//11111110
		break;
		case 1:
			byte=byte & 0xFD;//11111101
		break;
		case 2:
			byte=byte & 0xFB;//11111011
		break;
		case 3:
			byte=byte & 0xF7;//11110111
		break;
		case 4:
			byte=byte & 0xEF;//11101111
		break;
		case 5:
			byte=byte & 0xDF;//11011111
		break;
		case 6:
			byte=byte & 0xBF;//10111111
		break;
		case 7:
			byte=byte & 0x7F;//01111111
		break;	

	}	
   return byte;	
}

