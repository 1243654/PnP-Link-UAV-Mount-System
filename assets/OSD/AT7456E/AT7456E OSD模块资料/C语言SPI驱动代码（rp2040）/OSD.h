/*
本程序只供学习使用，未经作者许可，不得用于任何商业用途或变相获利！
注：尊重知识产权，禁止在互联网上传播、发布本程序内容。一经发现 ，追究责任。
    关注公众号、B站“易乎电子科技”，获取更多技术支持。
出品：易乎电子科技(iF,Make Technology Easier)
作者：小白
 * Copyright (c) 2030 iF Technology Ltd.
 */
#ifndef _OSD_H
#define _OSD_H

#include "pico/stdlib.h"

/*AT7456底层驱动*/
void spi0_init(void);//便于外部单独调用

uint8_t at7456_init(void);//at7456硬件配置初始化

void at7456_clearSRAM(void);//通过操作寄存器，清除显存
void at7456_writeSRAM(uint8_t row,uint8_t columns,uint8_t addr);//以单字节的方式写入SRAM，但不显示，需要调用at7456_OSD_on()

void at7456_OSD_on(void);
void at7456_OSD_off(void);


/*OSD显示相关*/
//void OSD_displayChar(unsigned char col, unsigned char row, unsigned char c);//显示单个字符

void OSD_init(void);//屏幕显示框架的初始化

void OSD_displyInt(uint8_t row,uint8_t columns,uint8_t number_len,long number);
void OSD_displyInt_1(uint8_t row,uint8_t columns,uint8_t number_len,long number);
void OSD_disply_Float(uint8_t row,uint8_t columns,uint8_t int_n,uint8_t flaot_n,float number);

void OSD_update(uint8_t mode, float batV, bool RSSI, uint8_t throt, float pitch, float roll, float coreTemp, bool lock);
/******寄存器地址定义*******/
// at7456
#define	VM0					0x00		// Video Mode0
#define	VM1					0x01		// Video Mode1
#define	HOS					0x02		// Horizontal Offset
#define	VOS					0x03		// Vertical Offset
#define	DMM					0x04		// Display Memory Mode
#define	DMAH				0x05		// Display Memory Address High
#define	DMAL				0x06		// Display Memory Address Low
#define	DMDI				0x07		// Display Memory Data In
#define	CMM					0x08		// Character Memory Mode
#define	CMAH				0x09		// Character Memory Address High
#define	CMAL				0x0a		// Character Memory Address Low
#define	CMDI				0x0b		// Character Memory Data In
#define	OSDM				0x0c		// OSD Insertion Mux
#define	RB0					0x10		// Row 0 Brightness
#define	RB1					0x11		// Row 1 Brightness
#define	RB2					0x12		// Row 2 Brightness
#define	RB3					0x13		// Row 3 Brightness
#define	RB4					0x14		// Row 4 Brightness
#define	RB5					0x15		// Row 5 Brightness
#define	RB6					0x16		// Row 6 Brightness
#define	RB7					0x17		// Row 7 Brightness
#define	RB8					0x18		// Row 8 Brightness
#define	RB9					0x19		// Row 9 Brightness
#define	RB10				0x1a		// Row 10 Brightness
#define	RB11				0x1b		// Row 11 Brightness
#define	RB12				0x1c		// Row 12 Brightness
#define	RB13				0x1d		// Row 13 Brightness
#define	RB14				0x1e		// Row 14 Brightness
#define	RB15				0x1f		// Row 15 Brightness
#define	OSDBL				0x6c		// OSD Black Level
#define	STAT				0x20		// Status
#define	DMDO				0x30		// Display Memory Data Out
#define	CMDO				0x40		// Character Memory Data Out

#define NVM_RAM				0x50		// 将NVM中的字库读取到镜像RAM中
#define RAM_NVM				0xa0		// 将镜像RAM中的字库数据写到NVM中

// VM0
#define NTSC				(0 << 6)	// D6 --- Vidoe Standard Select
#define PAL				    (1 << 6)
#define SYNC_AUTO			(0 << 4)	// D5,D4 --- Sync Select Mode
#define SYNC_EXTERNAL		(2 << 4)
#define SYNC_INTERNAL		(3 << 4)
#define OSD_ENABLE			(1 << 3)	// D3 --- Enable Display of OSD image
#define OSD_DISABLE			(0 << 3)
#define SOFT_RESET			(1 << 1)	// D1 --- Software Reset
#define VOUT_ENABLE			(0 << 0)
#define VOUT_DISABLE		(1 << 0)

// VM1
#define BACKGND_0			(0 << 4)	// 背景电平 WHT%
#define BACKGND_7			(1 << 4)
#define BACKGND_14			(2 << 4)
#define BACKGND_21			(3 << 4)
#define BACKGND_28			(4 << 4)
#define BACKGND_35			(5 << 4)
#define BACKGND_42			(6 << 4)
#define BACKGND_49			(7 << 4)

#define BLINK_TIME40		(0 << 2)	// 闪烁周期, ms
#define BLINK_TIME80		(1 << 2)
#define BLINK_TIME120		(2 << 2)
#define BLINK_TIME160		(3 << 2)
// 闪烁占空比(ON : OFF)
#define BLINK_DUTY_1_1		0			// BT : BT
#define BLINK_DUTY_1_2		1			// BT : 2BT
#define BLINK_DUTY_1_3		2			// BT : 3BT
#define BLINK_DUTY_3_1		3			// 3BT : BT

// DMM
#define SPI_BIT16			(0 << 6)	// 写字符时采用16bit方式，字符属性来自DMM[5:3]
#define SPI_BIT8			(1 << 6)	// 写字符时用8bit方式，字符属性单独写入
#define CHAR_LBC			(1 << 5)	// 本地背景
#define CHAR_BLK			(1 << 4)	// 闪烁显示
#define CHAR_INV			(1 << 3)	// 负像显示
#define CLEAR_SRAM			(1 << 2)	// 清屏，20us
#define VETICAL_SYNC		(1 << 1)	// 命令在场同步后起作用
#define AUTO_INC			(1 << 0)	// 字符地址自动递增

// RBi
#define BLACK_LEVEL_0		(0 << 2)	// 0% 白电平
#define BLACK_LEVEL_10		(1 << 2)	// 10% 白电平
#define BLACK_LEVEL_20		(2 << 2)	// 20% 白电平
#define BLACK_LEVEL_30		(3 << 2)	// 30% 白电平
#define WHITE_LEVEL_120		(0 << 0)	// 120% 白电平
#define WHITE_LEVEL_100		(1 << 0)	// 110% 白电平
#define WHITE_LEVEL_90		(2 << 0)	// 90% 白电平
#define WHITE_LEVEL_80		(3 << 0)	// 80% 白电平

// STAT
#define PAL_DETECT			(1 << 0)	// 检测到PAL信号
#define NTSC_DETECT			(1 << 1)	// 检测到NTSC信号
#define LOS_DETECT			(1 << 2)	// 检测到LOS信号
#define VSYNC				(1 << 4)	// 场同步

#endif
