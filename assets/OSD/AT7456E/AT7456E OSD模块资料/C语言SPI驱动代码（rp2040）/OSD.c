/**
本程序只供学习使用，未经作者许可，不得用于任何商业用途或变相获利！
注：尊重知识产权，禁止在互联网上传播、发布本程序内容。一经发现 ，追究责任。
    关注公众号、B站“易乎电子科技”，获取更多技术支持。
出品：易乎电子科技(iF,Make Technology Easier)
作者：小白
Copyright (c) 2030 iF Technology Ltd.
file name:OSD.c
author   :xiaobai
date     :20240701
version  :V0.01 micro
ps       :basic OSD function
	20240705:在显示成功的基础上，重构函数框架

 */
#include "OSD.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
//#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "config_param.h"


/******SPI硬件端口定义*******/
#define OSD_SPI_PORT spi0
#define READ_BIT 0x80

#define SPI0_PIN_MISO 4 //MISO Pin
#define SPI0_PIN_MOSI 3 //MOSI Pin
#define SPI0_PIN_SCK 2 //Pin
#define SPI0_PIN_CS 5 //Pin

//全局变量
bool OSD_WARNING_CLEAR = 1; //默认屏幕警告信息已清除
bool OSD_MSG_CLEAR = 0; //默认屏幕消息信息未清除
bool OSD_FLYMODE_SWITCHED = 1; //默认OSD显示的飞行模式有变化，否则不刷新，显示XXXX
uint8_t lastFlyMode = 1; //全局变量，用于存储OSD_Update中上次的飞行模式

void spi0_init(void){
    //外设及IO初始化
    // Enable SPI 1 at 1 MHz and connect to GPIOs
    //spi_init(OSD_SPI_PORT, 1000 * 1000);
    spi_init(OSD_SPI_PORT, 4000 * 1000);//500 * 1000);
    gpio_set_function(SPI0_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    //bi_decl(bi_3pins_with_func(SPI0_PIN_MISO, SPI0_PIN_MOSI, SPI0_PIN_SCK, GPIO_FUNC_SPI));

    //CS Pin的初始化
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(SPI0_PIN_CS);
    gpio_set_dir(SPI0_PIN_CS, GPIO_OUT);
    gpio_put(SPI0_PIN_CS, 1);
    // Make the CS pin available to picotool
    //bi_decl(bi_1pin_with_name(SPI0_PIN_CS, "SPI CS"));
   
}

static inline void CS0() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI0_PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void CS1() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPI0_PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

//向指定寄存器写8位数据
void at7456_write_addr_data(uint8_t addr, uint8_t dat){
    // Two bytes. First byte register, second byte data
    uint8_t buf2[2];
    //buf2[0] = addr;
    buf2[0] = addr & 0x7f;// remove read bit as this is a write
    buf2[1] = dat;

    CS0();  // Active low
    spi_write_blocking(OSD_SPI_PORT, buf2, 2); //写入数据
    CS1();

    //return buf2[0]; //根据NRF手册，SPI写操作，第1字节返回的是状态寄存器位

}

//从指定寄存器读取8位数据
uint8_t at7456_read_addr_data(uint8_t addr){
    uint8_t buf[1];//数据数组变量

    addr |= READ_BIT;
    CS0();  // Active low
    spi_write_blocking(OSD_SPI_PORT, &addr, 1); //发送寄存器号
    sleep_us(5);//10us,必须延迟一段时间
    spi_read_blocking(OSD_SPI_PORT, 0, buf, 1);
    CS1();

    return buf[0]; //返回读取的数据
}

// 需要在SPI初始化后，才能调用
// 通过写入/读出VM0.7来判断AT7456的版本问题，新版可以读写VM0.7位
// 防止SPI接口开路或短路，需要有0、1两种状态
uint8_t at7456_check(void){
	uint8_t r1, r2;

	r1 = at7456_read_addr_data(VM0);
	r2 = (r1 & ~(1 << 1)) | 0x88; 		// VM0.1(Software Reset Bit) = 0，同时将VM0.3置位1
	at7456_write_addr_data(VM0, r2);	// 写VM0
	sleep_us(20);
	r2 = at7456_read_addr_data(VM0) & 0x88;
	at7456_write_addr_data(VM0, r1);	// 恢复VM0
	sleep_us(20);

	if (r2 == 0x88)
	{
		return 0;//NEW7456;					// 新版7456
	}
	else if (r2 == 0x08)
	{
		return 1;//OLD7456;					// 老版7456
	}
	else
	{
		return 2;//BAD7456;					// SPI接口异常
	}
}

//AT7456初始化
uint8_t at7456_init(void){
	uint8_t k;

	spi0_init();
	//sleep_ms(50); //上电复位后要等待50ms

	uint8_t AT7456_Version = at7456_check();
    if(AT7456_Version == 0)  printf("AT7456 is new version.\n");
    else if(AT7456_Version == 1) printf("AT7456 is old version.\n");
    else if(AT7456_Version == 2) {
		printf("AT7456 error,exit at7456_init()! \n");
		return 1;
	}
	//设置 VM0 视频模式寄存器
	//bit6:视频模式选择	0 = NTSC	1 = PAL
	//内部同步分离、PAL制式、自动sync、OSD不使能、软件复位
	at7456_write_addr_data(VM0, 0x42);//0100 0010
	sleep_us(40);//软件复位延时
	//sleep_ms(1);
	k = at7456_read_addr_data(VM0);
	at7456_write_addr_data(VM0, k|PAL);//软件复位后必须重新设置视频制式，上边第一行其实不管用。。。

	//通过VM1 设置亮度
	k = at7456_read_addr_data(VM1);
	at7456_write_addr_data(VM1, (k&0x8F)|BACKGND_0);//bit654 先清零 0b1000 1111 再或 

	//DMM 显示存储器 模式寄存器
	//bit6 默认0 16位操作模式
	//写入DMM[6]＝1，选择 8 位工作模式。
	//at7456_write_addr_data(DMM, 0x40);//0100 0000

	//设置DMM[5]LBC为1，适用于16位模式下的通过VM1调节字符亮度的功能
	//不行，这样会导致字符有黑底，透明模式，必须为0
	//k = at7456_read_addr_data(DMM);
	//at7456_write_addr_data(DMM, k|(1 << 5));	

	//OSDBL黑电平寄存器
	//写入 OSDBL[4]＝0，使能自动 OSD 黑电平控制。
	k = at7456_read_addr_data(OSDBL);
	at7456_write_addr_data(OSDBL, k & ~(1 << 4));		

	at7456_clearSRAM();

	printf("at7456_init Ok!\n");
	return 0;
}

//清除缓存内容
void at7456_clearSRAM(void){
   uint8_t k=0;
   //DMM 显示存储器模式寄存器
   k = at7456_read_addr_data(DMM);
   at7456_write_addr_data(DMM,k|(1 << 2));  //0000 0100,bit[2]置1,fill all display memories with zeros
   sleep_us(40); // 清除缓存后至少为40us
}

//将需要显示的字符写入显示内存中,要显示需要再调用at7456_OSD_on()
/*
row:行数 NTSC=0~12,PAL=0~15;
columns:列数 =0~29
word：字符地址0-255(看手册图12,0(0x00)对应无字符；1(0x01)对应1 )
*/
//16位模式下只能访问page1的256个字符
void at7456_writeSRAM(uint8_t row,uint8_t columns,uint8_t addr){ 
	uint16_t kk;

	kk = row * 30 + columns;
	at7456_write_addr_data(DMAH, kk / 256);    		// address
	at7456_write_addr_data(DMAL, kk % 256);
	at7456_write_addr_data(DMDI, addr);
}

// 打开OSD
void at7456_OSD_on(void){
	uint8_t k;

	k = at7456_read_addr_data(VM0);
	at7456_write_addr_data(VM0, k | (1 << 3));		// VM0[3]=1，打开OSD字符显示 Enable Display of OSD Image
	sleep_us(10);//打开字符显示后延时10us
}
// 关闭OSD
void at7456_OSD_off(void){
	uint8_t k;

	k = at7456_read_addr_data(VM0);
	at7456_write_addr_data(VM0, k & ~(1 << 3));		// VM0[3]=0，禁止OSD
	sleep_us(30);//关闭字符显示后延时30us
}


//屏幕显示框架的初始化
void OSD_init(void){

	//at7456_OSD_off();
	//at7456_writeSRAM(13,5,0x01);//H
	//at7456_writeSRAM(0,0,0x12);//H
	//at7456_writeSRAM(0,1,0x19);//O
	//at7456_writeSRAM(0,2,0x1C);//R
	at7456_writeSRAM(0,0,0x3C);//x
	at7456_writeSRAM(0,1,0x3C);//x
	at7456_writeSRAM(0,2,0x3C);//x
	at7456_writeSRAM(0,3,0x3C);//x

	at7456_writeSRAM(0,6,0x26);//b
	at7456_writeSRAM(0,7,0x25);//a
	at7456_writeSRAM(0,8,0x38);//t
	at7456_writeSRAM(0,9,0x44);//：
	at7456_writeSRAM(0,15,0x3A);//v

	at7456_writeSRAM(0,21,0x1C);//R
	at7456_writeSRAM(0,22,0x1D);//S
	at7456_writeSRAM(0,23,0x1D);//S
	at7456_writeSRAM(0,24,0x13);//I

	//at7456_writeSRAM(0,26,0x4B);//>

	at7456_writeSRAM(0,27,0x49);//-
	at7456_writeSRAM(0,28,0x06);//6
	at7456_writeSRAM(0,29,0x04);//4	

	//中间十字星
	//at7456_writeSRAM(7,12,0x49);//-
	//at7456_writeSRAM(7,13,0x49);//-
	at7456_writeSRAM(7,13,0x41);//·
	//at7456_writeSRAM(7,14,0x33);//o
	//at7456_writeSRAM(7,14,0x44);//:
	//at7456_writeSRAM(7,15,0x49);//-
	at7456_writeSRAM(7,15,0x41);//·
	//at7456_writeSRAM(7,16,0x49);//-
	//at7456_writeSRAM(6,14,0x30);//|
	at7456_writeSRAM(6,14,0x41);//·
	//at7456_writeSRAM(8,14,0x30);//|
	at7456_writeSRAM(8,14,0x41);//·

	//左边框
	at7456_writeSRAM(4,6,0x41);//·
	at7456_writeSRAM(5,6,0x41);//·
	//at7456_writeSRAM(5,6,0x49);//-
	at7456_writeSRAM(6,6,0x41);//·
	at7456_writeSRAM(7,6,0x49);//-
	at7456_writeSRAM(8,6,0x41);//·
	//at7456_writeSRAM(9,6,0x49);//-
	at7456_writeSRAM(9,6,0x41);//·
	at7456_writeSRAM(10,6,0x41);//·
	at7456_writeSRAM(7,7,0x4B);//>

	//右边框
	at7456_writeSRAM(4,22,0x41);//·
	at7456_writeSRAM(5,22,0x41);//·
	//at7456_writeSRAM(5,22,0x49);//-
	at7456_writeSRAM(6,22,0x41);//·
	at7456_writeSRAM(7,22,0x49);//-
	at7456_writeSRAM(8,22,0x41);//·
	//at7456_writeSRAM(9,22,0x49);//-
	at7456_writeSRAM(9,22,0x41);//·
	at7456_writeSRAM(10,22,0x41);//·
	at7456_writeSRAM(7,21,0x4A);//<

	at7456_writeSRAM(13,0,0x1E);//T
	at7456_writeSRAM(14,0,0x1A);//P
	at7456_writeSRAM(15,0,0x1C);//R

	at7456_writeSRAM(15,11,0x1E);//T	
	at7456_writeSRAM(15,12,0x44);//：
	at7456_writeSRAM(15,17,0x0D);//C

	//20240825
	at7456_writeSRAM(4,11,0x0E);//D
	at7456_writeSRAM(4,12,0x13);//I
	at7456_writeSRAM(4,13,0x1D);//S
	at7456_writeSRAM(4,14,0x0B);//A
	at7456_writeSRAM(4,15,0x1C);//R
	at7456_writeSRAM(4,16,0x17);//M
	at7456_writeSRAM(4,17,0x0F);//E
	at7456_writeSRAM(4,18,0x0E);//D

	at7456_OSD_on();
	//at7456_write_addr_data(VM0, 0x48);
}

/*****显示实数，有0占位**********************/
/***参数：row：        显示行数0~15**********************/
/***参数：columns：    显示起始列数0~29******************/
/***参数：number_len： 整数占位数，负号占1位*************/
/***参数：number：     需要显示的实数********************/
void OSD_displyInt(uint8_t row,uint8_t columns,uint8_t number_len,long number)
{
	uint8_t k,m,h,i;
	uint8_t Knumber[8]={0};//最大显示10位数据
	k=0;
	h=0;
	if(number==0)//数据=0
	{//每个位上都显示0
		for(i=0;i<number_len;i++)
		{
			m=columns+i;
 			at7456_writeSRAM(row,m,0x0A);//显示0
		}
	}
	else if(number>0)//数据>0
	{
		while(number>0)//分离各位上的数字
		{
			Knumber[k]=number%10;
			number=number/10;
			k=k+1;
		}
		if(k>number_len)	k=number_len;
		h=k;
		for(i=0;i<number_len;i++ )//显示数字
		{
			m=number_len+columns-i-1;
			if(Knumber[i]==0)				
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
	}
	else if(number<0)//数据<0
	{
		long m_Abs_vaule;
		m_Abs_vaule=fabs(number);
		while(m_Abs_vaule>0)
		{
			Knumber[k]=m_Abs_vaule%10;
			m_Abs_vaule=m_Abs_vaule/10;
			k=k+1;
		}  
		//显示负号
		m=columns;
		at7456_writeSRAM(row,m,0x49);//显示”-“

		for(i=0;i<(number_len-1);i++ )
		{
			m=number_len+columns-i-1;
			if(Knumber[i]==0)					 
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
	}
}
/*****显示实数，无0占位**********************/
/***参数：row：        显示行数0~15**********************/
/***参数：columns：    显示起始列数0~29******************/
/***参数：number_len： 整数占位数，负号占1位*************/
/***参数：number：     需要显示的实数********************/
void OSD_displyInt_1(uint8_t row,uint8_t columns,uint8_t number_len,long number)
{
	uint8_t k,m,i;
	uint8_t Knumber[8]={0};//最大显示10位数据
	k=0;
	if(number==0)//数据=0
	{		
		m=columns+number_len-1;
		at7456_writeSRAM(row,m,0x0A);////最后一位上显示0
		for(i=0;i<(number_len-1);i++)
		{
			m=columns+i;
			at7456_writeSRAM(row,m,0x00);////清除空位
		}
	}
	else if(number>0)//数据>0
	{
		while(number>0)//分离各位上的数字
		{
			Knumber[k]=number%10;
			number=number/10;
			k=k+1;
		}
		if(k>number_len) k=number_len;
		for(i=0;i<k;i++ )//显示数字
		{
			m=number_len+columns-1-i;
			if(Knumber[i]==0)				
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
		if(k<number_len)
		{
			for(i=0;i<number_len-k;i++)
			{
				m=columns+i;
				at7456_writeSRAM(row,m,0x00);//清除空位
			}
		}
		 
	}
	else if(number<0)//数据<0
	{
		long m_Abs_vaule;
		m_Abs_vaule=fabs(number);
		while(m_Abs_vaule>0)
		{
			Knumber[k]=m_Abs_vaule%10;
			m_Abs_vaule=m_Abs_vaule/10;
			k=k+1;
		} 
		if(k>(number_len-1)) k=number_len-1;
		//显示数字
		for(i=0;i<k;i++ )
		{
			m=number_len+columns-i-1;
			if(Knumber[i]==0)					 
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
		if(k<(number_len-1))
		{
			for(i=0;i<(number_len-k-1);i++)
			{
				m=columns+i+1;
				at7456_writeSRAM(row,m,0x00);//清除空位
			}
		}
		//显示负号
		at7456_writeSRAM(row,columns,0x49);//显示”-“
   }
}
/*****显示浮点数,无0占位**********************/
/***参数：row：    显示行数0~15********************/
/***参数：columns：显示起始列数0~290~29************/
/***参数：int_n：  整数占位数，负号占1位***********/
/***参数：flaot_n：小数点后几位********************/
/***参数：number： 显示的数字**********************/
void OSD_disply_Float(uint8_t row,uint8_t columns,uint8_t int_n,uint8_t flaot_n,float number)
{
	uint8_t i,m;
	uint8_t float_Knumber;//小数最大5位;	
	if(number==0)//数据=0
	{
		m=columns+int_n+flaot_n;
		at7456_writeSRAM(row,m,0x0A);//显示0
	}
	else
	{
		long int_data;
		float m_float_data;
		int_data=number;//分离整数
		//分离小数
		if(number>0)//数据>0
			m_float_data=number-int_data;//分离小数
		else
		{
			float f_number;
			int   I_number;
			I_number=0-number;
			f_number=0-number;		
			m_float_data=f_number-I_number;//分离小数
		}
		//显示小数
		for(i=0;i<flaot_n;i++)
		{
			m_float_data=m_float_data*10;
			float_Knumber=m_float_data;
			m=columns+int_n+i+1;
			if(float_Knumber==0)
				at7456_writeSRAM(row,m,0x0A);
			else
				at7456_writeSRAM(row,m,float_Knumber);
			m_float_data=m_float_data-float_Knumber;
		}
		//处理小数部分显示
		m=columns+int_n;
		at7456_writeSRAM(row,m,0x41);//显示小数点
		//显示整数部分		
		OSD_displyInt_1(row,columns,int_n,int_data);	
		if(number<0 && number >-1)
			at7456_writeSRAM(row,columns,0x49);//显示”-“
	 }

}
//内部函数，显示低电压报警信息
void dispWarning_batV(void){
	at7456_writeSRAM(10,10,0x16);//L
	at7456_writeSRAM(10,11,0x33);//o
	at7456_writeSRAM(10,12,0x3B);//w

	at7456_writeSRAM(10,14,0x26);//b
	at7456_writeSRAM(10,15,0x25);//a
	at7456_writeSRAM(10,16,0x38);//t
	at7456_writeSRAM(10,17,0x20);//V

}
//内部函数，显示低信号报警信息
void dispWarning_RSSI(void){
	at7456_writeSRAM(10,10,0x16);//L
	at7456_writeSRAM(10,11,0x33);//o
	at7456_writeSRAM(10,12,0x3B);//w

	at7456_writeSRAM(10,14,0x1C);//R
	at7456_writeSRAM(10,15,0x1D);//S
	at7456_writeSRAM(10,16,0x1D);//S
	at7456_writeSRAM(10,17,0x13);//I
}
//内部函数，清除报警信息
void dispWarning_clear(void){
	at7456_writeSRAM(10,10,0x00);//
	at7456_writeSRAM(10,11,0x00);//
	at7456_writeSRAM(10,12,0x00);//

	at7456_writeSRAM(10,14,0x00);//
	at7456_writeSRAM(10,15,0x00);//
	at7456_writeSRAM(10,16,0x00);//
	at7456_writeSRAM(10,17,0x00);//
}

void dispMsg_disarm(uint8_t mode){
	if(mode == 1){
		at7456_writeSRAM(4,11,0x0E);//D
		at7456_writeSRAM(4,12,0x13);//I
		at7456_writeSRAM(4,13,0x1D);//S
		at7456_writeSRAM(4,14,0x0B);//A
		at7456_writeSRAM(4,15,0x1C);//R
		at7456_writeSRAM(4,16,0x17);//M
		at7456_writeSRAM(4,17,0x0F);//E
		at7456_writeSRAM(4,18,0x0E);//D
	}
	else{
		at7456_writeSRAM(4,11,0x00);//D
		at7456_writeSRAM(4,12,0x00);//I
		at7456_writeSRAM(4,13,0x00);//S
		at7456_writeSRAM(4,14,0x00);//A
		at7456_writeSRAM(4,15,0x00);//R
		at7456_writeSRAM(4,16,0x00);//M
		at7456_writeSRAM(4,17,0x00);//E
		at7456_writeSRAM(4,18,0x00);//D
	}
}
//OSD显示更新函数，在50hz循环中调用
void OSD_update(uint8_t mode, float batV, bool RSSI, uint8_t throt, float pitch, float roll, float coreTemp, bool lock){
	//1)模式显示，1为HOR自稳模式；2为ARCO手动模式
	if(mode != lastFlyMode) OSD_FLYMODE_SWITCHED = 1;
	lastFlyMode = mode; //更新上次模式
	if(OSD_FLYMODE_SWITCHED == 1){
		if(mode == 1) {
			//后续可以改为Angle，暂时先不动
			//at7456_writeSRAM(0,0,0x12);//H
			//at7456_writeSRAM(0,1,0x19);//O
			//at7456_writeSRAM(0,2,0x1C);//R	
			//at7456_writeSRAM(0,3,0x00);//第4位清除显示	
			at7456_writeSRAM(0,0,0x12);//H
			at7456_writeSRAM(0,1,0x19);//O
			at7456_writeSRAM(0,2,0x1C);//R	
			at7456_writeSRAM(0,3,0x00);//第4位清除显示
		}
		else if(mode == 2){
			at7456_writeSRAM(0,0,0x0B);//A
			at7456_writeSRAM(0,1,0x0D);//C
			at7456_writeSRAM(0,2,0x1C);//R
			at7456_writeSRAM(0,3,0x19);//O	
		}
		else{
			at7456_writeSRAM(0,0,0x42);//?
			at7456_writeSRAM(0,1,0x42);//?
			at7456_writeSRAM(0,2,0x42);//?
			at7456_writeSRAM(0,3,0x42);//?	
		}
		OSD_FLYMODE_SWITCHED = 0;
	}
	//2)电池电压显示
	OSD_disply_Float(0,10,2,2,batV);//显示电池电压
	//3)信号强度显示,1强，0弱
	if(RSSI == 1){
	    at7456_writeSRAM(0,26,0x4B);//显示信号阈值 > 
	}
	else if(RSSI == 0){
		at7456_writeSRAM(0,26,0x4A);//显示信号阈值 < 
	}
	//低电压、低信号强度告警显示：会存在当同时告警，但只显示RSSI的情况，但不影响，将RSSI第一个判断，提高其优先级
	if((RSSI == 0)&&(OSD_WARNING_CLEAR)){
		dispWarning_RSSI();
		OSD_WARNING_CLEAR = 0;//未清屏置位
	}
	else if((batV < Bat_alarm)&&(OSD_WARNING_CLEAR)){
		dispWarning_batV();
		OSD_WARNING_CLEAR = 0;//未清屏置位
	}
	else if(!OSD_WARNING_CLEAR){
		dispWarning_clear();
		OSD_WARNING_CLEAR = 1;//已清屏置位
	}
	//4)油门及姿态显示
	OSD_displyInt_1(13,1,4,throt);//显示油门值T
    OSD_disply_Float(14,1,4,1,pitch);//显示俯仰值P
    OSD_disply_Float(15,1,4,1,roll);//显示俯仰值R
	//5)CPU核心温度显示
    OSD_disply_Float(15,13,2,1,coreTemp);//显示核心温度
	//6)锁定状态显示，加标志位判断的目的是避免不需要清屏的时候，重复写入
	if(lock == 1){
		dispMsg_disarm(1);
		OSD_MSG_CLEAR = 0; //屏幕消息未清除
	}
	else{
		if(OSD_MSG_CLEAR == 0){
			dispMsg_disarm(0); //
			OSD_MSG_CLEAR = 1; //屏幕消息已清除
		}
	}

    at7456_OSD_on();
}



/*
    //OSD_displyInt(13,1,5,255);
    //OSD_displyInt_1(14,1,5,255);
    //OSD_displyInt(15,1,5,-255);
*/
