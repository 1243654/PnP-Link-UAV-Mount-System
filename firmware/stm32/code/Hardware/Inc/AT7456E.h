#ifndef __AT7456E_H
#define __AT7456E_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 寄存器地址 */
#define AT7546E_VM0   0X00
#define AT7546E_VM1   0X01
#define AT7546E_HOS   0X02
#define AT7546E_VOS   0X03
#define AT7546E_DMM   0X04
#define AT7546E_DMAH  0X05
#define AT7546E_DMAL  0X06
#define AT7546E_DMDI  0X07
#define AT7546E_CMM   0X08
#define AT7546E_CMAH  0X09
#define AT7546E_CMAL  0X0A
#define AT7546E_CMDI  0X0B
#define AT7546E_OSDM  0X0C
#define AT7546E_OSDBL 0X6C
#define AT7546E_STAT  0XA0

/* 字符映射宏 */
#define AT7546E_Letter_a  0X25
#define AT7546E_Letter_b  0X26 
#define AT7546E_Letter_c  0X27
#define AT7546E_Letter_d  0X28
#define AT7546E_Letter_e  0X29
#define AT7546E_Letter_f  0X2A
#define AT7546E_Letter_g  0X2B
#define AT7546E_Letter_h  0X2C
#define AT7546E_Letter_i  0X2D 
#define AT7546E_Letter_j  0X2E
#define AT7546E_Letter_k  0X2F
#define AT7546E_Letter_l  0X30
#define AT7546E_Letter_m  0X31
#define AT7546E_Letter_n  0X32
#define AT7546E_Letter_o  0X33
#define AT7546E_Letter_p  0X34
#define AT7546E_Letter_q  0X35
#define AT7546E_Letter_r  0X36
#define AT7546E_Letter_s  0X37
#define AT7546E_Letter_t  0X38
#define AT7546E_Letter_u  0X39
#define AT7546E_Letter_v  0X3A
#define AT7546E_Letter_w  0X3B
#define AT7546E_Letter_x  0X3C
#define AT7546E_Letter_y  0X3D
#define AT7546E_Letter_z  0X3E

#define AT7546E_Letter_A  0X0B
#define AT7546E_Letter_B  0X0C 
#define AT7546E_Letter_C  0X0D
#define AT7546E_Letter_D  0X0E
#define AT7546E_Letter_E  0X0F
#define AT7546E_Letter_F  0X10
#define AT7546E_Letter_G  0X11
#define AT7546E_Letter_H  0X12
#define AT7546E_Letter_I  0X13 
#define AT7546E_Letter_J  0X14
#define AT7546E_Letter_K  0X15
#define AT7546E_Letter_L  0X16
#define AT7546E_Letter_M  0X17
#define AT7546E_Letter_N  0X18
#define AT7546E_Letter_O  0X19
#define AT7546E_Letter_P  0X1A
#define AT7546E_Letter_Q  0X1B
#define AT7546E_Letter_R  0X1C
#define AT7546E_Letter_S  0X1D
#define AT7546E_Letter_T  0X1E
#define AT7546E_Letter_U  0X1F
#define AT7546E_Letter_V  0X20
#define AT7546E_Letter_W  0X21
#define AT7546E_Letter_X  0X22
#define AT7546E_Letter_Y  0X23
#define AT7546E_Letter_Z  0X24

/* SPI 句柄和使能引脚初始化 */
static SPI_HandleTypeDef *spi_handle = NULL;
static GPIO_TypeDef      *cs_port    = NULL;    
static uint16_t           cs_pin     = 0;

/* CS 引脚控制（按实际接线修改） */
#define AT7456_CS_PORT   GPIOA
#define AT7456_CS_PIN    GPIO_PIN_4

/* 函数声明 */
void   At7456E_Write_Byte(uint8_t Addr, uint8_t Data);
uint8_t At7456E_Read_Byte(uint8_t Addr);
void   AT7456_Read_Len(uint8_t reg, uint8_t len, uint8_t *buf);
void   AT7456_Init(SPI_HandleTypeDef *hspi,GPIO_TypeDef *port, uint16_t pin);
void   At7456E_Write_SRAM(uint8_t row, uint8_t columns, uint8_t word);
void   At7456E_refreshSRAM(uint8_t *SRAM_Addr);
void   At7456E_ClearSRAM(void);
void   At7456E_Open_OSD(void);
void   At7456E_Close_OSD(void);
void   At7456E_Rest(void);
void   Disply_osd(void);
uint8_t Check_AT7456_Present(void);
uint8_t Check_AT7456_Lock(void);
uint8_t Get_Video_Mode(void);
#endif