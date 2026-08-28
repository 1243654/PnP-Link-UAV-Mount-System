#include "AT7456E.h"
#include "Base.h"
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------ */
/* DWT 微秒延时（初始化由 AT7654_Int 调用）                            */
/* ------------------------------------------------------------------ */
static void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

#define Delay10us()   delay_us(10)
#define Delay50us()   delay_us(50)
#define Delay1us()    delay_us(1)
#define Delay25us()   delay_us(25)

/* ------------------------------------------------------------------ */
/* SPI 底层收发                                                         */
/* ------------------------------------------------------------------ */
static void SPI_CS_Low(void)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

static void SPI_CS_High(void)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

static uint8_t SPI_Transfer(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(spi_handle, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

/* ------------------------------------------------------------------ */
/* 寄存器读写                                                           */
/* ------------------------------------------------------------------ */
void At7456E_Write_Byte(uint8_t Addr, uint8_t Data)
{
    SPI_CS_Low();
    SPI_Transfer(Addr & 0x7F);
    Delay10us();
    SPI_Transfer(Data);
    Delay10us();
    SPI_CS_High();
}

uint8_t At7456E_Read_Byte(uint8_t Addr)
{
    uint8_t buf;
    SPI_CS_Low();
    SPI_Transfer(Addr | 0x80);
    Delay10us();
    buf = SPI_Transfer(0x00);
    Delay10us();
    SPI_CS_High();
    Delay1us();
    return buf;
}

/* ------------------------------------------------------------------ */
/* 连续读取多个寄存器（地址自增）                                       */
/* ------------------------------------------------------------------ */
void AT7456_Read_Len(uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (len == 0) return;
    SPI_CS_Low();
    SPI_Transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = SPI_Transfer(0x00);
    }
    SPI_CS_High();
    Delay25us();
}

/* ------------------------------------------------------------------ */
/* 初始化                                                               */
/* ------------------------------------------------------------------ */
void AT7456_Init(SPI_HandleTypeDef *hspi,GPIO_TypeDef *port, uint16_t pin)
{
    spi_handle = hspi;

    cs_port = port;
    cs_pin  = pin;

    uint8_t buf;
		HAL_Delay(200);
    DWT_Init();

    At7456E_Write_Byte(AT7546E_VM0, 0x40);
    At7456E_Write_Byte(AT7546E_VM1, 0x70);

    buf = At7456E_Read_Byte(AT7546E_DMM);
    buf = SET_byte_n_L(buf, 5);
    At7456E_Write_Byte(AT7546E_DMM, buf);

    buf = At7456E_Read_Byte(AT7546E_DMDI);
    buf = SET_byte_n_L(buf, 7);
    At7456E_Write_Byte(AT7546E_DMDI, buf);

    Delay50us();
    At7456E_ClearSRAM();
}

/* ------------------------------------------------------------------ */
/* 写入一个字符到显示内存                                               */
/* ------------------------------------------------------------------ */
void At7456E_Write_SRAM(uint8_t row, uint8_t columns, uint8_t word)
{
    uint8_t buf;
    uint16_t SRAM_addr = row * 30 + columns;   // 地址可能超256，用 uint16_t

    // 8 位模式
    buf = At7456E_Read_Byte(AT7546E_DMM);
    buf = SET_byte_n_H(buf, 6);
    At7456E_Write_Byte(AT7546E_DMM, buf);

    // 写字符地址字节（DMAH[1]=1）
    buf = At7456E_Read_Byte(AT7546E_DMAH);
    buf = SET_byte_n_H(buf, 1);
    if (SRAM_addr < 256)
        buf = SET_byte_n_L(buf, 0);
    else
        buf = SET_byte_n_H(buf, 0);
    At7456E_Write_Byte(AT7546E_DMAH, buf);

    // 地址低8位
    At7456E_Write_Byte(AT7546E_DMAL, (uint8_t)(SRAM_addr & 0xFF));

    // 字符地址高9位（此处设为0）
    At7456E_Write_Byte(AT7546E_DMDI, 0x00);

    // 切换为写字符地址
    buf = At7456E_Read_Byte(AT7546E_DMAH);
    buf = SET_byte_n_L(buf, 1);
    At7456E_Write_Byte(AT7546E_DMAH, buf);
    At7456E_Write_Byte(AT7546E_DMDI, word);
}

/* ------------------------------------------------------------------ */
/* 刷新显示缓冲区                                                       */
/* ------------------------------------------------------------------ */
void At7456E_refreshSRAM(uint8_t *SRAM_Addr)
{
    uint8_t buf;

    buf = At7456E_Read_Byte(AT7546E_DMAH);
    buf = SET_byte_n_L(buf, 1);
    buf = SET_byte_n_L(buf, 0);
    At7456E_Write_Byte(AT7546E_DMAH, buf);
    At7456E_Write_Byte(AT7546E_DMAL, 0x00);

    At7456E_Write_Byte(AT7546E_DMM, 0xC1);   // 自动递增 + 8位模式

    SPI_CS_Low();
    for (uint16_t i = 0; i < 480; i++)
    {
        SPI_Transfer(SRAM_Addr[i]);
        delay_us(1);   // 可选，保证时序
    }
    SPI_Transfer(0xFF);  // 结束自动递增
    SPI_CS_High();
}

/* ------------------------------------------------------------------ */
/* 清除显示缓冲区                                                       */
/* ------------------------------------------------------------------ */
void At7456E_ClearSRAM(void)
{
    uint8_t buf = At7456E_Read_Byte(AT7546E_DMM);
    buf = SET_byte_n_H(buf, 2);
    At7456E_Write_Byte(AT7546E_DMM, buf);
    Delay50us();
}

/* ------------------------------------------------------------------ */
/* OSD 显示开关                                                         */
/* ------------------------------------------------------------------ */
void At7456E_Open_OSD(void)
{
    uint8_t buf = At7456E_Read_Byte(AT7546E_VM0);
    buf = SET_byte_n_H(buf, 3);
    At7456E_Write_Byte(AT7546E_VM0, buf);
    Delay50us();
}

void At7456E_Close_OSD(void)
{
    uint8_t buf = At7456E_Read_Byte(AT7546E_VM0);
    buf = SET_byte_n_L(buf, 3);
    At7456E_Write_Byte(AT7546E_VM0, buf);
    Delay50us();
}

void At7456E_Rest(void)
{
    uint8_t buf = At7456E_Read_Byte(AT7546E_VM0);
    buf = SET_byte_n_H(buf, 1);
    At7456E_Write_Byte(AT7546E_VM0, buf);
    Delay50us();
}

void Disply_osd(void)
{
    uint8_t buf;
    At7456E_Open_OSD();
    buf = At7456E_Read_Byte(AT7546E_OSDBL);
    buf = SET_byte_n_L(buf, 4);
    At7456E_Write_Byte(AT7546E_OSDBL, buf);
}

// 检查芯片是否存在（读取 STAT 寄存器，看是否能正确返回）
uint8_t Check_AT7456_Present(void)
{
    uint8_t stat = At7456E_Read_Byte(AT7546E_STAT);
    // 判断返回值是否全为0或全为1（可能是SPI未连接）
    if (stat == 0x00 || stat == 0xFF)
        return 0;   // 通信异常
    return 1;       // 通信正常
}

// 检查视频信号是否锁定（STAT[5] = 0 表示锁定）
uint8_t Check_AT7456_Lock(void)
{
    uint8_t stat = At7456E_Read_Byte(AT7546E_STAT);
    if (get_byte_n(stat, 5) == 0)
        return 1;   // 已锁定
    return 0;       // 无视频信号
}

// 读取视频制式（STAT[4] : 0=NTSC, 1=PAL）
uint8_t Get_Video_Mode(void)
{
    uint8_t stat = At7456E_Read_Byte(AT7546E_STAT);
    return get_byte_n(stat, 4);  // 0=NTSC, 1=PAL
}