#include "OSD.h"
#include "AT7456E.h"
#include "Base.h"
#include <math.h>

/*****显示实数，有0占位**********************/
/***OSD_Disply_IntNumber(uint8_t row,uint8_t columns,uint8_t number_len,int32_t number)****/
/***参数：row：        显示行数0~15**********************/
/***参数：columns：    显示起始列数0~29******************/
/***参数：number_len： 整数占位数，负号占1位*************/
/***参数：number：     需要显示的实数********************/
void OSD_Disply_IntNumber(uint8_t row, uint8_t columns, uint8_t number_len, int32_t number)
{
    uint8_t k, m, i;
    uint8_t Knumber[10] = {0};
    k = 0;

    if (number == 0)
    {
        for (i = 0; i < number_len; i++)
        {
            m = columns + i;
            At7456E_Write_SRAM(row, m, 0x0A);  // 0
        }
    }
    else if (number > 0)
    {
        while (number > 0)
        {
            Knumber[k++] = number % 10;
            number /= 10;
        }
        if (k > number_len) k = number_len;
        for (i = 0; i < number_len; i++)
        {
            m = number_len + columns - i - 1;
            if (i < k)
                At7456E_Write_SRAM(row, m, Knumber[i]);
            else
                At7456E_Write_SRAM(row, m, 0x0A); // 补0
        }
    }
    else  // number < 0
    {
        int32_t m_Abs = -number;
        while (m_Abs > 0)
        {
            Knumber[k++] = m_Abs % 10;
            m_Abs /= 10;
        }
        // 负号
        At7456E_Write_SRAM(row, columns, 0x49);
        if (k > number_len - 1) k = number_len - 1;
        for (i = 0; i < number_len - 1; i++)
        {
            m = number_len + columns - i - 1;
            if (i < k)
                At7456E_Write_SRAM(row, m, Knumber[i]);
            else
                At7456E_Write_SRAM(row, m, 0x0A);
        }
    }
}

/*****显示实数，无0占位**********************/
/***OSD_Disply_IntNumber_2(uint8_t row,uint8_t columns,uint8_t number_len,int32_t number)****/
/***参数：row：        显示行数0~15**********************/
/***参数：columns：    显示起始列数0~29******************/
/***参数：number_len： 整数占位数，负号占1位*************/
/***参数：number：     需要显示的实数********************/
void OSD_Disply_IntNumber_2(uint8_t row, uint8_t columns, uint8_t number_len, int32_t number)
{
    uint8_t k, m, i;
    uint8_t Knumber[10] = {0};
    k = 0;

    if (number == 0)
    {
        m = columns + number_len - 1;
        At7456E_Write_SRAM(row, m, 0x0A);
        for (i = 0; i < number_len - 1; i++)
        {
            m = columns + i;
            At7456E_Write_SRAM(row, m, 0x00);   // 空格或空白
        }
    }
    else if (number > 0)
    {
        while (number > 0)
        {
            Knumber[k++] = number % 10;
            number /= 10;
        }
        if (k > number_len) k = number_len;
        for (i = 0; i < k; i++)
        {
            m = number_len + columns - 1 - i;
            At7456E_Write_SRAM(row, m, Knumber[i]);
        }
        for (i = 0; i < number_len - k; i++)
        {
            m = columns + i;
            At7456E_Write_SRAM(row, m, 0x00);
        }
    }
    else
    {
        int32_t m_Abs = -number;
        while (m_Abs > 0)
        {
            Knumber[k++] = m_Abs % 10;
            m_Abs /= 10;
        }
        if (k > number_len - 1) k = number_len - 1;
        for (i = 0; i < k; i++)
        {
            m = number_len + columns - 1 - i;
            At7456E_Write_SRAM(row, m, Knumber[i]);
        }
        for (i = 0; i < number_len - k - 1; i++)
        {
            m = columns + i + 1;
            At7456E_Write_SRAM(row, m, 0x00);
        }
        At7456E_Write_SRAM(row, columns, 0x49);
    }
}

/*****显示浮点数,无0占位**********************/
/*****OSD_Disply_FloatNumber(uint8_t row,uint8_t n,uint8_t int_n,uint8_t flaot_n,float y)**********************/
/***参数：row：    显示行数0~15********************/
/***参数：columns：显示起始列数0~290~29************/
/***参数：int_n：  整数占位数，负号占1位***********/
/***参数：flaot_n：小数点后几位********************/
/***参数：number： 显示的数字**********************/
void OSD_Disply_FloatNumber(uint8_t row, uint8_t columns, uint8_t int_n, uint8_t float_n, float number)
{
    if (number == 0.0f)
    {
        uint8_t m = columns + int_n + float_n;
        At7456E_Write_SRAM(row, m, 0x0A);
        return;
    }

    int32_t int_part;
    float frac;
    uint8_t i, m, d;

    if (number > 0)
    {
        int_part = (int32_t)number;
        frac = number - int_part;
    }
    else
    {
        float pos = -number;
        int_part = (int32_t)pos;
        frac = pos - int_part;
    }

    // 小数部分
    for (i = 0; i < float_n; i++)
    {
        frac *= 10;
        d = (uint8_t)frac;
        m = columns + int_n + i + 1;
        At7456E_Write_SRAM(row, m, d);
        frac -= d;
    }

    // 小数点
    At7456E_Write_SRAM(row, columns + int_n, 0x41);

    // 整数部分（无占位0）
    OSD_Disply_IntNumber_2(row, columns, int_n, int_part);

    // 处理 -0.x 显示
    if (number < 0 && number > -1.0f)
        At7456E_Write_SRAM(row, columns, 0x49);
}

/**
 * @brief  将标准 ASCII 字符映射为 AT7456E 字库地址
 * @param  ascii  标准 ASCII 字符，如 'A', 'a', '0', ' '
 * @return 对应的 AT7456E 字库地址
 * @note   目前支持大小写字母，未定义的字符返回空格（0x00）
 *         数字和符号
 */
uint8_t AT7456E_Char_Map(char ascii)
{
    // 仅处理 0x00~0x7F 的 ASCII 字符，超出范围返回空格
    if ((uint8_t)ascii >= 128) {
        return 0x00;
    }
    return ASCII_TO_AT7456E[(uint8_t)ascii];
}

/*****显示字符串***********************************/
/*****OSD_Disply_String(uint8_t row, uint8_t start_col, const char *str)**********************/
/***参数：row：    显示行数0~15********************/
/***参数：columns：显示起始列数0~290~29************/
/***参数：str：    字符串输入**********************/
void OSD_Disply_String(uint8_t row, uint8_t start_col, const char *str)
{
    if (str == NULL) return;

    while (*str != '\0') {
        uint8_t chip_char = AT7456E_Char_Map(*str);
        At7456E_Write_SRAM(row, start_col, chip_char);
        start_col++;
        str++;
    }
}
