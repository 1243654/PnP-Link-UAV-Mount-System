#ifndef __OSD_H
#define __OSD_H

#include <stdint.h>

/**
 * @brief  ASCII 到 AT7456E 字库地址的映射表 (128字节)
 *         索引为 ASCII 码值（0x00~0x7F），未定义的字符填 0x00（空格）
 */
static const uint8_t ASCII_TO_AT7456E[128] = {
    // 0x00~0x1F 控制字符，不显示
    [' ']    = 0x00,   // 空格

    // 符号 (按你给出的对应关系)
    ['"']    = 0x48,   // "
    ['\'']   = 0x46,   // '
    ['(']    = 0x3F,
    [')']    = 0x40,
    [',']    = 0x45,
    ['-']    = 0x49,
    ['.']    = 0x41,
    ['/']    = 0x47,
    [':']    = 0x44,
    [';']    = 0x43,
    ['<']    = 0x4A,
    ['>']    = 0x4B,
    ['?']    = 0x42,

    // 数字（待补充，暂时指向空格）
    ['0']    = 0x0A,   // TODO
    ['1']    = 0x01,
    ['2']    = 0x02,
    ['3']    = 0x03,
    ['4']    = 0x04,
    ['5']    = 0x05,
    ['6']    = 0x06,
    ['7']    = 0x07,
    ['8']    = 0x08,
    ['9']    = 0x09,

    // 大写字母 A~Z (0x41~0x5A) 连续段 0x0B~0x24
    ['A']    = 0x0B, ['B'] = 0x0C, ['C'] = 0x0D, ['D'] = 0x0E,
    ['E']    = 0x0F, ['F'] = 0x10, ['G'] = 0x11, ['H'] = 0x12,
    ['I']    = 0x13, ['J'] = 0x14, ['K'] = 0x15, ['L'] = 0x16,
    ['M']    = 0x17, ['N'] = 0x18, ['O'] = 0x19, ['P'] = 0x1A,
    ['Q']    = 0x1B, ['R'] = 0x1C, ['S'] = 0x1D, ['T'] = 0x1E,
    ['U']    = 0x1F, ['V'] = 0x20, ['W'] = 0x21, ['X'] = 0x22,
    ['Y']    = 0x23, ['Z'] = 0x24,

    // 小写字母 a~z (0x61~0x7A) 连续段 0x25~0x3E
    ['a']    = 0x25, ['b'] = 0x26, ['c'] = 0x27, ['d'] = 0x28,
    ['e']    = 0x29, ['f'] = 0x2A, ['g'] = 0x2B, ['h'] = 0x2C,
    ['i']    = 0x2D, ['j'] = 0x2E, ['k'] = 0x2F, ['l'] = 0x30,
    ['m']    = 0x31, ['n'] = 0x32, ['o'] = 0x33, ['p'] = 0x34,
    ['q']    = 0x35, ['r'] = 0x36, ['s'] = 0x37, ['t'] = 0x38,
    ['u']    = 0x39, ['v'] = 0x3A, ['w'] = 0x3B, ['x'] = 0x3C,
    ['y']    = 0x3D, ['z'] = 0x3E,
};

void OSD_Disply_IntNumber(uint8_t row, uint8_t columns, uint8_t number_len, int32_t number);							/*****显示实数，有0占位**********************/
void OSD_Disply_IntNumber_2(uint8_t row, uint8_t columns, uint8_t number_len, int32_t number);						/*****显示实数，无0占位**********************/
void OSD_Disply_FloatNumber(uint8_t row, uint8_t columns, uint8_t int_n, uint8_t float_n, float number);	/*****显示浮点数,无0占位*********************/
void OSD_Disply_String(uint8_t row, uint8_t start_col, const char *str);              										/*****显示字符串*****************************/


#endif
