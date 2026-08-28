#ifndef __BASE_H
#define __BASE_H

#include <stdint.h>

uint8_t get_byte_n(uint8_t byte, uint8_t n);    // 获取某位的值（返回0或1）
uint8_t SET_byte_n_H(uint8_t Byte, uint8_t n);   // 置高某位
uint8_t SET_byte_n_L(uint8_t Byte, uint8_t n);   // 置低某位

#endif
