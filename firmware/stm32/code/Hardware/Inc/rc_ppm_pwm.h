#ifndef __RC_PPM_PWM_H
#define __RC_PPM_PWM_H

#include "main.h"

// ========== 可自定义的 PPM 解码部分 ==========
#define PPM_MAX_CHANNELS   8
#define PPM_MAX_COUNT      65535
#define PPM_MIN_START      2500
#define PPM_FIXED_GAP      400

typedef struct {
    TIM_HandleTypeDef *htim;          // 定时器句柄
    uint32_t channel;                  // 捕获通道
    volatile uint16_t last_count;      // 上次捕获值
    volatile uint16_t raw_buffer[PPM_MAX_CHANNELS];
    volatile uint8_t channel_index;
    volatile uint8_t frame_ready;
    volatile uint16_t channel_value[PPM_MAX_CHANNELS];
} PPM_HandleTypeDef;

void PPM_Init(PPM_HandleTypeDef *ppm, TIM_HandleTypeDef *htim, uint32_t channel);
void PPM_IRQHandler(PPM_HandleTypeDef *ppm);  // 需要在定时器中断中调用
void PPM_GetRawData(PPM_HandleTypeDef *ppm, uint16_t *buf);
uint16_t* PPM_GetChannelValues(PPM_HandleTypeDef *ppm);
uint8_t PPM_IsFrameReady(PPM_HandleTypeDef *ppm);


// ========== 新增 PWM 输入部分 ==========
typedef struct {
    TIM_HandleTypeDef *htim;          // 定时器句柄
    uint32_t channel;                  // 通道
    uint32_t auto_reload;              // 自动重载值 (ARR)
    volatile uint32_t t_rising;        // 上升沿计数值
    volatile uint32_t t_falling;       // 下降沿计数值
    volatile uint32_t high_time;       // 高电平时间 (us)
    volatile uint32_t period;           // 周期时间 (us)
    volatile uint16_t duty;             // 占空比 (千分比)
    volatile uint32_t overflow_cnt;     // 定时器溢出计数
    volatile uint8_t state;             // 捕获状态机
    void (*callback)(uint16_t pulse, uint16_t period); // 数据更新回调
} PWM_Input_Handle;

void PWM_Input_Init(PWM_Input_Handle *pwm, TIM_HandleTypeDef *htim,
                    uint32_t channel, void (*callback)(uint16_t pulse, uint16_t period));
void PWM_Input_PeriodElapsedCallback(PWM_Input_Handle *pwm);
void PWM_Input_IC_CaptureCallback(PWM_Input_Handle *pwm);
uint16_t PWM_Input_GetPulse(PWM_Input_Handle *pwm);
uint16_t PWM_Input_GetPeriod(PWM_Input_Handle *pwm);
uint16_t PWM_Input_GetDuty(PWM_Input_Handle *pwm);

#endif
