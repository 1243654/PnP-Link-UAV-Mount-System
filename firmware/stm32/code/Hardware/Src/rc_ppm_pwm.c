#include "rc_ppm_pwm.h"
#include "tim.h"
#include <string.h>

#if 0
// ========== 可自定义的 PPM 解码 ==========
void PPM_Init(PPM_HandleTypeDef *ppm, TIM_HandleTypeDef *htim, uint32_t channel)
{
    ppm->htim = htim;
    ppm->channel = channel;
    ppm->last_count = 0;
    ppm->channel_index = 0;
    ppm->frame_ready = 0;
    memset((void*)ppm->raw_buffer, 0, sizeof(ppm->raw_buffer));
    memset((void*)ppm->channel_value, 0, sizeof(ppm->channel_value));

    HAL_TIM_IC_Start_IT(htim, channel);
}

static void PPM_Decode(PPM_HandleTypeDef *ppm, uint32_t count)
{
    uint16_t width;
    if (count >= ppm->last_count) {
        width = count - ppm->last_count;
    } else {
        width = (PPM_MAX_COUNT - ppm->last_count) + count + 1;
    }
    ppm->last_count = count;

    if (width > PPM_MIN_START) {
        ppm->channel_index = 0;
        ppm->frame_ready = 1;
    } else {
        if (ppm->channel_index < PPM_MAX_CHANNELS) {
            ppm->raw_buffer[ppm->channel_index] = width;
            uint16_t real = width - PPM_FIXED_GAP;
            if (real >= 900 && real <= 2100) {
                ppm->channel_value[ppm->channel_index] = real;
            }
            ppm->channel_index++;
        }
    }
}

void PPM_IRQHandler(PPM_HandleTypeDef *ppm)
{
    uint32_t count = HAL_TIM_ReadCapturedValue(ppm->htim, ppm->channel);
    PPM_Decode(ppm, count);
}

void PPM_GetRawData(PPM_HandleTypeDef *ppm, uint16_t *buf)
{
    for (int i = 0; i < PPM_MAX_CHANNELS; i++) {
        buf[i] = ppm->raw_buffer[i];
    }
}

uint16_t* PPM_GetChannelValues(PPM_HandleTypeDef *ppm)
{
    return (uint16_t*)ppm->channel_value;
}

uint8_t PPM_IsFrameReady(PPM_HandleTypeDef *ppm)
{
    uint8_t ready = ppm->frame_ready;
    ppm->frame_ready = 0;  // 自动清除标志
    return ready;
}
#endif

// ==========  可自定义的 PWM 输入部分 ==========
#define PWM_STATE_RISING    0
#define PWM_STATE_FALLING   1
#define PWM_STATE_PERIOD    2

void PWM_Input_Init(PWM_Input_Handle *pwm, TIM_HandleTypeDef *htim,
                    uint32_t channel, void (*callback)(uint16_t pulse, uint16_t period))
{
    pwm->htim = htim;
    pwm->channel = channel;
    pwm->callback = callback;
    pwm->auto_reload = htim->Instance->ARR;   // 获取自动重载值
    pwm->t_rising = 0;
    pwm->t_falling = 0;
    pwm->high_time = 0;
    pwm->period = 0;
    pwm->duty = 0;
    pwm->overflow_cnt = 0;
    pwm->state = PWM_STATE_RISING;

    HAL_TIM_IC_Start_IT(htim, channel);
}

void PWM_Input_PeriodElapsedCallback(PWM_Input_Handle *pwm)
{
    if (pwm && pwm->htim) {
        pwm->overflow_cnt++;
    }
}

void PWM_Input_IC_CaptureCallback(PWM_Input_Handle *pwm)
{
    if (!pwm || !pwm->htim) return;

    uint32_t capture = HAL_TIM_ReadCapturedValue(pwm->htim, pwm->channel);
    uint32_t total_cnt = capture + pwm->overflow_cnt * (pwm->auto_reload + 1);

    switch (pwm->state) {
        case PWM_STATE_RISING:
            pwm->t_rising = total_cnt;
            __HAL_TIM_SET_CAPTUREPOLARITY(pwm->htim, pwm->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
            pwm->state = PWM_STATE_FALLING;
            break;

        case PWM_STATE_FALLING:
            pwm->t_falling = total_cnt;
            pwm->high_time = pwm->t_falling - pwm->t_rising;
            __HAL_TIM_SET_CAPTUREPOLARITY(pwm->htim, pwm->channel, TIM_INPUTCHANNELPOLARITY_RISING);
            pwm->state = PWM_STATE_PERIOD;
            break;

        case PWM_STATE_PERIOD:
            pwm->period = total_cnt - pwm->t_rising;
            if (pwm->period != 0) {
                pwm->duty = (uint16_t)(((uint32_t)pwm->high_time * 1000) / pwm->period);
                if (pwm->callback) {
                    pwm->callback((uint16_t)pwm->high_time, (uint16_t)pwm->period);
                }
            }
            pwm->state = PWM_STATE_RISING;
            pwm->overflow_cnt = 0;
            break;
    }
}

uint16_t PWM_Input_GetPulse(PWM_Input_Handle *pwm)
{
    return (uint16_t)pwm->high_time;
}

uint16_t PWM_Input_GetPeriod(PWM_Input_Handle *pwm)
{
    return (uint16_t)pwm->period;
}

uint16_t PWM_Input_GetDuty(PWM_Input_Handle *pwm)
{
    return pwm->duty;
}
