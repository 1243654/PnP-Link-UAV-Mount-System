#ifndef _MOUNT_MONITOR_H
#define _MOUNT_MONITOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 故障等级枚举 */
typedef enum {
    MM_LEVEL_NORMAL = 0,
    MM_LEVEL_ATTENTION,
    MM_LEVEL_WARNING,
    MM_LEVEL_EMERGENCY
} MM_FaultLevel_t;

/* 状态索引（用于调试） */
//0b(EN)(微动)(心跳)
typedef enum {
    MM_STATE_111 = 0b111,   // 正常
    MM_STATE_110 = 0b110,   // 逻辑性故障
    MM_STATE_101 = 0b101,   // 结构性故障
    MM_STATE_100 = 0b100,   // 双重故障
    MM_STATE_011 = 0b011,   // 连接性故障
    MM_STATE_010 = 0b010,   // 紧急预兆
    MM_STATE_001 = 0b001,   // 传感器异常
    MM_STATE_000 = 0b000    // 异常脱钩
} MM_StateIndex_t;

/* 配置结构体（需要用户填充） */
typedef struct {
    // EN脚 GPIO配置
    GPIO_TypeDef* en_port;
    uint16_t en_pin;
    
    // 微动开关 GPIO配置
    GPIO_TypeDef* micro_port;
    uint16_t micro_pin;
    
    // 心跳串口句柄
    UART_HandleTypeDef* heartbeat_huart;
    
    // 心跳超时阈值（毫秒）
    uint32_t heartbeat_timeout_ms;
    
    // 轮询间隔建议（毫秒），内部不会主动延时，由用户控制调用频率
    // 该字段仅用于记录，实际由用户保证调用周期小于超时阈值的一半
} MM_Config_t;

/* 回调函数类型：当故障等级变化时调用 */
typedef void (*MM_OnStateChangeCallback)(MM_FaultLevel_t new_level, MM_StateIndex_t state_index);

/* API 函数 */
void MM_Init(MM_Config_t* config, MM_OnStateChangeCallback callback);
void MM_Process(void);          // 非阻塞轮询，需在main loop中周期性调用（如每50ms）
void MM_OnHeartbeatReceived(void);  // 在串口接收中断中调用此函数（例如收到任意字节）

/* 获取当前状态 */
MM_FaultLevel_t MM_GetCurrentLevel(void);
MM_StateIndex_t MM_GetCurrentStateIndex(void);

#endif
