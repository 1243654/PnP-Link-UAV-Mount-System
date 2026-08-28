#include "mount_monitor.h"

/* 内部静态变量 */
static MM_Config_t g_config;
static MM_OnStateChangeCallback g_callback = NULL;
static volatile uint32_t g_last_heartbeat_tick = 0;
static volatile uint8_t g_heartbeat_flag = 0;
static uint8_t g_signals[3] = {0};     // [EN, 微动, 心跳]
static MM_FaultLevel_t g_current_level = MM_LEVEL_NORMAL;
static MM_StateIndex_t g_current_state = MM_STATE_111;
static uint32_t g_last_process_tick = 0;  // 用于简单速率限制（可选）

/* 内部函数声明 */
static void update_heartbeat_flag(void);
static void read_gpio_signals(void);
static void evaluate_state_machine(void);
static MM_FaultLevel_t index_to_fault_level(uint8_t idx);
static MM_StateIndex_t index_to_state_enum(uint8_t idx);

/* 初始化函数 */
void MM_Init(MM_Config_t* config, MM_OnStateChangeCallback callback)
{
    if (config == NULL) return;
    g_config = *config;
    g_callback = callback;
    
    // 初始化心跳时间（启动时认为无心跳）
    g_last_heartbeat_tick = HAL_GetTick();
    g_heartbeat_flag = 0;
    
    // 初始读取一次GPIO
    read_gpio_signals();
    
    // 首次状态评估
    evaluate_state_machine();
}

/* 在串口接收中断中调用（例如收到任意字节时） */
void MM_OnHeartbeatReceived(void)
{
    g_last_heartbeat_tick = HAL_GetTick();
    // 注意：此函数在中断上下文中，应快速执行，不进行复杂处理
}

/* 内部：更新心跳标志（非阻塞，依赖系统tick） */
static void update_heartbeat_flag(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_last_heartbeat_tick) < g_config.heartbeat_timeout_ms) {
        g_heartbeat_flag = 1;
    } else {
        g_heartbeat_flag = 0;
    }
}

/* 内部：读取两个GPIO信号 */
static void read_gpio_signals(void)
{
    uint8_t en_val = HAL_GPIO_ReadPin(g_config.en_port, g_config.en_pin);
    uint8_t micro_val = HAL_GPIO_ReadPin(g_config.micro_port, g_config.micro_pin);
    g_signals[0] = en_val;
    g_signals[1] = micro_val;
}

/* 内部：将原始信号组合成索引并执行状态机 */
static void evaluate_state_machine(void)
{
    // 更新心跳标志（基于系统tick）
    update_heartbeat_flag();
    // 心跳状态存入g_signals[2]
    g_signals[2] = g_heartbeat_flag;
    
    // 组合索引 (EN<<2)|(micro<<1)|heartbeat
    uint8_t idx = (g_signals[0] << 2) | (g_signals[1] << 1) | g_signals[2];
    MM_StateIndex_t new_state = index_to_state_enum(idx);
    MM_FaultLevel_t new_level = index_to_fault_level(idx);
    
    // 保存当前状态
    g_current_state = new_state;
    
    // 如果等级变化，触发回调
    if (new_level != g_current_level) {
        g_current_level = new_level;
        if (g_callback != NULL) {
            g_callback(new_level, new_state);
        }
    }
}

/* 索引 -> 故障等级映射（可根据需求修改） */
static MM_FaultLevel_t index_to_fault_level(uint8_t idx)
{
    switch(idx) {
        case 0b111: return MM_LEVEL_NORMAL;
        case 0b110: return MM_LEVEL_EMERGENCY;   // 逻辑性故障
        case 0b101: return MM_LEVEL_NORMAL;     // 结构性故障
        case 0b100: return MM_LEVEL_EMERGENCY;     // 双重故障
        case 0b011: return MM_LEVEL_EMERGENCY;     // 连接性故障（EN失效但心跳正常）
        case 0b010: return MM_LEVEL_EMERGENCY;   // 机械到位但EN失效且心跳丢失
        case 0b001: return MM_LEVEL_EMERGENCY;   // 传感器异常
        case 0b000: return MM_LEVEL_EMERGENCY;   // 异常脱钩
        default:    return MM_LEVEL_EMERGENCY;
    }
}

static MM_StateIndex_t index_to_state_enum(uint8_t idx)
{
    return (MM_StateIndex_t)idx;
}

/* 轮询函数：需在main loop中周期性调用（例如每20~50ms） */
void MM_Process(void)
{
    // 读取GPIO（非阻塞）
    read_gpio_signals();
    // 执行状态机评估
    evaluate_state_machine();
}

/* 获取当前故障等级（供外部查询） */
MM_FaultLevel_t MM_GetCurrentLevel(void)
{
    return g_current_level;
}

MM_StateIndex_t MM_GetCurrentStateIndex(void)
{
    return g_current_state;
}
