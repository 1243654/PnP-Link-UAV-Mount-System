#ifndef CONFIG_H
#define CONFIG_H

// 串口配置
#define SERIAL_PORT         Serial2
#define RX2_PIN             16
#define TX2_PIN             17
#define BAUD_RATE           115200

// 舵机引脚
#define SERVO1_PIN          2      // 子ID=1
#define SERVO2_PIN          4      // 子ID=2

// 舵机1（子ID=1）配置
#define SERVO1_PWM_THRESHOLD    1750   // 大于此值转高角度，否则转低角度
#define SERVO1_ANGLE_HIGH       180    // 高角度（当PWM > 阈值）
#define SERVO1_ANGLE_LOW        0      // 低角度（当PWM ≤ 阈值）

// 舵机2（子ID=2）配置
#define SERVO2_PWM_THRESHOLD    1750   // 示例：阈值1500
#define SERVO2_ANGLE_HIGH       180     // 大于1500时转90度
#define SERVO2_ANGLE_LOW        0      // 否则转0度

// 心跳包配置
#define HEARTBEAT_INTERVAL  2000
#define HEARTBEAT_DATA      0xAA

// 协议定义
#define FRAME_HEAD          0xAA
#define FRAME_TAIL          0x55
#define TYPE_HEARTBEAT      0x01
#define TYPE_DATA           0x02

#endif
