/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "servo_uart.h"
#include "rc_ppm_pwm.h"
#include "servo_pwm.h"
#include "OSD.h"
#include "AT7456E.h"
#include "Base.h"
#include "protocol.h"
#include "mount_monitor.h"
#include <math.h>
#include <stdio.h>   
#include <string.h>  
#include <stdarg.h> 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart3;  // 声明串口3句柄
extern UART_HandleTypeDef huart4;  // 声明串口4句柄
extern UART_HandleTypeDef huart5;  // 声明串口5句柄
MPU9250_HandleTypeDef hmpu9250;
Kalman_HandleTypeDef kalman_roll;   
Kalman_HandleTypeDef kalman_pitch;  
PWM_Input_Handle rc_ch[4];	   // TIM2 的 4 个 PWM 输入通道
PWM_Input_Handle rc_ch3[4];    // TIM3 的 4 个 PWM 输入通道
Servo_Handle servo9g_Left;
Servo_Handle servo9g_Right;
static MM_Config_t monitor_cfg;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//指示灯------------------------------------------------------------------------
/*(已在main文件中宏定义)
红色指示灯
#define R_Pin GPIO_PIN_10
#define R_GPIO_Port GPIOA

绿色指示灯
#define G_Pin GPIO_PIN_11
#define G_GPIO_Port GPIOA

蓝色指示灯
#define B_Pin GPIO_PIN_12
#define B_GPIO_Port GPIOA

子挂载使能引脚
#define EN_Pin GPIO_PIN_0
#define EN_GPIO_Port GPIOC

子挂载检测微动B
#define MICROSWITCH_B_Pin GPIO_PIN_1
#define MICROSWITCH_B_GPIO_Port GPIOC

子挂载检测微动A
#define MICROSWITCH_A_Pin GPIO_PIN_2
#define MICROSWITCH_A_GPIO_Port GPIOC

手动脱钩
#define MANUAL_REMOVAL_Pin GPIO_PIN_3
#define MANUAL_REMOVAL_GPIO_Port GPIOC

OSD叠加模块的片选引脚CS
#define AT7456_CS_Pin GPIO_PIN_4
#define AT7456_CS_GPIO_Port GPIOA
*/

// 遥控器阈值-----------------------------------------------------------------
#define STABILIZE_THRESHOLD  	 1750   // 通道4 > 1750 开启自稳
#define SWITCH_THRESHOLD     	 1750   // 通道3 > 1750 触发9g舵机

// 遥控器脉宽范围 (us)--------------------------------------------------------
#define RC_MIN_PULSE         	 1000
#define RC_MAX_PULSE        	 2000
#define RC_MID_PULSE        	 1500

// 角度范围-------------------------------------------------------------------
#define ANGLE_MIN             	-90
#define ANGLE_MAX             	 90

//脱钩舵机角度----------------------------------------------------------------
#define DECOUPLER_START_LEFT_ANGLE	 40	   //脱钩舵机挂钩角度
#define DECOUPLER_STOP_LEFT_ANGLE	 115   //脱钩舵机释放角度
#define DECOUPLER_START_RIGHT_ANGLE	 115
#define DECOUPLER_STOP_RIGHT_ANGLE	 10

// 飞特舵机映射参数-----------------------------------------------------------
/*参考点位
#define SERVO_MIN                1024      //飞特舵机限幅最小值
#define SERVO_MAX                3072      //飞特舵机限幅最大值
#define SERVO_MID                2048      //飞特舵机中位点*/
/*实际点位*/
#define SERVO_MIN                1600      //飞特舵机限幅最小值
#define SERVO_MAX                3000      //飞特舵机限幅最大值
#define SERVO_MID                2355      //飞特舵机中位点

#define SERVO_RANGE              2048      // 3072-1024
#define ANGLE_RANGE              180       // 90 - (-90)
#define ANGLE_TO_SERVO_FACTOR  (SERVO_RANGE / ANGLE_RANGE)  // ≈11.3778
//#define ANGLE_TO_SERVO_FACTOR     5

#define SERVO_ROLL_ID            1   
#define SERVO_PITCH_ID 	        2   

// PID 参数-------------------------------------------------------------------
#define KP  1.2f   // 比例增益
#define KI  0.0f    // 积分增益（消除静差）
#define KD  0.0f    // 微分增益（角速度阻尼）

// 积分限幅，防止饱和---------------------------------------------------------
#define INTEGRAL_LIMIT 50.0f

// 低通滤波系数 (用于角度测量，减小噪声)--------------------------------------
#define ANGLE_LPF_ALPHA 1.0f   // 值越小越平滑，但响应变慢，0.2~0.5

#define RX_BUF_SIZE 256
static uint8_t protocol_rx_buf[RX_BUF_SIZE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 脱钩舵机状态机变量----------------------------------------------------------
typedef enum {
    DECOUPLER_START = 0,   // 挂钩（0°）
    DECOUPLER_STOP  = 1    // 脱钩（90°）
} DECOUPLER_State_t;

DECOUPLER_State_t targetDecouplerState;  // 目标状态（由遥控器或手动按键决定）
int manualOverride = 0;                  // 手动覆盖标志：1表示正在手动脱钩，0表示由遥控器控制

// 陀螺仪变量------------------------------------------------------------------
float gyro_bias[3] = {0,0,0};

uint32_t last_tick = 0;
float dt = 0.01f;  

uint8_t mpu9250_ok = 0;  // 0: 失败/未初始化, 1: 正常

// 心跳监控（用于灯光）
static uint32_t last_heartbeat_tick = 0;
#define HEARTBEAT_TIMEOUT_MS 3000   // 与 mount_monitor 保持一致

static uint8_t last_main_id = 0xFF;  // 初始值不同于任何有效 ID
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CheckKeys(void);
uint8_t MPU9250_Reinit(void);
void on_frame_received(uint8_t main_id, uint8_t sub_id, uint8_t type,uint8_t *data, uint8_t len);
void on_mount_state_change(MM_FaultLevel_t level, MM_StateIndex_t state);
static void MPU9250_CalibrateGyro(MPU9250_HandleTypeDef *dev, float *bias, uint16_t samples);
void Servo_Test(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 简单调试打印函数（阻塞式发送字符串）-----------------------------------------
void Debug_Print(const char *str)
{
    HAL_UART_Transmit(&huart4, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// 带格式的打印（使用sprintf）--------------------------------------------------
void Debug_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void on_frame_received(uint8_t main_id, uint8_t sub_id, uint8_t type,
                       uint8_t *data, uint8_t len)
{
    if (type == 0x01) {
        // 心跳包：通知挂载监控
        MM_OnHeartbeatReceived();
		last_heartbeat_tick = HAL_GetTick(); // 新增：记录心跳时间
/*   	 数据包：根据 (main_id, sub_id) 分发挂载类型
	
	   桂载种类:SDC      	--- 	Serial DataDisconnection	未连接
				UAV-APT    	--- 	UAV Airdrop Payload Type	通用空役类挂载
				UAV-AB-PL	---		UAV Airborne Payload		通用空中使用类挂载
				UAV-UD-PL	---		UAV User-Defined Payload
				UAV-AUT-PL	---		UAV Autonomous Payload
				UPT			---		Undefined Payload Type
*/
        // 挂载类型显示（仅在类型变化时更新 OSD）
        if (main_id != last_main_id) {
            last_main_id = main_id;
            switch (main_id) {
                case 0x01:
                    OSD_Disply_String(4, 7, "UAV-APT");
                    OSD_Disply_String(5, 7, "ON ");
                    break;
                case 0x02:
                    OSD_Disply_String(4, 7, "UAV-AB-PL");
                    OSD_Disply_String(5, 7, "ON ");
                    break;
                default:
                    // 未知类型可显示默认字符串
                    OSD_Disply_String(4, 7, "UPT");
                    OSD_Disply_String(5, 7, "ERR");
                    break;
            }
        }
        // 打印调试信息
        Debug_Printf("Heartbeat from (0x%02X,0x%02X)\r\n", main_id, sub_id);
    }
    else if (type == 0x02) {

        switch (main_id) {
            case 0x01:  //数据处理

                break;
            case 0x02:  //数据处理

                break;
            default:
                break;
        }
    }
    
}
//按键检测----------------------------------------------------------------------
void CheckKeys(void) {
    static uint32_t lastKeyTime = 0;
    static uint8_t last_micro_state = 0;  // 上次微动状态（默认拔出，低电平）
    uint32_t now = HAL_GetTick();
    if ((now - lastKeyTime) < 200) return;
    
    // 手动脱钩按键
    if (HAL_GPIO_ReadPin(MANUAL_REMOVAL_GPIO_Port, MANUAL_REMOVAL_Pin) == GPIO_PIN_RESET) {
        manualOverride = 1;
        targetDecouplerState = DECOUPLER_STOP;
        lastKeyTime = now;
    } else {
        manualOverride = 0;
		lastKeyTime = now;
    }

	// 子挂载接入检测（微动开关 A，高电平表示插入））
	uint8_t micro_state = HAL_GPIO_ReadPin(MICROSWITCH_A_GPIO_Port, MICROSWITCH_A_Pin);

	// 上升沿检测（插入）
	if (micro_state == GPIO_PIN_SET && last_micro_state == GPIO_PIN_RESET) {
		// 彻底重置串口3
		HAL_UART_DMAStop(&huart3);
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		// 清空接收缓冲区
		memset(protocol_rx_buf, 0, RX_BUF_SIZE);
		// 重新初始化协议库
		PROTOCOL_Init(&huart3, protocol_rx_buf, RX_BUF_SIZE);
		PROTOCOL_RegisterCallback(MAIN_ID_ANY, SUB_ID_ANY, on_frame_received);
		Debug_Print("Mount inserted, UART3 reset\r\n");
		last_micro_state = GPIO_PIN_SET;
	}
	// 下降沿检测（拔出）
	else if (micro_state == GPIO_PIN_RESET && last_micro_state == GPIO_PIN_SET) {
		// 拔出时也重置串口，以便下次插入正常工作
		HAL_UART_DMAStop(&huart3);
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		PROTOCOL_Init(&huart3, protocol_rx_buf, RX_BUF_SIZE);
		PROTOCOL_RegisterCallback(MAIN_ID_ANY, SUB_ID_ANY, on_frame_received);
		Debug_Print("Mount removed, UART3 reset\r\n");
		last_micro_state = GPIO_PIN_RESET;
	}
	
	// 如果 MPU9250 之前失败，尝试重新初始化
	if (!mpu9250_ok) {
		if (MPU9250_Reinit()) {
			mpu9250_ok = 1;
		}
	}
    lastKeyTime = now;
}
uint8_t MPU9250_Reinit(void) {
    HAL_StatusTypeDef status = MPU9250_Init(&hmpu9250);
    if (status != HAL_OK) {
        Debug_Print("MPU9250 初始化失败\r\n");
        return 0;
    }else {
			Debug_Print("MPU9250 初始化成功\r\n");
	}

    // 校准陀螺仪
    MPU9250_CalibrateGyro(&hmpu9250, gyro_bias, 100);
    float tmp = gyro_bias[0];
    gyro_bias[0] = gyro_bias[1];
    gyro_bias[1] = tmp;
    Debug_Printf("MPU9250 重新初始化成功，偏置 X=%.2f Y=%.2f Z=%.2f\r\n", 
                 gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    // 重新初始化卡尔曼滤波器
    MPU9250_Kalman_Init(&kalman_roll, 0.001f, 0.003f, 0.03f);
    MPU9250_Kalman_Init(&kalman_pitch, 0.001f, 0.003f, 0.03f);
    return 1;
}
void on_mount_state_change(MM_FaultLevel_t level, MM_StateIndex_t state)
{
    // 仅打印状态变化（灯光已由独立逻辑控制）
    Debug_Printf("[MOUNT] Level: %d, State: %d\r\n", level, state);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_StatusTypeDef status;
	AT7456_Init(&hspi1,GPIOB,GPIO_PIN_0);
	HAL_Delay(2000);
	
	Debug_Print("\r\n========== MPU9250舵机云台启动 ==========\r\n");
	//============================ 1. 初始化LED  ========================================================================
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); // 自检指示灯（默认点亮，板载PC13高电平点亮，）

	
	//============================ 2. 初始化MPU9250  ====================================================================
	hmpu9250.hi2c = &hi2c1;
	mpu9250_ok = MPU9250_Reinit();
	if (!mpu9250_ok) {
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET); 
		Debug_Print("MPU9250 初始化失败，将定期重试\r\n");
	}
	
	
	//============================ 5. 测试舵机发送（验证舵机通信）  =====================================================
	Debug_Print("测试舵机1（ID=1）移动到中位...\r\n");
	HAL_StatusTypeDef servo_status = Servo_WritePosEx(&huart5, SERVO_ROLL_ID, SERVO_MID, 200);
	if (servo_status != HAL_OK) {
			Debug_Print("舵机指令发送失败\r\n");
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);  // 自检指示灯熄灭
	} else {
			Debug_Print("舵机指令发送成功！\r\n");
	}
	Servo_Test();
	
	
	//============================ 6. 初始化遥控器PWM输入 (TIM2）  ======================================================
	PWM_Input_Init(&rc_ch[0], &htim2, TIM_CHANNEL_1, NULL);
	PWM_Input_Init(&rc_ch[1], &htim2, TIM_CHANNEL_2, NULL);
	PWM_Input_Init(&rc_ch[2], &htim2, TIM_CHANNEL_3, NULL);
	PWM_Input_Init(&rc_ch[3], &htim2, TIM_CHANNEL_4, NULL);
	
	PWM_Input_Init(&rc_ch3[0], &htim3, TIM_CHANNEL_1, NULL);
	PWM_Input_Init(&rc_ch3[1], &htim3, TIM_CHANNEL_2, NULL);
	PWM_Input_Init(&rc_ch3[2], &htim3, TIM_CHANNEL_3, NULL);
	PWM_Input_Init(&rc_ch3[3], &htim3, TIM_CHANNEL_4, NULL);

	
	//============================ 7. 初始化9g舵机 (TIM4 CH1、CH2) ======================================================
	Servo_Init(&servo9g_Left, &htim4, TIM_CHANNEL_1, 500, 2500);//0~180°
	Servo_Start(&servo9g_Left);
	Servo_SetAngle(&servo9g_Left, DECOUPLER_START_LEFT_ANGLE);
	
	Servo_Init(&servo9g_Right, &htim4, TIM_CHANNEL_2, 500, 2500);//0~180°
	Servo_Start(&servo9g_Right);
	Servo_SetAngle(&servo9g_Right, DECOUPLER_START_RIGHT_ANGLE);
	
	targetDecouplerState = DECOUPLER_START;
	manualOverride = 0;
	
	
	//============================ 8. 初始化OSD界面  ===================================================================
	/*挂载自稳（GBS）Gimbal Stabilization*/
		OSD_Disply_String(2,2,"GBS:");
		OSD_Disply_String(2,7,"ON ");
	/*挂载状态（GBT）Gimbal Status*/
		OSD_Disply_String(3,2,"GBT:");
		OSD_Disply_String(3,7,"SU ");
	/*挂载种类（GBK）Gimbal Kind*/
		OSD_Disply_String(4,2,"GBK:");
		OSD_Disply_String(4,7,"SRC");
	/*挂载回传数据（GDD）Gimbal Downlink Data*/
		OSD_Disply_String(5,2,"GDD:");
		OSD_Disply_String(5,7,"ON ");
		
		Disply_osd();


	//============================ 9. 配置挂载监测库  ==================================================================
    monitor_cfg.en_port = EN_GPIO_Port;
    monitor_cfg.en_pin = EN_Pin;
    monitor_cfg.micro_port = MICROSWITCH_A_GPIO_Port;
    monitor_cfg.micro_pin = MICROSWITCH_A_Pin;
    monitor_cfg.heartbeat_huart = &huart3;
    monitor_cfg.heartbeat_timeout_ms = 3000;
    
    MM_Init(&monitor_cfg, on_mount_state_change);
    
	
	//============================ 10. 初始化 protocol 库 ==============================================================
	PROTOCOL_Init(&huart3, protocol_rx_buf, RX_BUF_SIZE);
	PROTOCOL_RegisterCallback(MAIN_ID_ANY, SUB_ID_ANY, on_frame_received);


	//============================ 11. 初始化完成，自检指示灯常亮1S  =============================================
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET); 
	HAL_Delay(1000);
	uint32_t loop_count = 0;
	
	
	//============================ 12. 记录起始时间  ===================================================================
	last_tick = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ========== 1. 时间计算 =============================================
    uint32_t now = HAL_GetTick();
    dt = (now - last_tick) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    last_tick = now;
	  
	// 周期性重试 MPU9250 初始化（如果之前失败）
	static uint32_t last_mpu_retry = 0;
	if (!mpu9250_ok && (HAL_GetTick() - last_mpu_retry > 2000)) {
		last_mpu_retry = HAL_GetTick();
		if (MPU9250_Reinit()) {
			mpu9250_ok = 1;
//			HAL_GPIO_WritePin(SELF_CHECK_GPIO_Port, SELF_CHECK_Pin, GPIO_PIN_SET);
		}
	}

	/* 非阻塞地处理挂载状态机（建议每 20~50ms 调用一次） */
	static uint32_t last_mm_time = 0;
	if (HAL_GetTick() - last_mm_time >= 50) {
		last_mm_time = HAL_GetTick();
		MM_Process();
	}
	
	    // 静态变量，用于保持上一次的滤波角度和角速度（跨循环使用）
    static float smooth_roll = 0, smooth_pitch = 0;
    static float gx = 0, gy = 0;
	
    // ========== 2. 读取传感器数据并处理 =================================
    if (mpu9250_ok) {
        HAL_StatusTypeDef accel_status = MPU9250_ReadAccel(&hmpu9250);
        HAL_StatusTypeDef gyro_status = MPU9250_ReadGyro(&hmpu9250);
        if (accel_status != HAL_OK || gyro_status != HAL_OK) {
            Debug_Print("传感器读取失败！\r\n");
            mpu9250_ok = 0;  // 标记失败，后续重试
            // 保持上一次的 smooth_roll, smooth_pitch, gx, gy 不变
        } else {
            // 交换 X/Y 轴
            float ax = hmpu9250.accel[1];
            float ay = hmpu9250.accel[0];
            float az = hmpu9250.accel[2];
            gx = hmpu9250.gyro[1] - gyro_bias[1];
            gy = hmpu9250.gyro[0] - gyro_bias[0];
            float gz = hmpu9250.gyro[2] - gyro_bias[2];

            // 加速度计角度
            float accel_roll  = MPU9250_AccelToAngle(ax, ay, az, 'r');
            float accel_pitch = MPU9250_AccelToAngle(ax, ay, az, 'p');

            // 卡尔曼滤波更新
            float filtered_roll  = MPU9250_Kalman_Update(&kalman_roll,  accel_roll,  gx, dt);
            float filtered_pitch = MPU9250_Kalman_Update(&kalman_pitch, accel_pitch, gy, dt);

            // 修正符号
            filtered_roll  = -filtered_roll;
            filtered_pitch = -filtered_pitch;

            // 低通滤波
            float alpha = ANGLE_LPF_ALPHA;
            smooth_roll  = alpha * filtered_roll  + (1 - alpha) * smooth_roll;
            smooth_pitch = alpha * filtered_pitch + (1 - alpha) * smooth_pitch;
        }
    } else {
        // 传感器未就绪，保持上次值，可加短暂延时避免空转
        HAL_Delay(5);
    }

	
    // ========== 3. 读取遥控器 PWM 脉宽 ===================================
	uint16_t raw_ch1 = PWM_Input_GetPulse(&rc_ch[0]);
	uint16_t raw_ch2 = PWM_Input_GetPulse(&rc_ch[1]);
	uint16_t raw_ch3 = PWM_Input_GetPulse(&rc_ch[2]);
	uint16_t raw_ch4 = PWM_Input_GetPulse(&rc_ch[3]);

	// 所有通道统一有效性判断（有效范围 900~2100µs）
	if (raw_ch1 < 900 || raw_ch1 > 2100) raw_ch1 = RC_MID_PULSE;
	if (raw_ch2 < 900 || raw_ch2 > 2100) raw_ch2 = RC_MID_PULSE;
	if (raw_ch3 < 900 || raw_ch3 > 2100) raw_ch3 = RC_MID_PULSE;
	if (raw_ch4 < 900 || raw_ch4 > 2100) raw_ch4 = RC_MID_PULSE;

    // 限幅到有效范围
	if (raw_ch1 < RC_MIN_PULSE) raw_ch1 = RC_MIN_PULSE;
	if (raw_ch1 > RC_MAX_PULSE) raw_ch1 = RC_MAX_PULSE;
	if (raw_ch2 < RC_MIN_PULSE) raw_ch2 = RC_MIN_PULSE;
	if (raw_ch2 > RC_MAX_PULSE) raw_ch2 = RC_MAX_PULSE;
	if (raw_ch3 < RC_MIN_PULSE) raw_ch3 = RC_MIN_PULSE;
	if (raw_ch3 > RC_MAX_PULSE) raw_ch3 = RC_MAX_PULSE;
	if (raw_ch4 < RC_MIN_PULSE) raw_ch4 = RC_MIN_PULSE;
	if (raw_ch4 > RC_MAX_PULSE) raw_ch4 = RC_MAX_PULSE;

    // 将摇杆（通道1、2）映射到目标角度（-90° ~ 90°）
    float target_roll  = (float)(raw_ch1 - RC_MID_PULSE) * ANGLE_RANGE / (RC_MAX_PULSE - RC_MIN_PULSE);
    float target_pitch = (float)(raw_ch2 - RC_MID_PULSE) * ANGLE_RANGE / (RC_MAX_PULSE - RC_MIN_PULSE);

    // 自稳模式开关（通道4）
    uint8_t stabilize_mode = (raw_ch3 < STABILIZE_THRESHOLD) ? 1 : 0;


    // ========== 4. 脱钩舵机控制（通道3 + 手动按键） ======================
	CheckKeys();
    if (!manualOverride) {
        if (raw_ch4 > SWITCH_THRESHOLD)
            targetDecouplerState = DECOUPLER_STOP;
        else
            targetDecouplerState = DECOUPLER_START;
    }
    Servo_SetAngle(&servo9g_Left, (targetDecouplerState == DECOUPLER_STOP) ? DECOUPLER_STOP_LEFT_ANGLE : DECOUPLER_START_LEFT_ANGLE);
    Servo_SetAngle(&servo9g_Right, (targetDecouplerState == DECOUPLER_STOP) ? DECOUPLER_STOP_RIGHT_ANGLE : DECOUPLER_START_RIGHT_ANGLE);

	
    // ========== 5. 云台控制（自稳 / 手动） ===============================
    int16_t roll_pos, pitch_pos;

    static float integral_roll = 0, integral_pitch = 0;

	if (stabilize_mode) {
		// 增稳模式：误差 = 机身角度 - 目标角度（目标角度通常为0）
		float error_roll  = smooth_roll - target_roll;
		float error_pitch = smooth_pitch - target_pitch;
		// 纯比例，KP=1，无积分微分
		roll_pos  = SERVO_MID - (int16_t)(error_roll  * ANGLE_TO_SERVO_FACTOR);
		pitch_pos = SERVO_MID - (int16_t)(error_pitch * ANGLE_TO_SERVO_FACTOR);
		// 更新 OSD
		OSD_Disply_String(2, 7, "ON ");
	}
    else
    {
        // 手动模式：直接映射遥控器角度
        roll_pos  = SERVO_MID + (int16_t)(target_roll  * ANGLE_TO_SERVO_FACTOR);
        pitch_pos = SERVO_MID + (int16_t)(target_pitch * ANGLE_TO_SERVO_FACTOR);

        // 清零积分，避免切回自稳时跳变
        integral_roll = 0;
        integral_pitch = 0;

        OSD_Disply_String(2, 7, "OFF");
    }

    // 限幅到舵机安全范围
    if (roll_pos < SERVO_MIN) roll_pos = SERVO_MIN;
    if (roll_pos > SERVO_MAX) roll_pos = SERVO_MAX;
    if (pitch_pos < SERVO_MIN) pitch_pos = SERVO_MIN;
    if (pitch_pos > SERVO_MAX) pitch_pos = SERVO_MAX;

	
    // ========== 6. 发送舵机指令 ===========================================
    Servo_WritePosEx(&huart5, SERVO_ROLL_ID,  roll_pos,  0);
    Servo_WritePosEx(&huart5, SERVO_PITCH_ID, pitch_pos, 0);

	
	// ========== 7. 子挂载控制信号 (TIM3 CH1~CH4) ==========
	// 定义通道对应的子 ID
	const uint8_t sub_ids[] = {0x01, 0x02, 0x03, 0x04};
	static uint16_t last_pulse[4] = {0};

	for (int i = 0; i < 4; i++) {
		uint16_t pulse = PWM_Input_GetPulse(&rc_ch3[i]);
		
		// 信号有效性判断：无效则强制设为中位值 1500
		if (pulse < 900 || pulse > 2100) {
			pulse = 1500;
		} else {
			// 限幅到标准遥控器范围 1000~2000
			if (pulse < RC_MIN_PULSE) pulse = RC_MIN_PULSE;
			if (pulse > RC_MAX_PULSE) pulse = RC_MAX_PULSE;
		}
		
		// 仅在数值变化时发送
		if (pulse != last_pulse[i]) {
			last_pulse[i] = pulse;
			uint8_t data[2] = {
				(uint8_t)(pulse & 0xFF),
				(uint8_t)((pulse >> 8) & 0xFF)
			};
			PROTOCOL_SendFrame(0x01, sub_ids[i], 0x02, data, 2);
		}
	}

		
	// 判断是否已接入（EN引脚有效）
	uint8_t mount_connected = (HAL_GPIO_ReadPin(EN_GPIO_Port, EN_Pin) == GPIO_PIN_RESET); // 假设低电平接入

	if (!mount_connected) {
		// 未接入 -> 黄灯（空闲等待）
		HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	} else {
		// 已接入，检查故障条件（心跳超时）
		uint8_t heartbeat_ok = (HAL_GetTick() - last_heartbeat_tick) < HEARTBEAT_TIMEOUT_MS;
		if (!heartbeat_ok) {
			// 已接入但心跳丢失 -> 红灯（故障）
			HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
		} else {
			// 已接入且心跳正常，根据脱钩状态决定颜色
			if (targetDecouplerState == DECOUPLER_STOP) {
				// 脱钩状态 -> 黄灯（正常工作状态，不算故障）
				HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
				last_main_id = 0xFF;
				OSD_Disply_String(3,7,"SU ");
				OSD_Disply_String(4,7,"SDC                   ");
				OSD_Disply_String(5,7,"OFF");
			} else {
				// 正常接入且未脱钩 -> 绿灯
				HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
				OSD_Disply_String(3,7,"SC ");
			}
		}
	}

//	// ========== 8. 调试打印（每50次） =====================================
//	static uint32_t print_cnt = 0;
//	if (++print_cnt >= 50)
//	{
//		print_cnt = 0;
//		// 原有四个通道 + 模式
//		Debug_Printf("CH1=%d CH2=%d CH3=%d CH4=%d Mode=%d\r\n",
//					 raw_ch1, raw_ch2, raw_ch3, raw_ch4, stabilize_mode);
//		Debug_Printf("SubPulse: CH1=%d CH2=%d CH3=%d CH4=%d\r\n",
//					 last_pulse[0], last_pulse[1], last_pulse[2], last_pulse[3]);
//		// 角度信息
//		Debug_Printf("Target R=%.1f P=%.1f | Filtered R=%.1f P=%.1f\r\n",
//					 target_roll, target_pitch, smooth_roll, smooth_pitch);
//		// 舵机位置
//		Debug_Printf("Pos R=%d P=%d\r\n", roll_pos, pitch_pos);
//	}
//	
//	
    // 控制周期 10ms (100Hz)
    HAL_Delay(5);
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 陀螺仪偏置校准（静止时采集平均值）++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  * @param dev  MPU9250句柄
  * @param bias 陀螺仪偏置数组（度/秒）
  * @param samples 采样次数
  */
static void MPU9250_CalibrateGyro(MPU9250_HandleTypeDef *dev, float *bias, uint16_t samples)
{
    float sum[3] = {0,0,0};
    for (uint16_t i = 0; i < samples; i++) {
        MPU9250_ReadGyro(dev);
        sum[0] += dev->gyro[0];
        sum[1] += dev->gyro[1];
        sum[2] += dev->gyro[2];
        HAL_Delay(5); // 延迟5ms
    }
    bias[0] = sum[0] / samples;
    bias[1] = sum[1] / samples;
    bias[2] = sum[2] / samples;
	HAL_GPIO_TogglePin(B_GPIO_Port, B_Pin); // 自检指示灯闪烁
}


/**
* @brief 云台舵机测试+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  */
void Servo_Test(void)
{
    uint8_t resp[6];
    Debug_Print("发送舵机指令 ID=1 位置=2048...\r\n");
    if (Servo_WritePosEx(&huart5, 1, 2048, 500) == HAL_OK) {
        Debug_Print("指令发送成功，等待应答...\r\n");
        if (Servo_Receive(&huart5, resp, 100) == HAL_OK) {
            Debug_Printf("应答数据: %02X %02X %02X %02X %02X %02X\r\n",
                         resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
        } else {
            Debug_Print("无应答或超时！\r\n");
        }
    } else {
        Debug_Print("指令发送失败！\r\n");
    }
}

/**
  * @brief 定时器输入捕获回调（静止时采集平均值）+++++++++++++++++++++++++++++++++++++++++++++++++++
  * @param htim  输入捕获句柄
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	    //  TIM2 处理
    if (htim->Instance == TIM2) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {PWM_Input_IC_CaptureCallback(&rc_ch[0]);} 
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {PWM_Input_IC_CaptureCallback(&rc_ch[1]);} 
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {PWM_Input_IC_CaptureCallback(&rc_ch[2]);} 
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {PWM_Input_IC_CaptureCallback(&rc_ch[3]);}
    }
	    //  TIM3 处理
	    else if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) PWM_Input_IC_CaptureCallback(&rc_ch3[0]);
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) PWM_Input_IC_CaptureCallback(&rc_ch3[1]);
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) PWM_Input_IC_CaptureCallback(&rc_ch3[2]);
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) PWM_Input_IC_CaptureCallback(&rc_ch3[3]);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // 如果 TIM2 也需要溢出处理，可以在这里调用（但你的 TIM2 可能没使能更新中断）
        // 为了完整性，可以为每个通道调用，但通常 TIM2 只捕获不溢出？
        // 简单起见，TIM2 的溢出我们暂不处理，因为 TIM2 计数器可能很大（32位）
        // 但如果你在 TIM2 中使用了 overflow_cnt，则需要添加。
    }
    else if (htim->Instance == TIM3) {
        // TIM3 所有通道共享同一个溢出计数器，每个通道都要更新
        PWM_Input_PeriodElapsedCallback(&rc_ch3[0]);
        PWM_Input_PeriodElapsedCallback(&rc_ch3[1]);
        PWM_Input_PeriodElapsedCallback(&rc_ch3[2]);
        PWM_Input_PeriodElapsedCallback(&rc_ch3[3]);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3) {
        PROTOCOL_ProcessRxData(protocol_rx_buf, Size);
		
		HAL_UART_DMAStop(huart);   // 停止 DMA 和 UART 的相关中断
        // 重新启动 DMA 接收（注意：PROTOCOL_Init 已经启动过一次，但空闲中断后 DMA 可能停止，需要重新启动）
        HAL_UARTEx_ReceiveToIdle_DMA(huart, protocol_rx_buf, RX_BUF_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        PROTOCOL_TxCpltCallback(huart);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
