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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ringbuf.h"
#include "bt_proto.h"
#include "speed_ctrl.h"
#include "motor.h"
#include "oled.h"
#include "stm32f4xx_it.h"
#include "MPU6050.h"
#include "MahonyAHRS.h"
#include "gyro_calib.h"
#include "kinematics.h"
#include "yaw_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BT_RxPacket_t s_rx_pkt;
BT_TxPacket_t s_tx_pkt;
uint8_t s_tx_buf[BT_TX_PACKET_LEN];
uint8_t s_tx_len;
uint32_t s_led_off_tick;  /* LED 闪烁计时 */
float g_roll, g_pitch, g_yaw;  /* 欧拉角 (度)，由 IMU 姿态解算更新 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern RingBuf_t *Get_UART_RxRingBuf(void);
extern volatile int tim8_counter;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * 函    数：IMU 姿态解算更新任务
  * 说    明：以 200Hz 频率读取 MPU6050 数据，运行 Mahony AHRS 算法，
  *           将四元数转换为欧拉角并存入全局变量 g_roll/g_pitch/g_yaw
  */
static void imu_update_task(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    /* 200Hz = 每5ms更新一次 */
    if (now - last_tick < 5) return;
    last_tick = now;

    int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    MPU6050_GetData(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

    /* 原始值转换为物理单位 */
    /* 陀螺仪: ±2000°/s → rad/s,  加速度计: ±16g → g */
    const float gyro_scale  = (2000.0f / 32768.0f) * (3.14159265359f / 180.0f);
    const float accel_scale = 16.0f / 32768.0f;

    float gx = gyro_x * gyro_scale;
    float gy = gyro_y * gyro_scale;
    float gz = gyro_z * gyro_scale;
    float ax = acc_x * accel_scale;
    float ay = acc_y * accel_scale;
    float az = acc_z * accel_scale;

    /* 在线补偿陀螺仪零偏（静止检测 + EMA 估计） */
    GyroCalib_Update(&gx, &gy, &gz, ax, ay, az);

    /* 更新 Mahony AHRS (6-DOF IMU 模式，无磁力计) */
    float q[4] = {q0, q1, q2, q3};
    MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
    q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];

    /* 四元数转欧拉角 (度) */
    QuaternionToEuler(q, &g_roll, &g_pitch, &g_yaw);
}

int Vx = 10;
int Vy = 0;
float wheel_rpm[4];
/**
  * 函    数：底盘运动学控制任务
  * 说    明：以 200Hz 频率执行：摇杆映射 → yaw角度环PID → 全向轮运动学 → 下发4轮目标转速
  */
static void chassis_control_task(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    /* 200Hz = 每5ms执行一次，与 IMU 同步 */
    if (now - last_tick < 5) return;
    last_tick = now;

    /* 1. 摇杆映射：(0~4095, 中心2048) → Vx, Vy 目标速度 */
    // Vx = (int)(((int)rocker_lx - 2048) * ROCKER_SCALE);
    // Vy = (int)(((int)rocker_ly - 2048) * ROCKER_SCALE);

    /* 2. Yaw 角度环 PID → omega */
    // float omega = YawCtrl_Update(g_yaw, 0.005f);
    float omega = 0;
    /* 3. 全向轮运动学解算 → 4轮目标转速 RPM */
    wheel_rpm[4];
    OmniKinematics(Vx, Vy, (int)omega, wheel_rpm);

    /* 4. 下发给速度环 (SpeedCtrl_1kHz_Tick 自动完成闭环) */
    // for (int i = 0; i < 4; i++) {
    //     SpeedCtrl_SetTarget(i, wheel_rpm[i]);
    // }

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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Motor_InitAll();
  SpeedCtrl_Init();
  oled_init();
  // MPU6050_Init();
  // GyroCalib_Init(500);  /* 采集500个样本估计陀螺仪初始零偏（约2.5秒） */
  // YawCtrl_Init();        /* 初始化 yaw 角度环 PID（Kp/Ki/Kd=0，用户自行调试） */
  /* 启用 TIM8 更新中断（PWM 已在 Motor_InitAll 中启动） */
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

  /* 使能 USART3 RXNE 中断 */
  USART3->CR1 |= USART_CR1_RXNEIE;

  /* LED 初始状态 */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  s_led_off_tick = 0;
  NRF_check();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // imu_update_task();
    nrf_receive_task();
    chassis_control_task();
    blue_setparam_task();
    oled_task();

  }  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
