/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rabcl/utils/type.hpp"
#include "rabcl/utils/utils.hpp"
#include "rabcl/interface/uart.hpp"
#include "rabcl/interface/can.hpp"
#include "rabcl/component/bno055.hpp"
#include "rabcl/controller/omni_drive.hpp"
#include "rabcl/controller/pd_gravity_ff.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t control_count = 0;
uint16_t imu_count = 0;
uint16_t omni_count = 0;
static const int32_t YAW_OFFSET = 24000;   // [0.01 deg] motor zero → robot zero
static const int32_t PITCH_OFFSET = 5000;  // [0.01 deg] motor zero → robot zero

char printf_buf[100];
uint8_t uart_led_count = 0;


uint8_t can_tx_idx = 0;
static const uint8_t CAN_MOTOR_COUNT = 6;
const uint32_t can_motor_ids[CAN_MOTOR_COUNT] = {
  (uint32_t)rabcl::CAN_ID::CHASSIS_FRONT_RIGHT_TX + 0x200,
  (uint32_t)rabcl::CAN_ID::CHASSIS_FRONT_LEFT_TX  + 0x200,
  (uint32_t)rabcl::CAN_ID::CHASSIS_BACK_RIGHT_TX  + 0x200,
  (uint32_t)rabcl::CAN_ID::CHASSIS_BACK_LEFT_TX   + 0x200,
  (uint32_t)rabcl::CAN_ID::YAW_TX,
  (uint32_t)rabcl::CAN_ID::PITCH_TX,
};
uint8_t can_motor_data[CAN_MOTOR_COUNT][8];

uint16_t dm_enable_count = 0;
uint8_t dm_enable_remain = 0;

rabcl::Info robot_data;
rabcl::Uart* uart;
rabcl::BNO055 bno055;
rabcl::OmniDrive omni_drive(0.06, 0.28);
rabcl::PdGravityFf yaw_pd(5000.0f, 600.0f, 0.0f, 2000.0f);
rabcl::PdGravityFf pitch_pd(4300.0f, 215.0f, -600.0f, 2000.0f);

float imu_rad_init = 0.0f;
float imu_rad_sum = 0.0f;
uint8_t pre_chassis_mode = 0;

// FF odometry: integrate yaw command while chassis_mode == 1 (IMU-free spin compensation)
float yaw_ff = 0.0f;

float spin_speed_min = 2.5f;
float spin_speed_max = 5.0f;
float spin_speed = 0.0f;
float spin_t = 0.0f;  // phase accumulator, wraps at 8*PI

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// I2C bus recovery: release any slave holding SCL/SDA from a prior session
// by manually toggling SCL 9 times + manual STOP, then re-init the peripheral.
static void I2cBusRecovery()
{
  HAL_I2C_DeInit(&hi2c1);

  GPIO_InitTypeDef g = {0};
  g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  for (volatile int i = 0; i < 400; i++) { __NOP(); }

  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    for (volatile int j = 0; j < 400; j++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile int j = 0; j < 400; j++) { __NOP(); }
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  for (volatile int j = 0; j < 400; j++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  for (volatile int j = 0; j < 400; j++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  for (volatile int j = 0; j < 400; j++) { __NOP(); }

  MX_I2C1_Init();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    // ---imu: DISABLED until BNO055 hardware is verified
#if 0
    imu_count++;
    if (imu_count >= 15)
    {
      imu_count = 0;
      uint8_t buf[2];
      if (HAL_I2C_Mem_Read(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::EULER_H_LSB, 1, buf, 2, HAL_MAX_DELAY) == HAL_OK)
      {
        int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);
        robot_data.imu_.euler_heading_ = raw * (1.0f / 16.0f) * (M_PI / 180.0f);
      }
    }
#endif

    // ---control (100Hz)
    control_count++;
    if (control_count >= 15)
    {
      control_count = 0;

      // --- load motor
      if (robot_data.load_mode_ == 1)
      {
        HAL_GPIO_WritePin(LOAD_MOTOR_PAHSE_GPIO_Port, LOAD_MOTOR_PAHSE_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)500); // ~999
      }
      else if (robot_data.load_mode_ == 2)
      {
        HAL_GPIO_WritePin(LOAD_MOTOR_PAHSE_GPIO_Port, LOAD_MOTOR_PAHSE_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)500); // ~999
      }
      else
      {
        HAL_GPIO_WritePin(LOAD_MOTOR_PAHSE_GPIO_Port, LOAD_MOTOR_PAHSE_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)0);
      }

      // ---fire motor
      if (robot_data.fire_mode_ == 1)
      {
        // right
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)1200); // 1400 2025/3/2 test in robosupo lab
        // left
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)1200);
      }
      else
      {
        // right
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)1000);
        // left
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)1000);
      }

      // ---feedback TX (100Hz)
      uart->PrepareFeedbackPacket(robot_data);
      HAL_UART_Transmit_IT(&huart2,
          uart->feedback_transmit_buffer_,
          rabcl::Uart::FEEDBACK_PACKET_SIZE);
    }

    // ---motor commands (500Hz)
    omni_count++;
    if (omni_count >= 3)
    {
      omni_count = 0;
#if 0
      // --- infinite rotation mode: IMU heading tracking (DISABLED)
      float imu_heading = robot_data.imu_.euler_heading_;
      if (pre_chassis_mode == 0 && robot_data.chassis_mode_ == 1)
      {
        imu_rad_init = imu_heading;
      }
      if (pre_chassis_mode == 1 && robot_data.chassis_mode_ == 0)
      {
        imu_rad_sum += (imu_heading - imu_rad_init);
      }
      pre_chassis_mode = robot_data.chassis_mode_;
#endif

      // --- chassis: omni drive from RasPi commands
      float chassis_rot_z = robot_data.chassis_vel_z_;
      if (robot_data.chassis_mode_ == 1)
      {
        // smooth random-like spin (no reverse): v = v_min + v_max * (((sin t + sin t/2 + sin t/4)/3 + 1) * 0.5), t ∈ [0, 8π), ~10s/cycle
        spin_t += 8.0f * static_cast<float>(M_PI) / 5000.0f;  // 500Hz * 10s = 5000 ticks per 8π
        if (spin_t >= 8.0f * static_cast<float>(M_PI)) {
          spin_t -= 8.0f * static_cast<float>(M_PI);
        }
        float phase = ((sinf(spin_t) + sinf(spin_t * 0.5f) + sinf(spin_t * 0.25f)) * (1.0f / 3.0f) + 1.0f) * 0.5f;
        spin_speed = spin_speed_min + (spin_speed_max - spin_speed_min) * phase;
        chassis_rot_z += spin_speed;
        yaw_ff += chassis_rot_z * 0.002f;  // 500Hz -> dt = 2ms; FF yaw odometry
      }
      double chassis_vel_cmd[4];
      omni_drive.CalcVel(
        robot_data.chassis_vel_x_, robot_data.chassis_vel_y_, chassis_rot_z,
        chassis_vel_cmd[0], chassis_vel_cmd[1], chassis_vel_cmd[2], chassis_vel_cmd[3],
        robot_data.yaw_act_.position_);
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[0], can_motor_data[0]);  // FR
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[1], can_motor_data[1]);  // FL
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[2], can_motor_data[2]);  // BR
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[3], can_motor_data[3]);  // BL

      // --- YAW: torque command with PD control (single-turn angular)
      {
        float target_rad = static_cast<float>(robot_data.yaw_pos_) + yaw_ff;
        float torque = yaw_pd.CalcAngular(
          target_rad,
          robot_data.yaw_act_.position_,
          robot_data.yaw_act_.velocity_);
        rabcl::Can::PrepareLKMotorTorqueCmd(static_cast<int16_t>(torque), can_motor_data[4]);
      }
      // --- PITCH: torque command with PD control
      {
        float torque = pitch_pd.CalcAngular(
          static_cast<float>(robot_data.pitch_pos_),
          robot_data.pitch_act_.position_,
          robot_data.pitch_act_.velocity_);
        rabcl::Can::PrepareLKMotorTorqueCmd(static_cast<int16_t>(torque), can_motor_data[5]);
      }

      // --- UART DMA watchdog: restart if stuck
      if (huart2.RxState == HAL_UART_STATE_READY || huart2.ErrorCode != HAL_UART_ERROR_NONE)
      {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
        huart2.ErrorCode = HAL_UART_ERROR_NONE;
        huart2.RxState = HAL_UART_STATE_READY;
        HAL_UART_Receive_DMA(&huart2, uart->reference_receive_buffer_, rabcl::Uart::REFERENCE_PACKET_SIZE);
      }
    }

    // ---DM2325 periodic enable (0.5Hz)
    dm_enable_count++;
    if (dm_enable_count >= 750)
    {
      dm_enable_count = 0;
      dm_enable_remain = 4;
    }

    // ---CAN TX sequential (1500Hz, 250Hz/motor)
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
      CAN_TxHeaderTypeDef TxHeader;
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.IDE = CAN_ID_STD;
      TxHeader.DLC = 8;
      TxHeader.TransmitGlobalTime = DISABLE;
      uint32_t TxMailbox;

      TxHeader.StdId = can_motor_ids[can_tx_idx];
      uint8_t * tx_data = can_motor_data[can_tx_idx];

      // DM motor (index 0..3): periodic enable
      uint8_t enable_data[8];
      if (dm_enable_remain > 0 && can_tx_idx < 4)
      {
        rabcl::Can::PrepareDMMotorEnable(enable_data);
        tx_data = enable_data;
        TxHeader.StdId = (uint32_t)rabcl::CAN_ID::CHASSIS_FRONT_RIGHT_TX + can_tx_idx;
        dm_enable_remain--;
      }

      HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_data, &TxMailbox);
      can_tx_idx = (can_tx_idx + 1) % CAN_MOTOR_COUNT;
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    if (rabcl::Can::UpdateData(RxHeader.StdId, RxData, robot_data,
          YAW_OFFSET * (static_cast<float>(M_PI) / 18000.0f),
          PITCH_OFFSET * (static_cast<float>(M_PI) / 18000.0f)))
    {
      // DM2325: invert feedback to match robot coordinate system
      uint32_t id = RxHeader.StdId;
      if (id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_FRONT_RIGHT_RX) ||
          id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_FRONT_LEFT_RX) ||
          id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_BACK_RIGHT_RX) ||
          id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_BACK_LEFT_RX))
      {
        rabcl::MotorInfo * m = nullptr;
        if (id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_FRONT_RIGHT_RX)) m = &robot_data.chassis_fr_act_;
        else if (id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_FRONT_LEFT_RX)) m = &robot_data.chassis_fl_act_;
        else if (id == static_cast<uint32_t>(rabcl::CAN_ID::CHASSIS_BACK_RIGHT_RX)) m = &robot_data.chassis_br_act_;
        else m = &robot_data.chassis_bl_act_;
        m->position_ = -m->position_;
        m->velocity_ = -m->velocity_;
        m->torque_ = -m->torque_;
      }
      // HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart2) return;

  auto result = uart->HandleRxComplete(robot_data);
  if (result.data_updated)
  {
    uart_led_count++;
    if (uart_led_count >= 50)
    {
      uart_led_count = 0;
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
  HAL_UART_Receive_DMA(&huart2, result.next_rx_buf, result.next_rx_size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
    huart2.ErrorCode = HAL_UART_ERROR_NONE;
    uart->HandleRxError();
    HAL_UART_Receive_DMA(&huart2, uart->reference_receive_buffer_, rabcl::Uart::REFERENCE_PACKET_SIZE);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uart = new rabcl::Uart();

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // ---init indicator: LED ON during initialization
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

  // ---ESC calibration
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)2000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)2000);
  HAL_Delay(3000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)1000);
  HAL_Delay(3000);

  // ---init BNO055: DISABLED until hardware is verified
#if 0
  // 1) release any physically stuck SCL/SDA from a prior session
  I2cBusRecovery();
  HAL_Delay(100);
  // 2) SYS_TRIGGER (0x3F) bit 5 = system reset; blind write
  uint8_t reset_cmd = 0x20;
  HAL_I2C_Mem_Write(&hi2c1, rabcl::BNO055::I2C_ADDR, 0x3F, 1, &reset_cmd, 1, 100);
  HAL_Delay(700);  // BNO055 internal boot time
  // 3) configure to NDOF mode
  uint8_t bno_mode = rabcl::BNO055::MODE_CONFIG;
  HAL_I2C_Mem_Write(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::OPR_MODE_ADDR, 1, &bno_mode, 1, 100);
  HAL_Delay(20);
  bno_mode = rabcl::BNO055::MODE_NDOF;
  HAL_I2C_Mem_Write(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::OPR_MODE_ADDR, 1, &bno_mode, 1, 100);
  HAL_Delay(20);
#endif

  // ---start PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // ---CAN start
  HAL_CAN_Start(&hcan);
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  // ---clear UART error flags accumulated during init delay
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);

  // ---random seed
  rabcl::Utils::SetRandomSeed(HAL_GetTick());

  // ---start interrupt processing
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_DMA(&huart2, uart->reference_receive_buffer_, rabcl::Uart::REFERENCE_PACKET_SIZE);

  // ---init complete: LED OFF
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
