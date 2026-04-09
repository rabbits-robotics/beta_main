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

enum class ImuState : uint8_t
{
  IDLE,
  READING_ACCEL,
  READING_GYRO,
  READING_EULER,
};
volatile ImuState imu_state = ImuState::IDLE;
uint8_t imu_buf[6];

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

rabcl::Info robot_data;
rabcl::Uart* uart;
rabcl::BNO055 bno055;
rabcl::OmniDrive omni_drive(0.06, 0.28);
rabcl::PdGravityFf yaw_pd(5000.0f, 600.0f, 0.0f, 2000.0f);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    // ---imu (500Hz)
    imu_count++;
    if (imu_count >= 3)
    {
      imu_count = 0;
      if (imu_state == ImuState::IDLE)
      {
        imu_state = ImuState::READING_ACCEL;
        HAL_I2C_Mem_Read_IT(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::ACC_DATA_X_LSB, 1, imu_buf, 6);
      }
    }

    // ---control (100Hz)
    control_count++;
    if (control_count >= 15)
    {
      control_count = 0;

      // --- load motor
      if (robot_data.load_mode_ == 1)
      {
        HAL_GPIO_WritePin(LOAD_MOTOR_PAHSE_GPIO_Port, LOAD_MOTOR_PAHSE_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)140);
      }
      else if (robot_data.load_mode_ == 2)
      {
        HAL_GPIO_WritePin(LOAD_MOTOR_PAHSE_GPIO_Port, LOAD_MOTOR_PAHSE_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)140);
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
    }

    // ---motor commands (500Hz)
    omni_count++;
    if (omni_count >= 3)
    {
      omni_count = 0;
      // --- chassis: omni drive from RasPi commands
      double chassis_vel_cmd[4];
      omni_drive.CalcVel(
        robot_data.chassis_vel_x_, robot_data.chassis_vel_y_, robot_data.chassis_vel_z_,
        chassis_vel_cmd[0], chassis_vel_cmd[1], chassis_vel_cmd[2], chassis_vel_cmd[3],
        robot_data.yaw_act_.position_);
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[0], can_motor_data[0]);  // FR
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[1], can_motor_data[1]);  // FL
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[2], can_motor_data[2]);  // BR
      rabcl::Can::PrepareDMMotorVelocityCmd(-(float)chassis_vel_cmd[3], can_motor_data[3]);  // BL

      // --- YAW: torque command with PD control (single-turn angular)
      {
        float target_rad = static_cast<float>(robot_data.yaw_pos_);
        float torque = yaw_pd.CalcAngular(
          target_rad,
          robot_data.yaw_act_.position_,
          robot_data.yaw_act_.velocity_);
        rabcl::Can::PrepareLKMotorTorqueCmd(static_cast<int16_t>(torque), can_motor_data[4]);
      }
      // --- PITCH: position command
      {
        int32_t pitch_cmd = static_cast<int32_t>(robot_data.pitch_pos_ * 5729.578f) + PITCH_OFFSET;
        rabcl::Can::PrepareLKMotorPositionCmd(pitch_cmd, 400, can_motor_data[5]);
      }
    }

    // ---CAN TX round-robin (1500Hz, 500Hz/motor)
    {
      CAN_TxHeaderTypeDef TxHeader;
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.IDE = CAN_ID_STD;
      TxHeader.DLC = 8;
      TxHeader.TransmitGlobalTime = DISABLE;
      uint32_t TxMailbox;
      uint8_t free_mb = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
      for (uint8_t i = 0; i < free_mb; i++)
      {
        TxHeader.StdId = can_motor_ids[can_tx_idx];
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, can_motor_data[can_tx_idx], &TxMailbox);
        can_tx_idx = (can_tx_idx + 1) % CAN_MOTOR_COUNT;
      }
    }
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != &hi2c1) return;

  switch (imu_state)
  {
    case ImuState::READING_ACCEL:
      bno055.UpdateAccel(imu_buf, robot_data.imu_);
      imu_state = ImuState::READING_GYRO;
      HAL_I2C_Mem_Read_IT(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::GYR_DATA_X_LSB, 1, imu_buf, 6);
      break;
    case ImuState::READING_GYRO:
      bno055.UpdateGyro(imu_buf, robot_data.imu_);
      imu_state = ImuState::READING_EULER;
      HAL_I2C_Mem_Read_IT(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::EULER_H_LSB, 1, imu_buf, 6);
      break;
    case ImuState::READING_EULER:
      bno055.UpdateEuler(imu_buf, robot_data.imu_);
      imu_state = ImuState::IDLE;
      break;
    default:
      imu_state = ImuState::IDLE;
      break;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    imu_state = ImuState::IDLE;
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
    HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, rabcl::Uart::PACKET_SIZE);
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
  // ---ESC calibration
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)2000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)2000);
  HAL_Delay(3000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)1000);
  HAL_Delay(3000);

  // ---init BNO055
  HAL_Delay(1000);
  uint8_t bno_id = 0;
  HAL_I2C_Mem_Read(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::CHIP_ID_ADDR, 1, &bno_id, 1, HAL_MAX_DELAY);
  uint8_t bno_mode = rabcl::BNO055::MODE_CONFIG;
  HAL_I2C_Mem_Write(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::OPR_MODE_ADDR, 1, &bno_mode, 1, HAL_MAX_DELAY);
  HAL_Delay(20);
  bno_mode = rabcl::BNO055::MODE_NDOF;
  HAL_I2C_Mem_Write(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::OPR_MODE_ADDR, 1, &bno_mode, 1, HAL_MAX_DELAY);
  HAL_Delay(20);

  // ---start PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // ---CAN start
  HAL_CAN_Start(&hcan);
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  // ---write & read back YAW motor PID params
  {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::YAW_TX;
    rabcl::Can::PrepareLKMotorWritePID(0x0A, 200, 0, 50, TxData);  // angle: kp=200 ki=0 kd=50
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    rabcl::Can::PrepareLKMotorWritePID(0x0B, 300, 0, 0, TxData);  // speed: kp=300 ki=0 kd=0
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    rabcl::Can::PrepareLKMotorReadParam(0x0B, TxData);
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
  }

  // ---read PITCH motor PID params
  {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::PITCH_TX;
    rabcl::Can::PrepareRMDMotorReadPID(TxData);
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    rabcl::Can::PrepareRMDMotorWritePIDToRAM(250, 100, 250, 0, 250, 0, TxData); // curr(250,100) speed(250,0) pos(250,0)
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    rabcl::Can::PrepareRMDMotorReadPID(TxData);
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
  }

  // ---enable DM motors
  {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    rabcl::Can::PrepareDMMotorEnable(TxData);
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::CHASSIS_FRONT_RIGHT_TX;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::CHASSIS_FRONT_LEFT_TX;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::CHASSIS_BACK_RIGHT_TX;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::CHASSIS_BACK_LEFT_TX;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  }

  // ---start interrupt processing
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, rabcl::Uart::PACKET_SIZE);

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
