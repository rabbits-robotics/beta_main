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
#include "rabcl/interface/uart.hpp"
#include "rabcl/interface/can.hpp"
#include "rabcl/component/bno055.hpp"

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
int32_t tmp_output = 3000;
char printf_buf[100];

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
    // ---control (100Hz)
    control_count++;
    if (control_count >= 15)
    {
      control_count = 0;

      // ---imu
      uint8_t imu_buf[6];
      HAL_I2C_Mem_Read(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::ACC_DATA_X_LSB, 1, imu_buf, 6, HAL_MAX_DELAY);
      bno055.UpdateAccel(imu_buf, robot_data.imu_);
      HAL_I2C_Mem_Read(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::GYR_DATA_X_LSB, 1, imu_buf, 6, HAL_MAX_DELAY);
      bno055.UpdateGyro(imu_buf, robot_data.imu_);
      HAL_I2C_Mem_Read(&hi2c1, rabcl::BNO055::I2C_ADDR, rabcl::BNO055::EULER_H_LSB, 1, imu_buf, 6, HAL_MAX_DELAY);
      bno055.UpdateEuler(imu_buf, robot_data.imu_);

      tmp_output += 3;
      if (tmp_output > 8000) { tmp_output = 3000; }
      HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, 8);

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

      // ---update CAN motor commands
      rabcl::Can::PrepareDMMotorVelocityCmd(1.0f, can_motor_data[0]);  // FR: 1 rad/s (output shaft)
      rabcl::Can::PrepareDMMotorVelocityCmd(0.0f, can_motor_data[1]);  // FL
      rabcl::Can::PrepareDMMotorVelocityCmd(0.0f, can_motor_data[2]);  // BR
      rabcl::Can::PrepareDMMotorVelocityCmd(0.0f, can_motor_data[3]);  // BL
      rabcl::Can::PrepareLKMotorPositionCmd(tmp_output, 3000, can_motor_data[4]);  // YAW
      rabcl::Can::PrepareLKMotorPositionCmd(tmp_output, 1500, can_motor_data[5]);  // PITCH
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    if (rabcl::Can::UpdateData(RxHeader.StdId, RxData, robot_data))
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    if (uart->UpdateData(robot_data))
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    else
    {
      snprintf(printf_buf, 100, "Failed to get uart info\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
    }
    HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, 8);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, 8);
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

  // ---ESC calibration
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)2000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)2000);
  HAL_Delay(3000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)1000);
  HAL_Delay(3000);

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
    rabcl::Can::PrepareLKMotorWritePID(0x0A, 20, 0, 5, TxData);  // angle: kp=20 ki=0 kd=5
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    rabcl::Can::PrepareLKMotorWritePID(0x0B, 300, 0, 0, TxData);  // speed: kp=300 ki=0 kd=0
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    HAL_Delay(10);
    rabcl::Can::PrepareLKMotorReadParam(0x0B, TxData);
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
  HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, 8);

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
