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
uint16_t can_count = 0;
int32_t tmp_output = 3000;
char printf_buf[100];

rabcl::Info robot_data;
rabcl::Uart* uart;

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
    control_count++;
    if (control_count >= 10) // 100Hz
    {
      control_count = 0;
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
    }

    // ---can
    can_count++;
    if (can_count >= 2) // 500Hz
    {
      tmp_output+=1;
      if (tmp_output > 8000) {
        tmp_output = 3000;
      }
      can_count = 0;
      CAN_TxHeaderTypeDef TxHeader;
      TxHeader.RTR = CAN_RTR_DATA;
      TxHeader.IDE = CAN_ID_STD;
      TxHeader.DLC = 8;
      TxHeader.TransmitGlobalTime = DISABLE;
      uint32_t TxMailbox;
      uint8_t TxData[8];
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
      {
        TxHeader.StdId = (uint32_t)rabcl::CAN_ID::PITCH_TX;
        rabcl::Can::PrepareLKMotorPositionCmd(tmp_output, 1500, TxData);
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
      }
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
      {
        TxHeader.StdId = (uint32_t)rabcl::CAN_ID::YAW_TX;
        rabcl::Can::PrepareLKMotorPositionCmd(tmp_output, 300, TxData);
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
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
    if (RxData[0] == 0xC0)
    {
      uint16_t kp = (uint16_t)(((uint16_t)RxData[3] << 8) | RxData[2]);
      uint16_t ki = (uint16_t)(((uint16_t)RxData[5] << 8) | RxData[4]);
      uint16_t kd = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
      snprintf(printf_buf, 100, "pid param_id=%#x: kp=%u ki=%u kd=%u\n", RxData[1], kp, ki, kd);
      HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
    }
    else if (rabcl::Can::UpdateData(RxHeader.StdId, RxData, robot_data))
    {
      if (RxHeader.StdId == (uint32_t)rabcl::CAN_ID::PITCH_RX)
      {
        snprintf(printf_buf, 100, "pitch: pos=%.3f vel=%.3f cur=%.3f tmp=%.1f\n",
          robot_data.pitch_act_.position_,
          robot_data.pitch_act_.velocity_,
          robot_data.pitch_act_.current_,
          robot_data.pitch_act_.temperature_);
        HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      }
      else if (RxHeader.StdId == (uint32_t)rabcl::CAN_ID::YAW_RX)
      {
        snprintf(printf_buf, 100, "yaw:   pos=%.3f vel=%.3f cur=%.3f tmp=%.1f\n",
          robot_data.yaw_act_.position_,
          robot_data.yaw_act_.velocity_,
          robot_data.yaw_act_.current_,
          robot_data.yaw_act_.temperature_);
        HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      }
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

  // ---start interrupt processing
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_CAN_Start(&hcan);
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Receive_DMA(&huart2, uart->uart_receive_buffer_, 8);

  // ---read YAW motor angle PID params
  {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    TxHeader.StdId = (uint32_t)rabcl::CAN_ID::YAW_TX;
    rabcl::Can::PrepareLKMotorReadParam(0x0A, TxData);
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  }

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
