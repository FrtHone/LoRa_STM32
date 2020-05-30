/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lcd.h"
#include "gui.h"
#include "SX127X_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum DemoInternalStates
{
  APP_RNG = 0, // nothing to do (or wait a radio interrupt)
  RX_DONE,
  TX_DONE,
  TX_ING,
  APP_IDLE,
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
uint8_t TXbuffer[8] = {0};
uint8_t RXbuffer[8] = {0};
uint8_t colorcnt=0;
uint32_t Fre[5] = {470800000, 494600000, 510000000, 868000000, 915000000};
int16_t SNR = 0;
int16_t RSSI = 0;
int16_t ENV_RSSI = 0;

uint8_t communication_states;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MX_TIM11_Init_Ms(uint16_t time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  G_LoRaConfig.LoRa_Freq = Fre[0];      //中心频率470MHz
  G_LoRaConfig.BandWidth = BW125KHZ;    //BW = 125KHz  BW125KHZ
  G_LoRaConfig.SpreadingFactor = SF09;  //SF = 9
  G_LoRaConfig.CodingRate = CR_4_6;     //CR = 4/6
  G_LoRaConfig.PowerCfig = 15;          //19±1dBm
  G_LoRaConfig.MaxPowerOn = true;       //最大功率开启
  G_LoRaConfig.CRCON = true;            //CRC校验开启
  G_LoRaConfig.ExplicitHeaderOn = true; //Header开启
  G_LoRaConfig.PayloadLength = 8;       //数据包长度
  
  LCD_Init();
  while(1)
  {
    if(SX127X_Lora_init() == NORMAL)      //无线模块初始化
    {
      break;
    }
    Show_Str(0, 240, WHITE, BLUE, "ERROR", 16, 1);
  }
  
  Show_Str(192, 224, BLACK, WHITE, "MASTER", 16, 0);
  Show_Str(0, 40, BLACK, WHITE, "RH:  0.0, TEMP:  0.0", 16, 0);
  Show_Str(0, 56, BLACK, WHITE, "G:   0, R:   0, B:   0", 16, 0);
  Show_Str(0, 72, BLACK, WHITE, "SNR:   0, RSSI:   0", 16, 0);
  Show_Str(0, 88, BLACK, WHITE, "ENV_RSSI: 0", 16, 0);
  Show_Str(0, 104, BLACK, WHITE, "Lock Status:", 16, 0);
//  HAL_TIM_Base_Stop_IT(&htim11);
//  MX_TIM11_Init_Ms(5000);
//  HAL_TIM_Base_Start_IT(&htim11);//定时2s开启
  DIO0_EnableInterrupt();
  SX127X_StartRx();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch(communication_states)
    {
      case APP_IDLE:
        break;
      
      case TX_ING:
        HAL_Delay(50);
        DIO0_EnableInterrupt();
        SX127X_TxPacket(TXbuffer);
        communication_states = APP_IDLE;
        break;
      
      case TX_DONE:
        SX127X_StandbyMode();
        DIO0_EnableInterrupt();
        SX127X_StartRx();
        communication_states = APP_IDLE;
        break;
      
      case RX_DONE:
        DIO0_DisableInterrupt();
        SX127X_RxPacket(RXbuffer);
        RSSI = G_LoRaPara.Packet_RSSI;
        SNR = G_LoRaPara.Packet_SNR;
        ENV_RSSI = G_LoRaPara.Current_RSSI;
        HAL_Delay(50);
        if(SNR<0)
        {
          LCD_ShowChar(32, 72, BLACK, WHITE, '-', 16, 0);
          SNR=-SNR;
        }
        else
        {
          LCD_ShowChar(32, 72, BLACK, WHITE, ' ', 16, 0);
        }
        if(RSSI<0)
        {
          LCD_ShowChar(120, 72, BLACK, WHITE, '-', 16, 0);
          RSSI=-RSSI;
        }
        else
        {
          LCD_ShowChar(120, 72, BLACK, WHITE, ' ', 16, 0);
        }
        if(ENV_RSSI<0)
        {
          LCD_ShowChar(72, 88, BLACK, WHITE, '-', 16, 0);
          ENV_RSSI=-ENV_RSSI;
        }
        else
        {
          LCD_ShowChar(72, 88, BLACK, WHITE, ' ', 16, 0);
        }
        LCD_ShowNum(40, 72, SNR, 3, 16);
        LCD_ShowNum(128, 72, RSSI, 3, 16);
        LCD_ShowNum(80, 88, ENV_RSSI, 3, 16);
        
        if(RXbuffer[0]==0x01)
        {
          LCD_ShowNum(32, 40, RXbuffer[1], 2, 16);
          LCD_ShowNum(56, 40, RXbuffer[2], 1, 16);
          LCD_ShowNum(128, 40, RXbuffer[3], 2, 16);
          LCD_ShowNum(152, 40, RXbuffer[4], 1, 16);
        }
        else
        {
          if(RXbuffer[0]==0x02)
          {
          }
          else
          {
            if(RXbuffer[0]==0x03)
            {
              if(RXbuffer[1]==0x77)
              {
                Show_Str(104, 104, BLACK, WHITE, "Closed", 16, 0);
              }
              else
              {
                Show_Str(104, 104, BLACK, WHITE, "Open  ", 16, 0);
              }
            }
          }
        }
        
        communication_states = TX_DONE;
        break;
      
      default:
        break;
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 32-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB3 PB4 PB5 
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void MX_TIM11_Init_Ms(uint16_t time)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 32000-1;   
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = time-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    return;
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim11, &sClockSourceConfig) != HAL_OK)
  {
    return;
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim11, &sMasterConfig) != HAL_OK)
  {
    return;
  }
  
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_13||GPIO_Pin==GPIO_PIN_14||GPIO_Pin==GPIO_PIN_15)
  {
    if(DIO0_GetState() == GPIO_PIN_SET)
    {
      uint8_t flag;
      SX127X_Read(REG_LR_IRQFLAGS, &flag);
      SX127X_Write(REG_LR_IRQFLAGS, 0xff); //clear flags
      if(flag & RFLR_IRQFLAGS_TXDONE)
      {
        communication_states = TX_DONE;
      }
      else
      {
        if(flag & RFLR_IRQFLAGS_RXDONE)
        {
          communication_states = RX_DONE;
        }
      }
    }
  }
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==GPIO_PIN_RESET)
  {
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==GPIO_PIN_RESET)
    {
      TXbuffer[0]=0x02;
      if(colorcnt==0)
      {
        TXbuffer[1]=0xff;
        TXbuffer[2]=0xff;
        TXbuffer[3]=0xff;
        colorcnt++;
      }
      else
      {
        if(colorcnt==1)
        {
          TXbuffer[1]=0x00;
          TXbuffer[2]=0xff;
          TXbuffer[3]=0x00;
          colorcnt++;
        }
        else
        {
          if(colorcnt==2)
          {
            TXbuffer[1]=0xff;
            TXbuffer[2]=0x00;
            TXbuffer[3]=0x00;
            colorcnt++;
          }
          else
          {
            if(colorcnt==3)
            {
              TXbuffer[1]=0x00;
              TXbuffer[2]=0x00;
              TXbuffer[3]=0xff;
              colorcnt++;
            }
            else
            {
              TXbuffer[1]=0x00;
              TXbuffer[2]=0x00;
              TXbuffer[3]=0x00;
              colorcnt=0;
            }
          }
        }
      }
      LCD_ShowNum(24, 56, TXbuffer[1], 3, 16);
      LCD_ShowNum(88, 56, TXbuffer[2], 3, 16);
      LCD_ShowNum(152, 56, TXbuffer[3], 3, 16);
      communication_states = TX_ING;
    }
  }
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==GPIO_PIN_RESET)
  {
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==GPIO_PIN_RESET)
    {
      TXbuffer[0]=0x03;
      TXbuffer[2]=0xff;
      communication_states = TX_ING;
    }
  }
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==GPIO_PIN_RESET)
  {
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==GPIO_PIN_RESET)
    {
      TXbuffer[0]=0x01;
      TXbuffer[5]=0xff;
      communication_states = TX_ING;
    }
  }
//  if(GPIO_Pin==GPIO_PIN_11)
//  {
//  }
//  if(GPIO_Pin==GPIO_PIN_12)
//  {
//  }
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  communication_states = TX_ING;
//  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
