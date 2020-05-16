#ifndef __DHT11_H
#define	__DHT11_H

#include "stm32l1xx_hal.h"
#include "main.h"

//Defines the port and the timer used.
//Change it to the one you uses.
#define DATA_Pin GPIO_PIN_9
#define DATA_Port GPIOB
#define TIMERUSED htim10

extern TIM_HandleTypeDef TIMERUSED;

typedef struct
{
	uint8_t  Rh_int;		//湿度的整数部分
	uint8_t  Rh_deci;	 	//湿度的小数部分
	uint8_t  temp_int;	 	//温度的整数部分
	uint8_t  temp_deci;	 	//温度的小数部分
	uint8_t  check_sum;	 	//校验和
		                 
}DHT11_Data_TypeDef;

void DHT11_start(void);
uint8_t Check_Response (void);
uint8_t Byte_Read (void);
uint8_t Read_DHT11(DHT11_Data_TypeDef *DHT11_Data);

#endif /* __DHT11_H */
