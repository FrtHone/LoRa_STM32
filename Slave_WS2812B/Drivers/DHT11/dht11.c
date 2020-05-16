#include "dht11.h"

void delay(uint16_t time)
{
	/* change your code here for the delay in microseconds */
//	__HAL_TIM_ENABLE(&htim11);
	HAL_TIM_Base_Start(&TIMERUSED);
	__HAL_TIM_SET_COUNTER(&TIMERUSED, 0);
	while ((__HAL_TIM_GET_COUNTER(&TIMERUSED))<time);
	HAL_TIM_Base_Stop(&TIMERUSED);
//	__HAL_TIM_DISABLE(&htim11);
}

//void delay(uint32_t value)
//{
//	uint32_t i;
//	i = value * 8;
//	while(i--);
//}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_start(void)
{
	Set_Pin_Output(DATA_Port, DATA_Pin);
	HAL_GPIO_WritePin(DATA_Port, DATA_Pin, GPIO_PIN_RESET);
	HAL_Delay(18);
	HAL_GPIO_WritePin(DATA_Port, DATA_Pin, GPIO_PIN_SET);
	Set_Pin_Input(DATA_Port, DATA_Pin);
}

uint8_t Check_Response (void)
{
	delay (40);
	if (!(HAL_GPIO_ReadPin (DATA_Port, DATA_Pin)))
	{
		while(!HAL_GPIO_ReadPin (DATA_Port, DATA_Pin));
		while(HAL_GPIO_ReadPin (DATA_Port, DATA_Pin));
		return 1;
//		delay (80);
//		if ((HAL_GPIO_ReadPin (DATA_Port, DATA_Pin))) Response = 1;
//		else Response = 0;
	}
	else
	{
		return 0;
	}
//	while ((HAL_GPIO_ReadPin (DATA_Port, DATA_Pin)));   // wait for the pin to go low

//	return Response;
}

uint8_t Byte_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DATA_Port, DATA_Pin)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DATA_Port, DATA_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else
		{
			while ((HAL_GPIO_ReadPin (DATA_Port, DATA_Pin)));
			i|= (1<<(7-j));  // if the pin is high, write 1
		}
	}
	return i;
}

uint8_t Read_DHT11(DHT11_Data_TypeDef *DHT11_Data)
{
	DHT11_start();
	
	/*判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行*/   
	if(Check_Response()==1)     
	{
		/*开始接收数据*/   
		DHT11_Data->Rh_int= Byte_Read();

		DHT11_Data->Rh_deci= Byte_Read();

		DHT11_Data->temp_int= Byte_Read();

		DHT11_Data->temp_deci= Byte_Read();

		DHT11_Data->check_sum= Byte_Read();


		/*读取结束，引脚改为输出模式*/
		Set_Pin_Output(DATA_Port, DATA_Pin);
		/*主机拉高*/
		HAL_GPIO_WritePin(DATA_Port, DATA_Pin, GPIO_PIN_SET);

		/*检查读取的数据是否正确*/
		if(DHT11_Data->check_sum == DHT11_Data->Rh_int + DHT11_Data->Rh_deci + DHT11_Data->temp_int+ DHT11_Data->temp_deci)
			return SUCCESS;
		else 
			return ERROR;
	}
	else
	{
		return ERROR;
	}
}
