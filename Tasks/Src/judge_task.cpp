#include "judge_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdbool.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
extern "C"{
//#include "pm01.h"
#include "ina226.h"
}
extern DMA_HandleTypeDef hdma_usart1_rx;

/*����ϵͳ�������������ݴ�������*/
uint8_t Judge_Buffer[JUDGE_BUFFER_LEN] = {0};
uint8_t usart1_rx_flag;
ina226_t ina226_data;
void Judge_Task(void const * argumt)
{
	  usart1_init();
	  
		vTaskDelay(20);
//	  uint32_t currentTime;
		while(1)
		{
//		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��			
		Judge_Read_Data(Judge_Buffer);		//��ȡ����ϵͳ����	

		osDelay(2);
		//pm01_access_poll();	
//		vTaskDelayUntil(&currentTime, 50);
		}
}

extern "C"
{
/*����1�жϺ���*/
void USART1_IRQHandler(void)
{
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
		}	
	  __HAL_DMA_DISABLE(huart1.hdmarx);
	

		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);

    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, JUDGE_BUFFER_LEN);

		__HAL_DMA_ENABLE(huart1.hdmarx);
		HAL_UART_Receive_DMA(&huart1,Judge_Buffer,JUDGE_BUFFER_LEN);		

}

}


