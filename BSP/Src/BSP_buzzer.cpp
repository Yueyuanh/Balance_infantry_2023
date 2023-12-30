#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


void buzzer_time(int times)
{
		for(int i;i<times;i++)
		{
				buzzer_on(95,10000);
				HAL_Delay(200);
				buzzer_off();
				HAL_Delay(200);
		}
		buzzer_off();
}

void buzzer_upper()
{
		buzzer_on(100,10000);
		HAL_Delay(1000);
	  buzzer_off();
}