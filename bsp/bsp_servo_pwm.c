#include "bsp_servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
int b=1;

void servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch(i)
    {
        case 0:
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm);
					b=0;
        }break;
        case 1:
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm);
        }break;
        
    }
}
