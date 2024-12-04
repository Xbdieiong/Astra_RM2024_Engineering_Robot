#include "bsp_gpio.h"


void gpio_set(uint8_t state)
{
    GPIO_PinState flag = GPIO_PIN_RESET;
    if(state)
    {
        flag = GPIO_PIN_SET;
    }
		

     HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, flag);


				
    
}





