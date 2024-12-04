#include "led_flow_task.h"
#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "main.h"
#include "bsp_servo_pwm.h"

//最大最小的PWM值
#define SERVO_MIN_PWM   250
#define SERVO_MAX_PWM   1250

uint8_t head_state = 0;
uint8_t turn_state = 0;
uint8_t front_state = 0;
//uint8_t beng_state = 0;

//定义舵机控制变量
uint16_t servo_pwm[2] = {750, 750};

void led_RGB_flow_task(void const * argument)
{
		//这里设置io变化
		const RC_ctrl_t *RC;               		//遥控器指针
		RC = get_remote_control_point();
		while(1)
		{
			if(RC->rc.ch[1]>10)
				{
						//遥控器键盘控制
						for(int i=0;i<2;i++)
						{
							servo_pwm[i] -= 30;
						}
				}
				else if(RC->rc.ch[1]<-10)
				{
						//遥控器键盘控制
						for(int i=0;i<2;i++)
						{
							servo_pwm[i] += 30;
						}
				}
				//设置PWM
				for(int i = 0;i < 2;i++)
				{
						//限制幅度
						if(servo_pwm[i] < SERVO_MIN_PWM)
						{
								servo_pwm[i] = SERVO_MIN_PWM;
						}
						else if(servo_pwm[i] > SERVO_MAX_PWM)
						{
								servo_pwm[i] = SERVO_MAX_PWM;
						}
						servo_pwm_set(servo_pwm[i], i);
				}
				
				
				
				
				
			/*if((RC->key.v & KEY_PRESSED_OFFSET_Q)&&(RC->key.v & KEY_PRESSED_OFFSET_F))
			{
					head_state = !head_state;
					while((RC->key.v & KEY_PRESSED_OFFSET_Q)&&(RC->key.v & KEY_PRESSED_OFFSET_F))
					{
							osDelay(10);
					}
					gpio_set(1,head_state);
			}
			else if((RC->key.v & KEY_PRESSED_OFFSET_E)&&(RC->key.v & KEY_PRESSED_OFFSET_X))
			{
					turn_state = !turn_state;
					while((RC->key.v & KEY_PRESSED_OFFSET_E)&&(RC->key.v & KEY_PRESSED_OFFSET_X))
					{
							osDelay(10);
					}
					gpio_set(2,turn_state);
			}
			else if((RC->key.v & KEY_PRESSED_OFFSET_Q)&&(RC->key.v & KEY_PRESSED_OFFSET_R))
			{
					front_state = !front_state;
					while((RC->key.v & KEY_PRESSED_OFFSET_Q)&&(RC->key.v & KEY_PRESSED_OFFSET_R))
					{
							osDelay(10);
					}
					gpio_set(3,front_state);
			}
			else if(RC->rc.ch[2]>10)
			{
					beng_state = !beng_state;
					while(RC->rc.ch[2]>10)
					{
							osDelay(10);
					}
					gpio_set(4,beng_state);
			}*/
			osDelay(200);
	}
}



