#include "damiao_task.h"
#include "CAN_receive.h"
#include "dm4310_drv.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
//#include "can_driver.h"
//#include "key_modlue.h"
#include "string.h"
#include "bsp_gpio.h"

motor_t motor[num];

int8_t motor_id;

uint8_t beng_state = 0;

uint8_t kk=0,ss=0;

extern int8_t bishen;

const RC_ctrl_t *damiao_RC;

void damiao_task(void const *pvParameters)
{
   //程序暂停一段时间
    vTaskDelay(22000);
	  uint8_t damiao_biao=0;
    //电机初始化，设置ID、控制模式和命令模式
    dm4310_motor_init();
	  damiao_RC = get_remote_control_point();
	  //启用电机
	  for(motor_id=1;motor_id<5;motor_id++){
		ctrl_enable();
		}
    while (1)
    {
				if(damiao_biao==0){
					motor[Motor3].ctrl.pos_set=motor[Motor3].para.pos;
					motor[Motor4].ctrl.pos_set=motor[Motor4].para.pos;
					damiao_biao=1;
				}
				
				if(switch_is_up(damiao_RC->rc.s[1]))
				{
					for(motor_id=1;motor_id<5;motor_id++){
						ctrl_clear_err();
					}
					for(motor_id=1;motor_id<5;motor_id++){
						ctrl_enable();
					}
					motor[Motor3].ctrl.pos_set=motor[Motor3].para.pos;
					motor[Motor4].ctrl.pos_set=motor[Motor4].para.pos;
					ss=1;
					kk=0;
				}
				//发送电机控制数据
			  if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_Q)){
					motor[Motor1].ctrl.vel_set=3;
					motor[Motor1].ctrl.pos_set=0;
					motor[Motor1].ctrl.kp_set=0;
					motor[Motor2].ctrl.vel_set=3;	
					motor[Motor2].ctrl.pos_set=0;
					motor[Motor2].ctrl.kp_set=0;	
					kk=0;
        }
				else if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_A)){
					motor[Motor1].ctrl.vel_set=-3;
					motor[Motor1].ctrl.pos_set=0;
					motor[Motor1].ctrl.kp_set=0;
					motor[Motor2].ctrl.vel_set=-3;
					motor[Motor2].ctrl.pos_set=0;
					motor[Motor2].ctrl.kp_set=0;
					kk=0;
        }
				else if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_W)){
					motor[Motor1].ctrl.vel_set=2;
					motor[Motor1].ctrl.pos_set=0;
					motor[Motor1].ctrl.kp_set=0;
					motor[Motor2].ctrl.vel_set=-2;
					motor[Motor2].ctrl.pos_set=0;
					motor[Motor2].ctrl.kp_set=0;
					kk=0;
				}
				else if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_S)){
					motor[Motor1].ctrl.vel_set=-2;
					motor[Motor1].ctrl.pos_set=0;
					motor[Motor1].ctrl.kp_set=0;
					motor[Motor2].ctrl.vel_set=2;
					motor[Motor2].ctrl.pos_set=0;
					motor[Motor2].ctrl.kp_set=0;
					kk=0;
				}
				 else if(kk==0){
					motor[Motor1].ctrl.vel_set=0;
					motor[Motor1].ctrl.pos_set=motor[Motor1].para.pos;
					motor[Motor1].ctrl.kp_set=5;
					motor[Motor2].ctrl.vel_set=0;
					motor[Motor2].ctrl.pos_set=motor[Motor2].para.pos;
					motor[Motor2].ctrl.kp_set=5;
					kk=1;
				}
				ctrl_send();
				
				if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_E)){
					motor[Motor3].ctrl.pos_set+=0.1f;
						if(motor[Motor3].ctrl.pos_set>0.47f) motor[Motor3].ctrl.pos_set=0.47f;
					while(motor[Motor3].para.pos<motor[Motor3].ctrl.pos_set-0.02f){
						ctrl_send();
					}
        }
				else if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_D)){
					motor[Motor3].ctrl.pos_set-=0.1f;	
					if(motor[Motor3].ctrl.pos_set<-1.53f) motor[Motor3].ctrl.pos_set=-1.53f;
					while(motor[Motor3].para.pos>motor[Motor3].ctrl.pos_set+0.02f){
						ctrl_send();
					}
        }
				
				if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_R)){
					motor[Motor4].ctrl.pos_set+=0.1f;	
						if(motor[Motor4].ctrl.pos_set>3.1f) motor[Motor4].ctrl.pos_set=3.1f;
					//while(motor[Motor4].para.pos<motor[Motor4].ctrl.pos_set-0.02f){
						ctrl_send();
					//}
        }
				else if((damiao_RC->key.v & KEY_PRESSED_OFFSET_X)&&(damiao_RC->key.v & KEY_PRESSED_OFFSET_F)){
					motor[Motor4].ctrl.pos_set-=0.1f;	
						if(motor[Motor4].ctrl.pos_set<0) motor[Motor4].ctrl.pos_set=0;
					//while(motor[Motor4].para.pos<motor[Motor4].ctrl.pos_set-0.02f){
						ctrl_send();
					//}
        }

				if(bishen==1){
					motor[Motor3].ctrl.pos_set=-0.48;
					motor[Motor4].ctrl.pos_set=1.55;
					ctrl_send();
				}

				if(bishen==2){
					motor[Motor3].ctrl.pos_set=0;
					motor[Motor4].ctrl.pos_set=1.55;
					ctrl_send();
				}
				bishen=0;
				
				if(damiao_RC->mouse.press_l)
			{
					beng_state = !beng_state;
					while(damiao_RC->mouse.press_l)
					{
							osDelay(10);
					}
					gpio_set(beng_state);
			}
        //系统延时
        vTaskDelay(1);
			}
}


void dm4310_motor_init(void)
{
	// 初始化Motor1和Motor2的电机结构
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));

	// 设置Motor1的电机信息
	motor[Motor1].id = 1;
	motor[Motor1].ctrl.mode = 0;		// 0: MIT模式   1: 位置速度模式   2: 速度模式

	motor[Motor1].ctrl.pos_set=0;
	motor[Motor1].ctrl.vel_set=0;
	motor[Motor1].ctrl.tor_set=0;
	motor[Motor1].ctrl.kp_set=0;
	motor[Motor1].ctrl.kd_set=1;
	
	
	// 设置Motor2的电机信息
	motor[Motor2].id = 2;
	motor[Motor2].ctrl.mode = 0;
	
	motor[Motor2].ctrl.pos_set=0;
	motor[Motor2].ctrl.vel_set=0;
	motor[Motor2].ctrl.tor_set=0;
	motor[Motor2].ctrl.kp_set=0;
	motor[Motor2].ctrl.kd_set=1;
	
	// 设置Motor3的电机信息
	motor[Motor3].id = 3;
	motor[Motor3].ctrl.mode = 1;
	
	motor[Motor3].ctrl.pos_set=motor[Motor3].para.pos;
	motor[Motor3].ctrl.vel_set=10;
	motor[Motor3].ctrl.tor_set=0;
	motor[Motor3].ctrl.kp_set=0;
	motor[Motor3].ctrl.kd_set=1;
	
	// 设置Motor4的电机信息
	motor[Motor4].id = 4;
	motor[Motor4].ctrl.mode = 1;
	
	motor[Motor4].ctrl.pos_set=motor[Motor4].para.pos;
	motor[Motor4].ctrl.vel_set=10;
	motor[Motor4].ctrl.tor_set=0;
	motor[Motor4].ctrl.kp_set=0;
	motor[Motor4].ctrl.kd_set=0;
}

/**
************************************************************************
* @brief:      	ctrl_enable: 启用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），启用对应的电机控制。
*               设置指定电机的启动标志，并调用dm4310_enable函数启用电机。
************************************************************************
**/
void ctrl_enable(void)
{
	switch(motor_id)
	{
		case 1:
			// 启用Motor1的电机控制
			motor[Motor1].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 启用Motor2的电机控制
			motor[Motor2].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor2]);
			break;
		case 3:
			// 启用Motor3的电机控制
			motor[Motor3].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor3]);
			break;
		case 4:
			// 启用Motor4的电机控制
			motor[Motor4].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor4]);
			break;
	}
	vTaskDelay(10);
}
/**
************************************************************************
* @brief:      	ctrl_disable: 禁用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），禁用对应的电机控制。
*               设置指定电机的启动标志为0，并调用dm4310_disable函数禁用电机。
************************************************************************
**/
void ctrl_disable(void)
{
	switch(motor_id)
	{
		case 1:
			// 禁用Motor1的电机控制
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 禁用Motor2的电机控制
			motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// 禁用Motor2的电机控制
			motor[Motor3].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor3]);
			break;
		case 4:
			// 禁用Motor2的电机控制
			motor[Motor4].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor4]);
			break;
	}
}

/**
************************************************************************
* @brief:      	ctrl_clear_para: 清除电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机参数
			dm4310_clear_para(&motor[Motor1]);
			break;
		case 2:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor2]);
			break;
		case 3:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor3]);
			break;
		case 4:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor4]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: 清除电机错误信息
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 清除Motor2的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor2]);
			break;
		case 3:
			// 清除Motor3的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor3]);
			break;
		case 4:
			// 清除Motor3的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor4]);
			break;
	}
	vTaskDelay(10);
}

/**
************************************************************************
* @brief:      	ctrl_send: 发送电机控制命令函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），向对应电机发送控制命令。
*               调用dm4310_ctrl_send函数向指定电机发送控制命令，以响应外部命令。
************************************************************************
**/

void ctrl_send(void)
{
	for(motor_id=1;motor_id<5;motor_id++){
	switch(motor_id)
	{
		case 1:
			 // 向Motor1发送控制命令
			dm4310_ctrl_send(&hcan1, &motor[Motor1]);
			break;
		case 2:
			 // 向Motor2发送控制命令
			dm4310_ctrl_send(&hcan1, &motor[Motor2]);
			break;
		case 3:
			 // 向Motor3发送控制命令
			dm4310_ctrl_send(&hcan1, &motor[Motor3]);
			break;
		case 4:
			 // 向Motor4发送控制命令
			dm4310_ctrl_send(&hcan1, &motor[Motor4]);
			break;
	}
	vTaskDelay(10);
	}
}

/*void can1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0: dm4310_fbdata(&motor[Motor1], rx_data); break;
		case 1: dm4310_fbdata(&motor[Motor3], rx_data); break;
	}
}
*/
/*const motor_t *get_damiao_motor_measure_point(void)
{
    return &motor[Motor1];
}*/
