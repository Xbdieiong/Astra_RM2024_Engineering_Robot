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
   //������ͣһ��ʱ��
    vTaskDelay(22000);
	  uint8_t damiao_biao=0;
    //�����ʼ��������ID������ģʽ������ģʽ
    dm4310_motor_init();
	  damiao_RC = get_remote_control_point();
	  //���õ��
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
				//���͵����������
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
        //ϵͳ��ʱ
        vTaskDelay(1);
			}
}


void dm4310_motor_init(void)
{
	// ��ʼ��Motor1��Motor2�ĵ���ṹ
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));

	// ����Motor1�ĵ����Ϣ
	motor[Motor1].id = 1;
	motor[Motor1].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ

	motor[Motor1].ctrl.pos_set=0;
	motor[Motor1].ctrl.vel_set=0;
	motor[Motor1].ctrl.tor_set=0;
	motor[Motor1].ctrl.kp_set=0;
	motor[Motor1].ctrl.kd_set=1;
	
	
	// ����Motor2�ĵ����Ϣ
	motor[Motor2].id = 2;
	motor[Motor2].ctrl.mode = 0;
	
	motor[Motor2].ctrl.pos_set=0;
	motor[Motor2].ctrl.vel_set=0;
	motor[Motor2].ctrl.tor_set=0;
	motor[Motor2].ctrl.kp_set=0;
	motor[Motor2].ctrl.kd_set=1;
	
	// ����Motor3�ĵ����Ϣ
	motor[Motor3].id = 3;
	motor[Motor3].ctrl.mode = 1;
	
	motor[Motor3].ctrl.pos_set=motor[Motor3].para.pos;
	motor[Motor3].ctrl.vel_set=10;
	motor[Motor3].ctrl.tor_set=0;
	motor[Motor3].ctrl.kp_set=0;
	motor[Motor3].ctrl.kd_set=1;
	
	// ����Motor4�ĵ����Ϣ
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
* @brief:      	ctrl_enable: ���õ�����ƺ���
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ�ĵ�����ơ�
*               ����ָ�������������־��������dm4310_enable�������õ����
************************************************************************
**/
void ctrl_enable(void)
{
	switch(motor_id)
	{
		case 1:
			// ����Motor1�ĵ������
			motor[Motor1].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// ����Motor2�ĵ������
			motor[Motor2].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor2]);
			break;
		case 3:
			// ����Motor3�ĵ������
			motor[Motor3].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor3]);
			break;
		case 4:
			// ����Motor4�ĵ������
			motor[Motor4].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor4]);
			break;
	}
	vTaskDelay(10);
}
/**
************************************************************************
* @brief:      	ctrl_disable: ���õ�����ƺ���
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ�ĵ�����ơ�
*               ����ָ�������������־Ϊ0��������dm4310_disable�������õ����
************************************************************************
**/
void ctrl_disable(void)
{
	switch(motor_id)
	{
		case 1:
			// ����Motor1�ĵ������
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// ����Motor2�ĵ������
			motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// ����Motor2�ĵ������
			motor[Motor3].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor3]);
			break;
		case 4:
			// ����Motor2�ĵ������
			motor[Motor4].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor4]);
			break;
	}
}

/**
************************************************************************
* @brief:      	ctrl_clear_para: ��������������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id���������Ӧ����Ĳ�����
*               ����dm4310_clear�������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// ���Motor1�ĵ������
			dm4310_clear_para(&motor[Motor1]);
			break;
		case 2:
			// ���Motor2�ĵ������
			dm4310_clear_para(&motor[Motor2]);
			break;
		case 3:
			// ���Motor2�ĵ������
			dm4310_clear_para(&motor[Motor3]);
			break;
		case 4:
			// ���Motor2�ĵ������
			dm4310_clear_para(&motor[Motor4]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: ������������Ϣ
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id���������Ӧ����Ĳ�����
*               ����dm4310_clear�������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// ���Motor1�ĵ���������
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// ���Motor2�ĵ���������
			dm4310_clear_err(&hcan1, &motor[Motor2]);
			break;
		case 3:
			// ���Motor3�ĵ���������
			dm4310_clear_err(&hcan1, &motor[Motor3]);
			break;
		case 4:
			// ���Motor3�ĵ���������
			dm4310_clear_err(&hcan1, &motor[Motor4]);
			break;
	}
	vTaskDelay(10);
}

/**
************************************************************************
* @brief:      	ctrl_send: ���͵�����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������Ӧ������Ϳ������
*               ����dm4310_ctrl_send������ָ��������Ϳ����������Ӧ�ⲿ���
************************************************************************
**/

void ctrl_send(void)
{
	for(motor_id=1;motor_id<5;motor_id++){
	switch(motor_id)
	{
		case 1:
			 // ��Motor1���Ϳ�������
			dm4310_ctrl_send(&hcan1, &motor[Motor1]);
			break;
		case 2:
			 // ��Motor2���Ϳ�������
			dm4310_ctrl_send(&hcan1, &motor[Motor2]);
			break;
		case 3:
			 // ��Motor3���Ϳ�������
			dm4310_ctrl_send(&hcan1, &motor[Motor3]);
			break;
		case 4:
			 // ��Motor4���Ϳ�������
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
