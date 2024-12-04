/******************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  ****************************(C) COPYRIGHT 2019 DJI*****************************/

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
//#include "detect_task.h"
#include "remote_control.h"
#include "pid.h"

#define gimbal_rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
//���̳�ʼ������
static void gimbal_init(gimbal_move_t *gimbal_move_init);

  /**************************************************************************
  * @brief          �������ݸ��£���������ٶȣ�														*
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.							*
  * @retval         none																										*
  ***************************************************************************/
static void gimbal_feedback_update(gimbal_move_t *gimbal_move_update);
/*************************************************************************************************
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�	*
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.																		*
  * @retval         none																																					*								
  *************************************************************************************************/
static void gimbal_set_contorl(gimbal_move_t *gimbal_move_update);

 /*******************************************************************************
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���					*
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.						*
  * @retval         none																												*
  *******************************************************************************/
static void gimbal_control_loop(gimbal_move_t *gimbal_move_update);		
		
		
		
		
//��̨���������������
gimbal_move_t gimbal_control;

		
/*******************************************************************
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  ******************************************************************/

void gimbal_task(void const *pvParameters)
{
   //������ͣһ��ʱ��
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //�ṹ�������ʼ��
    gimbal_init(&gimbal_control);
    while (1)
    {
        //���̵�ǰ״̬���ݸ��£���������ٶȣ���̬�������������ٶ�
        gimbal_feedback_update(&gimbal_control);
        //���̿���������
        gimbal_set_contorl(&gimbal_control);
        //���̿���PID����
        gimbal_control_loop(&gimbal_control);

        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        /*if (!(toe_is_error(UP_MOTOR1_TOE) && toe_is_error(UP_MOTOR2_TOE) && toe_is_error(TURN_MOTOR1_TOE) && toe_is_error(TURN_MOTOR2_TOE)))
				{ 
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE))
            {
              	CAN_cmd_mid(0,0,0,0);
            }
            else
            {
                //���Ϳ��Ƶ���
						}
				}
					*/		
							CAN_cmd_gimbal(gimbal_control.motor_gimbal[0].give_current, gimbal_control.motor_gimbal[1].give_current,0, 0);
				
						
        
        //ϵͳ��ʱ
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
			}
}


//���̳�ʼ������
static void gimbal_init(gimbal_move_t *gimbal_move_init)
{
		//�ϰ����ٶȻ�pidֵ
		const static fp32 zak_speed_pid[3] = {MOTOR_3508_SPEED_PID_KP, MOTOR_3508_SPEED_PID_KI, MOTOR_3508_SPEED_PID_KD};
		
		//�˲�����----���ǲ���ʹ��
		const static fp32 chassis_up_order_filter[1] = {GIMBAL_ACCEL_X_NUM};
    
		//�����־��
		uint8_t i;
		//��ȡң��������ָ��
		gimbal_move_init->gimbal_RC = get_remote_control_point();
		//��ȡ�������ָ�룬��ʼ��PID 
    
		for (i = 0; i < 2; i++)
    {
        gimbal_move_init->motor_gimbal[i].chassis_motor_measure = get_front_motor_measure_point(i);
        PID_init(&gimbal_move_init->motor_speed_pid[i], PID_POSITION,  zak_speed_pid, MOTOR_3508_SPEED_PID_MAX_OUT, MOTOR_3508_SPEED_PID_MAX_IOUT);
    }
		first_order_filter_init(&gimbal_move_init->up_cmd_slow_set, GIMBAL_CONTROL_TIME, chassis_up_order_filter);
    
		//��� ��С�ٶ�
    gimbal_move_init->vup_max_speed = NORMAL_MAX_UP_SPEED;
    gimbal_move_init->vup_min_speed = -NORMAL_MAX_UP_SPEED;
    
    //����һ������
    gimbal_feedback_update(gimbal_move_init);
}

  /**************************************************************************
  * @brief          ��̨�������ݸ��£���������ٶȣ������ٶȣ���ת�ٶ�		*
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.							*
  * @retval         none																										*
  ***************************************************************************/
static void gimbal_feedback_update(gimbal_move_t *gimbal_move_update)
{
    if (gimbal_move_update == NULL)
    {
        return;
    }
    uint8_t i = 0; 
    for (i = 0; i < 2; i++)
    {
        //���µ���ٶȡ����ٶ�
        gimbal_move_update->motor_gimbal[i].speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * gimbal_move_update->motor_gimbal[i].chassis_motor_measure->speed_rpm;
        gimbal_move_update->motor_gimbal[i].accel = gimbal_move_update->motor_speed_pid[i].Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
    }
    //���µ��������ٶ� x�� ��ת�ٶ�y
    gimbal_move_update->vup = (gimbal_move_update->motor_gimbal[0].speed - gimbal_move_update->motor_gimbal[1].speed )/2;
    
}



 
/*************************************************************************************************
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�	*
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.																		*
  * @retval         none																																					*								
  *************************************************************************************************/
static void gimbal_set_contorl(gimbal_move_t *gimbal_move_control)
{
    if (gimbal_move_control == NULL)
    {
        return;
    }
    //fp32 vup_set = 0.0f;
    //��ȡ������������ֵ
		//int16_t vup_channel;
   // fp32 vup_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
   // gimbal_rc_deadband_limit(gimbal_move_control->gimbal_RC->rc.ch[UP_CHANNEL], vup_channel, RC_DEADBAND);

   // vup_set_channel = vup_channel * UP_RC_SEN;
		    //���̿���
    /*if (gimbal_move_control->gimbal_RC->key.v & UP_KEYBOARD)
    {
				vup_set_channel = 0.5*gimbal_move_control->vup_max_speed;
    }
    else if (gimbal_move_control->gimbal_RC->key.v & DOWN_KEYBOARD)
    {
        vup_set_channel = 0.5*gimbal_move_control->vup_min_speed;
    }*/
		    //���̿���
    if ((gimbal_move_control->gimbal_RC->key.v & KEY_PRESSED_OFFSET_Q)&&(gimbal_move_control->gimbal_RC->key.v & KEY_PRESSED_OFFSET_R))
    {
        gimbal_move_control->motor_gimbal[0].speed_set = 0.2f;
				gimbal_move_control->motor_gimbal[1].speed_set = -0.2f;
    }
    else if ((gimbal_move_control->gimbal_RC->key.v & KEY_PRESSED_OFFSET_Q)&&(gimbal_move_control->gimbal_RC->key.v & KEY_PRESSED_OFFSET_V))
    {
        gimbal_move_control->motor_gimbal[0].speed_set = -0.2f;
				gimbal_move_control->motor_gimbal[1].speed_set = 0.2f;
    }
		else if (gimbal_move_control->gimbal_RC->rc.ch[3]>40)
    {
        gimbal_move_control->motor_gimbal[0].speed_set = 0.2f;
				gimbal_move_control->motor_gimbal[1].speed_set = -0.2f;
    }
		else if (gimbal_move_control->gimbal_RC->rc.ch[3]<-40)
		{
        gimbal_move_control->motor_gimbal[0].speed_set = -0.2f;
				gimbal_move_control->motor_gimbal[1].speed_set = 0.2f;
    }
		else {
				gimbal_move_control->motor_gimbal[0].speed_set = 0.0f;
				gimbal_move_control->motor_gimbal[1].speed_set = 0.0f;	
		}
		
		//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    //first_order_filter_cali(&gimbal_move_control->up_cmd_slow_set, vup_set_channel);
		//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    /*if (vup_set_channel < 10 * UP_RC_SEN && vup_set_channel > -10 * UP_RC_SEN)
    {
        gimbal_move_control->up_cmd_slow_set.out = 0.0f;
    }
    
    vup_set = gimbal_move_control->up_cmd_slow_set.out;

    gimbal_move_control->vup_set = fp32_constrain(vup_set, gimbal_move_control->vup_min_speed, gimbal_move_control->vup_max_speed);*/
    
}


/*******************************************************************************
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���					*
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.						*
  * @retval         none																												*
  *******************************************************************************/
static void gimbal_control_loop(gimbal_move_t *gimbal_move_update)
{
    //fp32 max_vector = 0.0f, vector_rate = 0.0f;
    //fp32 temp = 0.0f;
    //fp32 wheel_speed[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    uint8_t i = 0;
    //�˶��ֽ�
		//wheel_speed[4] = gimbal_move_update->vup_set ;
    //wheel_speed[5] = -gimbal_move_update->vup_set ;
   
		
    //�������ӿ�������ٶȣ�������������ٶ�
    /*for (i = 4; i < 6; i++)
    {
        gimbal_move_update->motor_gimbal[i].speed_set = wheel_speed[i];
        temp = fabs( gimbal_move_update->motor_gimbal[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }*/
    //����pid
    for (i = 0; i < 2; i++)
    {
        PID_calc(&gimbal_move_update->motor_speed_pid[i], gimbal_move_update->motor_gimbal[i].speed, gimbal_move_update->motor_gimbal[i].speed_set);
    }
    //��ֵ����ֵ
    for (i = 0; i < 2; i++)
    {
        gimbal_move_update->motor_gimbal[i].give_current = (int16_t)(gimbal_move_update->motor_speed_pid[i].out);
    }
}






