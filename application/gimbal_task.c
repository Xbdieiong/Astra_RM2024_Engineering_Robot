/******************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
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
//底盘初始化函数
static void gimbal_init(gimbal_move_t *gimbal_move_init);

  /**************************************************************************
  * @brief          测量数据更新，包括电机速度，														*
  * @param[out]     chassis_move_update:"chassis_move"变量指针.							*
  * @retval         none																										*
  ***************************************************************************/
static void gimbal_feedback_update(gimbal_move_t *gimbal_move_update);
/*************************************************************************************************
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的	*
  * @param[out]     chassis_move_update:"chassis_move"变量指针.																		*
  * @retval         none																																					*								
  *************************************************************************************************/
static void gimbal_set_contorl(gimbal_move_t *gimbal_move_update);

 /*******************************************************************************
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制					*
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.						*
  * @retval         none																												*
  *******************************************************************************/
static void gimbal_control_loop(gimbal_move_t *gimbal_move_update);		
		
		
		
		
//云台控制所有相关数据
gimbal_move_t gimbal_control;

		
/*******************************************************************
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  ******************************************************************/

void gimbal_task(void const *pvParameters)
{
   //程序暂停一段时间
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //结构体变量初始化
    gimbal_init(&gimbal_control);
    while (1)
    {
        //底盘当前状态数据更新，包括电机速度，姿态，机器人整体速度
        gimbal_feedback_update(&gimbal_control);
        //底盘控制量设置
        gimbal_set_contorl(&gimbal_control);
        //底盘控制PID计算
        gimbal_control_loop(&gimbal_control);

        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        /*if (!(toe_is_error(UP_MOTOR1_TOE) && toe_is_error(UP_MOTOR2_TOE) && toe_is_error(TURN_MOTOR1_TOE) && toe_is_error(TURN_MOTOR2_TOE)))
				{ 
            //当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE))
            {
              	CAN_cmd_mid(0,0,0,0);
            }
            else
            {
                //发送控制电流
						}
				}
					*/		
							CAN_cmd_gimbal(gimbal_control.motor_gimbal[0].give_current, gimbal_control.motor_gimbal[1].give_current,0, 0);
				
						
        
        //系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
			}
}


//底盘初始化函数
static void gimbal_init(gimbal_move_t *gimbal_move_init)
{
		//障碍块速度环pid值
		const static fp32 zak_speed_pid[3] = {MOTOR_3508_SPEED_PID_KP, MOTOR_3508_SPEED_PID_KI, MOTOR_3508_SPEED_PID_KD};
		
		//滤波参数----考虑不再使用
		const static fp32 chassis_up_order_filter[1] = {GIMBAL_ACCEL_X_NUM};
    
		//定义标志量
		uint8_t i;
		//获取遥控器数据指针
		gimbal_move_init->gimbal_RC = get_remote_control_point();
		//获取电机数据指针，初始化PID 
    
		for (i = 0; i < 2; i++)
    {
        gimbal_move_init->motor_gimbal[i].chassis_motor_measure = get_front_motor_measure_point(i);
        PID_init(&gimbal_move_init->motor_speed_pid[i], PID_POSITION,  zak_speed_pid, MOTOR_3508_SPEED_PID_MAX_OUT, MOTOR_3508_SPEED_PID_MAX_IOUT);
    }
		first_order_filter_init(&gimbal_move_init->up_cmd_slow_set, GIMBAL_CONTROL_TIME, chassis_up_order_filter);
    
		//最大 最小速度
    gimbal_move_init->vup_max_speed = NORMAL_MAX_UP_SPEED;
    gimbal_move_init->vup_min_speed = -NORMAL_MAX_UP_SPEED;
    
    //更新一下数据
    gimbal_feedback_update(gimbal_move_init);
}

  /**************************************************************************
  * @brief          云台测量数据更新，包括电机速度，升降速度，旋转速度		*
  * @param[out]     chassis_move_update:"chassis_move"变量指针.							*
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
        //更新电机速度、加速度
        gimbal_move_update->motor_gimbal[i].speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * gimbal_move_update->motor_gimbal[i].chassis_motor_measure->speed_rpm;
        gimbal_move_update->motor_gimbal[i].accel = gimbal_move_update->motor_speed_pid[i].Dbuf[0] * GIMBAL_CONTROL_FREQUENCE;
    }
    //更新底盘升降速度 x， 旋转速度y
    gimbal_move_update->vup = (gimbal_move_update->motor_gimbal[0].speed - gimbal_move_update->motor_gimbal[1].speed )/2;
    
}



 
/*************************************************************************************************
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的	*
  * @param[out]     chassis_move_update:"chassis_move"变量指针.																		*
  * @retval         none																																					*								
  *************************************************************************************************/
static void gimbal_set_contorl(gimbal_move_t *gimbal_move_control)
{
    if (gimbal_move_control == NULL)
    {
        return;
    }
    //fp32 vup_set = 0.0f;
    //获取三个控制设置值
		//int16_t vup_channel;
   // fp32 vup_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
   // gimbal_rc_deadband_limit(gimbal_move_control->gimbal_RC->rc.ch[UP_CHANNEL], vup_channel, RC_DEADBAND);

   // vup_set_channel = vup_channel * UP_RC_SEN;
		    //键盘控制
    /*if (gimbal_move_control->gimbal_RC->key.v & UP_KEYBOARD)
    {
				vup_set_channel = 0.5*gimbal_move_control->vup_max_speed;
    }
    else if (gimbal_move_control->gimbal_RC->key.v & DOWN_KEYBOARD)
    {
        vup_set_channel = 0.5*gimbal_move_control->vup_min_speed;
    }*/
		    //键盘控制
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
		
		//一阶低通滤波代替斜波作为底盘速度输入
    //first_order_filter_cali(&gimbal_move_control->up_cmd_slow_set, vup_set_channel);
		//停止信号，不需要缓慢加速，直接减速到零
    /*if (vup_set_channel < 10 * UP_RC_SEN && vup_set_channel > -10 * UP_RC_SEN)
    {
        gimbal_move_control->up_cmd_slow_set.out = 0.0f;
    }
    
    vup_set = gimbal_move_control->up_cmd_slow_set.out;

    gimbal_move_control->vup_set = fp32_constrain(vup_set, gimbal_move_control->vup_min_speed, gimbal_move_control->vup_max_speed);*/
    
}


/*******************************************************************************
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制					*
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.						*
  * @retval         none																												*
  *******************************************************************************/
static void gimbal_control_loop(gimbal_move_t *gimbal_move_update)
{
    //fp32 max_vector = 0.0f, vector_rate = 0.0f;
    //fp32 temp = 0.0f;
    //fp32 wheel_speed[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    uint8_t i = 0;
    //运动分解
		//wheel_speed[4] = gimbal_move_update->vup_set ;
    //wheel_speed[5] = -gimbal_move_update->vup_set ;
   
		
    //计算轮子控制最大速度，并限制其最大速度
    /*for (i = 4; i < 6; i++)
    {
        gimbal_move_update->motor_gimbal[i].speed_set = wheel_speed[i];
        temp = fabs( gimbal_move_update->motor_gimbal[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }*/
    //计算pid
    for (i = 0; i < 2; i++)
    {
        PID_calc(&gimbal_move_update->motor_speed_pid[i], gimbal_move_update->motor_gimbal[i].speed, gimbal_move_update->motor_gimbal[i].speed_set);
    }
    //赋值电流值
    for (i = 0; i < 2; i++)
    {
        gimbal_move_update->motor_gimbal[i].give_current = (int16_t)(gimbal_move_update->motor_speed_pid[i].out);
    }
}






