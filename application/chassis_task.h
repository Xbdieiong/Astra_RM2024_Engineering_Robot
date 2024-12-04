/******************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务
  *  Version    Date            Author          Modification
  ****************************(C) COPYRIGHT 2019 DJI*****************************/

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
//#include "usb_task.h"

//前后的遥控器通道号码（定义通道号，方便更改）
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2
//选择底盘状态 开关通道号（选择底盘状态） s
#define CHASSIS_MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f




#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f
//摇杆死区
#define CHASSIS_RC_DEADLINE 10

//电机速度到速度比例
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.6410f
//带确定
#define MOTOR_DISTANCE_TO_CENTER 0.395f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 4
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.004f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 250.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_TURN_RIGHT_KEY KEY_PRESSED_OFFSET_Q
#define CHASSIS_TURN_LEFT_KEY KEY_PRESSED_OFFSET_E
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.00042027f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.50f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 50.0f
#define M3505_MOTOR_SPEED_PID_KD 50.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 15000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 4.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f
/////////////////////////////////////////
#define CHASSIS_Zt_TO_VALUE 5214.55264
#define COMMANT_PID_KP 0.4f
#define COMMANT_PID_KI 0.001f
#define COMMANT_PID_KD 0.7f
#define COMMANT_PID_MAX_OUT 4.0f
#define COMMANT_PID_MAX_IOUT 2.0f

//底盘操控模式列表
typedef enum
{
	CHASSIS_SLAM_RPM,						//SLAM
	CHASSIS_VECTOR_RAW,                 	//直接发送到CAN总线的控制电流将。
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       	//底盘有旋转速度控制
} chassis_mode_e;

typedef struct
{
    	const motor_measure_t *chassis_motor_measure;  
    	fp32 accel;  											//加速度
    	fp32 speed;  											//速度
    	fp32 speed_set;  						    	//给定速度
			fp32 angle_set;  						    	//给定速度
    	int16_t give_current;  			            //给定电流
} chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;               			//遥控器指针
	/*底盘模式变量*/
	chassis_mode_e chassis_mode;               			//底盘控制状态

	chassis_motor_t motor_chassis[4];          			//底盘电机数据
	pid_type_def motor_speed_pid[4];           			//底盘电机速度pid
	pid_type_def chassis_angle_pid;            			//底盘跟随角度pid
	
	pid_type_def motor_angle_pid[4];           			//  底盘电机角度PID
	first_order_filter_type_t chassis_cmd_slow_set_angle; 
	first_order_filter_type_t chassis_cmd_slow_set_vx;  	//使用一阶低通滤波减缓设定值
	first_order_filter_type_t chassis_cmd_slow_set_vy;  	//使用一阶低通滤波减缓设定值
	first_order_filter_type_t chassis_cmd_slow_set_wz;  	//使用一阶低通滤波减缓设定值

	//限制
	fp32 vx_max_speed;  							//前进方向最大速度 单位m/s
	fp32 vx_min_speed;  							//后退方向最大速度 单位m/s
	fp32 vy_max_speed;  							//左方向最大速度 单位m/s
	fp32 vy_min_speed;  							//右方向最大速度 单位m/s
	//底盘当前速度状态
	fp32 vx;                          				//底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;                          				//底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;  
	//期待的状态---更新的命令-----设定值
	fp32 vx_set;                      				//底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;                      				//底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;                      				//底盘设定旋转角速度，逆时针为正 单位 rad/s           
} chassis_move_t;


/*** @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);


/*** @brief          根据遥控器通道值，计算纵向和横移速度
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
