/*升降*/
#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//升降控制通道以及状态开关通道
#define UP_CHANNEL   3


//定义升降的相关按键 Q/E
#define UP_KEYBOARD KEY_PRESSED_OFFSET_Q
#define DOWN_KEYBOARD KEY_PRESSED_OFFSET_E
//定义升降的电机速度
#define UP_SPEED    4f

//摇杆（max 660）转化成电机速度（m/s）的比例
#define UP_RC_SEN    0.004f




//右侧5升降电机速度环PID------------------需要调整
#define UP5_MOTOR_SPEED_PID_KP 12000.0f
#define UP5_MOTOR_SPEED_PID_KI 300.0f
#define UP5_MOTOR_SPEED_PID_KD 6000.0f
#define UP5_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define UP5_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//左侧6升降电机速度环PID------------------需要调整
#define UP6_MOTOR_SPEED_PID_KP 12000.0f
#define UP6_MOTOR_SPEED_PID_KI 300.0f
#define UP6_MOTOR_SPEED_PID_KD 6000.0f
#define UP6_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define UP6_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//障碍块救援电机速度环PID
#define MOTOR_3508_SPEED_PID_KP 10000.0f
#define MOTOR_3508_SPEED_PID_KI 300.0f
#define MOTOR_3508_SPEED_PID_KD 5000.0f
#define MOTOR_3508_SPEED_PID_MAX_OUT 16000.0f
#define MOTOR_3508_SPEED_PID_MAX_IOUT 2000.0f


#define MOTOR_2006_SPEED_PID_KP 10000.0f
#define MOTOR_2006_SPEED_PID_KI 500.0f
#define MOTOR_2006_SPEED_PID_KD 1500.0f
#define MOTOR_2006_SPEED_PID_MAX_OUT 10000.0f
#define MOTOR_2006_SPEED_PID_MAX_IOUT 4000.0f
//升降电机位置环pid------------------需要调整
//#define UP_MOTOR_POSITION_PID_KP 15000.0f
//#define UP_MOTOR_POSITION_PID_KI 50.0f
//#define UP_MOTOR_POSITION_PID_KD 50.0f
//#define UP_MOTOR_POSITION_PID_MAX_OUT 16000.0f
//#define UP_MOTOR_POSITION_PID_MAX_IOUT 2000.0f



//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

#define GIMBAL_ACCEL_X_NUM 0.1666666667f


//m3508转化成速度(m/s)的比例，
#define GIMBAL_M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define GIMBAL_MOTOR_RPM_TO_VECTOR_SEN GIMBAL_M3508_MOTOR_RPM_TO_VECTOR

//运动过程最大上升速度
#define NORMAL_MAX_UP_SPEED 1.0f

//控制间隔 1ms
#define GIMBAL_CONTROL_TIME_MS 1
#define GIMBAL_CONTROL_TIME 0.001
//底盘任务控制频率，尚未使用这个宏
#define GIMBAL_CONTROL_FREQUENCE 1000.0f
typedef struct
{
  const motor_measure_t *chassis_motor_measure;  
  fp32 accel;  								//加速度
  fp32 speed;  								//速度
  fp32 speed_set;  						//给定速度
  int16_t give_current;  			//给定电流
} gimbal_motor_t;


typedef struct
{
	//遥控器数据指针
  const RC_ctrl_t *gimbal_RC;               						//遥控器指针
  
	gimbal_motor_t motor_gimbal[2];          							//升降电机数据PID
  pid_type_def motor_speed_pid[2];           						//电机速度PID
	
  first_order_filter_type_t up_cmd_slow_set;  					//使用一阶低通滤波减缓设定值
	
	//底盘速度状态-当前状态
  fp32 vup;                          										//升降速度 前进方向 前为正，单位 m/s
  
	
	//期待的状态-更新的命令
  fp32 vup_set;                      										//底盘设定速度 前进方向 前为正，单位 m/s
  
          
	//限制
  fp32 vup_max_speed;  																	//上升方向最大速度 单位m/s
  fp32 vup_min_speed;  																	//下降方向最大速度 单位m/s
} gimbal_move_t;

extern void gimbal_task(void const *pvParameters);

#endif
