#ifndef ENGINER_TASK_H
#define ENGINER_TASK_H

#include "struct_typedef.h"
#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "remote_control.h"
#define INIT_CURRENT       1800

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
#define REMOTE_LIFT_MAX 3

#define MOTOR_ECD_TO_ANGLE          0.000039952259f

#define M3508_MOTOR_POSITION_PID_KP 1500.0f
#define M3508_MOTOR_POSITION_PID_KI 0.0f
#define M3508_MOTOR_POSITION_PID_KD 0.0f
#define M3508_MOTOR_POSITION_PID_MAX_OUT 1500.0f
#define M3508_MOTOR_POSITION_PID_MAX_IOUT 400.0f

#define M3508_MOTOR_SPEED_PID_KP 9.0f
#define M3508_MOTOR_SPEED_PID_KI 1.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//升降电机速度环PID------------------需要调整
#define UP_MOTOR_SPEED_PID_KP 12000.0f
#define UP_MOTOR_SPEED_PID_KI 300.0f
#define UP_MOTOR_SPEED_PID_KD 6000.0f
#define UP_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define UP_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


//m3508转化成速度(m/s)的比例，
#define UP_M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define UP_KEYBOARD KEY_PRESSED_OFFSET_Q
//#define DOWN_KEYBOARD KEY_PRESSED_OFFSET_E

typedef struct
{
    motor_measure_t *enginer_motor_measure;
		int16_t  target_speed;//目标速度
		float target_angle;//目标角度
		float angle;//累积角度
		fp32 speed;  								//速度
		fp32 speed_set;  						//给定速度
		int16_t give_current;
} enginer_motor_t;


typedef struct
{
    const RC_ctrl_t *enginer_rc_ctrl;
    enginer_motor_t enginer_yaw_motor;
	  enginer_motor_t enginer_pitch_motor;
	  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
		pid_type_def motor_angle_pid[2];              //follow angle PID.底盘跟随角度pid
} enginer_control_t;

typedef enum
{
	init,
	in,
} enginer_state;

extern void enginer_task(void const *pvParameters);

#endif
