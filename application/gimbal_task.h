/*����*/
#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//��������ͨ���Լ�״̬����ͨ��
#define UP_CHANNEL   3


//������������ذ��� Q/E
#define UP_KEYBOARD KEY_PRESSED_OFFSET_Q
#define DOWN_KEYBOARD KEY_PRESSED_OFFSET_E
//���������ĵ���ٶ�
#define UP_SPEED    4f

//ҡ�ˣ�max 660��ת���ɵ���ٶȣ�m/s���ı���
#define UP_RC_SEN    0.004f




//�Ҳ�5��������ٶȻ�PID------------------��Ҫ����
#define UP5_MOTOR_SPEED_PID_KP 12000.0f
#define UP5_MOTOR_SPEED_PID_KI 300.0f
#define UP5_MOTOR_SPEED_PID_KD 6000.0f
#define UP5_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define UP5_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//���6��������ٶȻ�PID------------------��Ҫ����
#define UP6_MOTOR_SPEED_PID_KP 12000.0f
#define UP6_MOTOR_SPEED_PID_KI 300.0f
#define UP6_MOTOR_SPEED_PID_KD 6000.0f
#define UP6_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define UP6_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//�ϰ����Ԯ����ٶȻ�PID
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
//�������λ�û�pid------------------��Ҫ����
//#define UP_MOTOR_POSITION_PID_KP 15000.0f
//#define UP_MOTOR_POSITION_PID_KI 50.0f
//#define UP_MOTOR_POSITION_PID_KD 50.0f
//#define UP_MOTOR_POSITION_PID_MAX_OUT 16000.0f
//#define UP_MOTOR_POSITION_PID_MAX_IOUT 2000.0f



//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND   10

#define GIMBAL_ACCEL_X_NUM 0.1666666667f


//m3508ת�����ٶ�(m/s)�ı�����
#define GIMBAL_M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define GIMBAL_MOTOR_RPM_TO_VECTOR_SEN GIMBAL_M3508_MOTOR_RPM_TO_VECTOR

//�˶�������������ٶ�
#define NORMAL_MAX_UP_SPEED 1.0f

//���Ƽ�� 1ms
#define GIMBAL_CONTROL_TIME_MS 1
#define GIMBAL_CONTROL_TIME 0.001
//�����������Ƶ�ʣ���δʹ�������
#define GIMBAL_CONTROL_FREQUENCE 1000.0f
typedef struct
{
  const motor_measure_t *chassis_motor_measure;  
  fp32 accel;  								//���ٶ�
  fp32 speed;  								//�ٶ�
  fp32 speed_set;  						//�����ٶ�
  int16_t give_current;  			//��������
} gimbal_motor_t;


typedef struct
{
	//ң��������ָ��
  const RC_ctrl_t *gimbal_RC;               						//ң����ָ��
  
	gimbal_motor_t motor_gimbal[2];          							//�����������PID
  pid_type_def motor_speed_pid[2];           						//����ٶ�PID
	
  first_order_filter_type_t up_cmd_slow_set;  					//ʹ��һ�׵�ͨ�˲������趨ֵ
	
	//�����ٶ�״̬-��ǰ״̬
  fp32 vup;                          										//�����ٶ� ǰ������ ǰΪ������λ m/s
  
	
	//�ڴ���״̬-���µ�����
  fp32 vup_set;                      										//�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  
          
	//����
  fp32 vup_max_speed;  																	//������������ٶ� ��λm/s
  fp32 vup_min_speed;  																	//�½���������ٶ� ��λm/s
} gimbal_move_t;

extern void gimbal_task(void const *pvParameters);

#endif
