/******************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      ���̿�������
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

//ǰ���ң����ͨ�����루����ͨ���ţ�������ģ�
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2
//ѡ�����״̬ ����ͨ���ţ�ѡ�����״̬�� s
#define CHASSIS_MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f




#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f
//ҡ������
#define CHASSIS_RC_DEADLINE 10

//����ٶȵ��ٶȱ���
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.6410f
//��ȷ��
#define MOTOR_DISTANCE_TO_CENTER 0.395f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 4
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.004f
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 250.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_TURN_RIGHT_KEY KEY_PRESSED_OFFSET_Q
#define CHASSIS_TURN_LEFT_KEY KEY_PRESSED_OFFSET_E
//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.00042027f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.50f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 50.0f
#define M3505_MOTOR_SPEED_PID_KD 50.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 15000.0f

//������ת����PID
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

//���̲ٿ�ģʽ�б�
typedef enum
{
	CHASSIS_SLAM_RPM,						//SLAM
	CHASSIS_VECTOR_RAW,                 	//ֱ�ӷ��͵�CAN���ߵĿ��Ƶ�������
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       	//��������ת�ٶȿ���
} chassis_mode_e;

typedef struct
{
    	const motor_measure_t *chassis_motor_measure;  
    	fp32 accel;  											//���ٶ�
    	fp32 speed;  											//�ٶ�
    	fp32 speed_set;  						    	//�����ٶ�
			fp32 angle_set;  						    	//�����ٶ�
    	int16_t give_current;  			            //��������
} chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;               			//ң����ָ��
	/*����ģʽ����*/
	chassis_mode_e chassis_mode;               			//���̿���״̬

	chassis_motor_t motor_chassis[4];          			//���̵������
	pid_type_def motor_speed_pid[4];           			//���̵���ٶ�pid
	pid_type_def chassis_angle_pid;            			//���̸���Ƕ�pid
	
	pid_type_def motor_angle_pid[4];           			//  ���̵���Ƕ�PID
	first_order_filter_type_t chassis_cmd_slow_set_angle; 
	first_order_filter_type_t chassis_cmd_slow_set_vx;  	//ʹ��һ�׵�ͨ�˲������趨ֵ
	first_order_filter_type_t chassis_cmd_slow_set_vy;  	//ʹ��һ�׵�ͨ�˲������趨ֵ
	first_order_filter_type_t chassis_cmd_slow_set_wz;  	//ʹ��һ�׵�ͨ�˲������趨ֵ

	//����
	fp32 vx_max_speed;  							//ǰ����������ٶ� ��λm/s
	fp32 vx_min_speed;  							//���˷�������ٶ� ��λm/s
	fp32 vy_max_speed;  							//��������ٶ� ��λm/s
	fp32 vy_min_speed;  							//�ҷ�������ٶ� ��λm/s
	//���̵�ǰ�ٶ�״̬
	fp32 vx;                          				//�����ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy;                          				//�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
	fp32 wz;  
	//�ڴ���״̬---���µ�����-----�趨ֵ
	fp32 vx_set;                      				//�����趨�ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy_set;                      				//�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 wz_set;                      				//�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s           
} chassis_move_t;


/*** @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);


/*** @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
