#include "enginer_task.h"
#include "main.h"
#include "cmsis_os.h"

enginer_control_t enginer_move;

enginer_state nowState = init;
int timeCount = 0;
int16_t angleCount = 0;
uint32_t time_smooth_speed;
fp32 target_a=0;

int8_t bishen=0;

uint16_t pwm1=430,pwm2=900;
extern TIM_HandleTypeDef htim5;
static void enginer_init(enginer_control_t *enginer_move_init)
{
	const static fp32 motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
	const static fp32 motor_positon_pid[3] = {M3508_MOTOR_POSITION_PID_KP, M3508_MOTOR_POSITION_PID_KI, M3508_MOTOR_POSITION_PID_KD};
	const static fp32 motor_su_speed_pid[3] = {UP_MOTOR_SPEED_PID_KP, UP_MOTOR_SPEED_PID_KI, UP_MOTOR_SPEED_PID_KD};

    enginer_move_init->enginer_rc_ctrl = get_remote_control_point();
	
	  
	    enginer_move_init->enginer_yaw_motor.enginer_motor_measure = get_mid_motor_measure_point(4);
			enginer_move_init->enginer_pitch_motor.enginer_motor_measure = get_mid_motor_measure_point(5);
		for(int i=0;i<2;i++)
	{
		PID_init(&enginer_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
		PID_init(&enginer_move_init->motor_angle_pid[i], PID_POSITION, motor_positon_pid, M3508_MOTOR_POSITION_PID_MAX_OUT, M3508_MOTOR_POSITION_PID_MAX_IOUT);
	}
	for(int i=2;i<4;i++){
	PID_init(&enginer_move_init->motor_speed_pid[i], PID_POSITION, motor_su_speed_pid, UP_MOTOR_SPEED_PID_MAX_OUT, UP_MOTOR_SPEED_PID_MAX_IOUT);
		}
	enginer_move_init->enginer_yaw_motor.enginer_motor_measure->ecd_count =0 ;
	enginer_move_init->enginer_pitch_motor.enginer_motor_measure->ecd_count =0 ;
}


static void enginer_update(void)
{
				
		enginer_move.enginer_pitch_motor.angle = (enginer_move.enginer_pitch_motor.enginer_motor_measure->ecd_count*ECD_RANGE +enginer_move.enginer_pitch_motor.enginer_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;
		enginer_move.enginer_yaw_motor.angle = (enginer_move.enginer_yaw_motor.enginer_motor_measure->ecd_count*ECD_RANGE +enginer_move.enginer_yaw_motor.enginer_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;	
		enginer_move.enginer_pitch_motor.speed = UP_M3508_MOTOR_RPM_TO_VECTOR * enginer_move.enginer_pitch_motor.enginer_motor_measure->speed_rpm;
		enginer_move.enginer_yaw_motor.speed = UP_M3508_MOTOR_RPM_TO_VECTOR * enginer_move.enginer_yaw_motor.enginer_motor_measure->speed_rpm;
}

void PIDrealize(void)
{
			/* if(nowState == init)
			{
			enginer_move.enginer_pitch_motor.give_current = INIT_CURRENT;
			enginer_move.enginer_yaw_motor.give_current = -INIT_CURRENT;//两电机相反
			}*/
			//else
			//{
			enginer_move.enginer_pitch_motor.target_speed = PID_calc(&enginer_move.motor_angle_pid[0],enginer_move.enginer_pitch_motor.angle, enginer_move.enginer_pitch_motor.target_angle);
			enginer_move.enginer_yaw_motor.target_speed = PID_calc(&enginer_move.motor_angle_pid[1],enginer_move.enginer_yaw_motor.angle,enginer_move.enginer_yaw_motor.target_angle);
			enginer_move.enginer_pitch_motor.give_current = PID_calc(&enginer_move.motor_speed_pid[0],enginer_move.enginer_pitch_motor.enginer_motor_measure->speed_rpm, enginer_move.enginer_pitch_motor.target_speed);
			enginer_move.enginer_yaw_motor.give_current = PID_calc(&enginer_move.motor_speed_pid[1],enginer_move.enginer_yaw_motor.enginer_motor_measure->speed_rpm, enginer_move.enginer_yaw_motor.target_speed);
			//}
}


void enginer_task(void const *pvParameters)
{
		osDelay(100);
		enginer_init(&enginer_move);
	  uint32_t k=0;
	while(1)
	{
		enginer_update();
				
					if(HAL_GetTick() - time_smooth_speed > 70) 
					{
						target_a += enginer_move.enginer_rc_ctrl->rc.ch[4] * -0.001;
						if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E))
						{
							target_a +=0.3f;
						}
						else if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C))
						{
							target_a -=0.3f;
						}
						time_smooth_speed = HAL_GetTick();
					}
					if(target_a > 21.4f) target_a = 21.4f;
					if(target_a <0) target_a = 0;
					//一键臂伸出
					if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL))
						{
								
							target_a =13.5f;
							enginer_move.enginer_pitch_motor.target_angle = -target_a;
							enginer_move.enginer_yaw_motor.target_angle = target_a;
								for(k=0;k<150;k++){
									enginer_update();
									PIDrealize();
									CAN_cmd_mid(enginer_move.enginer_yaw_motor.give_current,enginer_move.enginer_pitch_motor.give_current,0,0);
									osDelay(10);
								}
							bishen=1;
						}
						
						if(enginer_move.enginer_rc_ctrl->mouse.press_r)
						{
								
							target_a =13.5f;
							enginer_move.enginer_pitch_motor.target_angle = -target_a;
							enginer_move.enginer_yaw_motor.target_angle = target_a;
								for(k=0;k<150;k++){
									enginer_update();
									PIDrealize();
									CAN_cmd_mid(enginer_move.enginer_yaw_motor.give_current,enginer_move.enginer_pitch_motor.give_current,0,0);
									osDelay(10);
								}
							bishen=2;
						}
						
						
				enginer_move.enginer_pitch_motor.target_angle = -target_a;
				enginer_move.enginer_yaw_motor.target_angle = target_a;
			 
		PIDrealize();
		CAN_cmd_mid(enginer_move.enginer_yaw_motor.give_current,enginer_move.enginer_pitch_motor.give_current,0,0);
						
						
		while(switch_is_up(enginer_move.enginer_rc_ctrl->rc.s[0])){
					enginer_update();
					enginer_move.enginer_pitch_motor.speed_set=0.2f;
					enginer_move.enginer_yaw_motor.speed_set=-0.2f;
					enginer_move.enginer_pitch_motor.give_current=PID_calc(&enginer_move.motor_speed_pid[2],enginer_move.enginer_pitch_motor.speed,enginer_move.enginer_pitch_motor.speed_set);
					enginer_move.enginer_yaw_motor.give_current=PID_calc(&enginer_move.motor_speed_pid[3],enginer_move.enginer_yaw_motor.speed,enginer_move.enginer_yaw_motor.speed_set);
					CAN_cmd_mid(enginer_move.enginer_yaw_motor.give_current,enginer_move.enginer_pitch_motor.give_current,0,0);
					enginer_move.enginer_yaw_motor.enginer_motor_measure->ecd_count =0 ;
					enginer_move.enginer_pitch_motor.enginer_motor_measure->ecd_count =0 ;
					target_a=0;
					osDelay(10);
				}
		if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)){
			pwm1+=45;
			if(pwm1>1250) pwm1=1250;
			while((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F)) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm1);
		}
		else if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)){
			pwm1-=45;
			if(pwm1<250) pwm1=250;
			while((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm1);
		}
		else if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)){	
			pwm2+=45;
			if(pwm2>1250) pwm2=1250;
			while((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm2);
		}
		else if((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)){
			pwm2-=45;
			if(pwm2<250) pwm2=250;
			while((enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)&&(enginer_move.enginer_rc_ctrl->key.v & KEY_PRESSED_OFFSET_C)) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm2);
		}
				
			
		
			/*if(enginer_move.enginer_rc_ctrl->rc.ch[0]>40){
				pwm1+=30;
			if(pwm1>1250) pwm1=1250;
			while(enginer_move.enginer_rc_ctrl->rc.ch[0]>40) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm1);
		}
		
			if(enginer_move.enginer_rc_ctrl->rc.ch[0]<-40){
		pwm1-=30;
			if(pwm1<250) pwm1=250;
			while(enginer_move.enginer_rc_ctrl->rc.ch[0]<-40) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm1);
		}
	
		if(enginer_move.enginer_rc_ctrl->rc.ch[1]>40){	
		pwm2+=30;
			if(pwm2>1250) pwm2=1250;
			while(enginer_move.enginer_rc_ctrl->rc.ch[1]>40) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm2);
		}
		
			if(enginer_move.enginer_rc_ctrl->rc.ch[1]<-40){
		pwm2-=30;
			if(pwm2<250) pwm2=250;
			while(enginer_move.enginer_rc_ctrl->rc.ch[1]<-40) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm2);
		}*/
		osDelay(1);
	}
}

