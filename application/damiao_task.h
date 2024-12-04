#ifndef DAMIAO_TASK_H
#define DAMIAO_TASK_H
#include "main.h"
#include "dm4310_drv.h"

extern int8_t motor_id;

//任务初始化 空闲一段时间
#define DAMIAO_TASK_INIT_TIME 201

extern void damiao_task(void const *pvParameters);

typedef enum
{
	Motor1,
	Motor2,
	Motor3,
	Motor4,
	num
} motor_num;

extern motor_t motor[num];

void dm4310_motor_init(void);
void ctrl_enable(void);
void ctrl_disable(void);
void ctrl_clear_para(void);
void ctrl_clear_err(void);
void ctrl_send(void);
const motor_t *get_damiao_motor_measure_point(void);
#endif
