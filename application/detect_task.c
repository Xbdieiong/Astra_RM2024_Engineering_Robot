/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      ���������� ͨ����������ʱ�����ж�.�ṩ ��⹳�Ӻ���,������ں���.
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  ==============================================================================
    ���Ҫ����һ�����豸
    1.��һ����detect_task.h�������豸������errorList�������
    enum errorList
    {
        ...
        XXX_TOE,    //���豸
        ERROR_LIST_LENGHT,
    };
    2.��detect_init����,����offlineTime, onlinetime, priority����
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.�����data_is_error_fun ,solve_lost_fun,solve_data_error_fun��������ֵ������ָ��
    4.��XXX_TOE�豸��������ʱ��, ���Ӻ���detect_hook(XXX_TOE).
  ==============================================================================
  ****************************(C) COPYRIGHT 2019 DJI*****************************/
  
#include "detect_task.h"
#include "cmsis_os.h"

/**
  * @brief          ��ʼ��error_list,��ֵ offline_time, online_time, priority
  * @param[in]      time:ϵͳʱ��
  * @retval         none
  */
static void detect_init(uint32_t time);

error_t error_list[ERROR_LIST_LENGHT + 1];

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif


/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void detect_task(void const *pvParameters)
{
	//����ϵͳʱ��
    static uint32_t system_time;
	//��ȡϵͳʱ��
    system_time = xTaskGetTickCount();
    //��ʼ���豸״̬����ÿ���豸�ṹ�帳ֵ��
    detect_init(system_time);
    //����һ��ʱ��
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {	//���������=0
        static uint8_t error_num_display = 0;
		//����ϵͳʱ���
        system_time = xTaskGetTickCount();
		//���������=�豸��
        error_num_display = ERROR_LIST_LENGHT;
		//ȡ�����ڼ�����
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            //�����i���豸δʹ�ܣ�����������һ��ѭ��
            if (error_list[i].enable == 0)
            {
                continue;
            }
            //�ж��Ƿ���ߣ������ǰʱ���ȥ�ϴ�ʱ��������õ�����ʱ�䣩
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
				//���δ��¼���ڴ���
                if (error_list[i].error_exist == 0)
                {
                    //��¼�����Լ�����ʱ��
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_time;
                }
                //�жϴ������ȼ��� �������ȼ���ߵĴ�����
                if (error_list[i].priority > error_list[error_num_display].priority)
                {
                    error_num_display = i;
                }
                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;
                //����ṩ������������н������
                if (error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }
			//�ж��Ƿ���ߣ������ǰʱ���ȥ�ϴ�ʱ��С�����õ�����ʱ�䣩
            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                //�ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].is_lost = 0;
                //�ж��Ƿ�������ݴ���
                if (error_list[i].data_is_error != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }
                //����Ƶ��
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          ��ȡ�豸��Ӧ�Ĵ���״̬
  * @param[in]      toe:�豸Ŀ¼
  * @retval         true(����) ����false(û����)
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}


/**
  * @brief          ��¼ʱ��
  * @param[in]      toe:�豸Ŀ¼
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    
    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }
    
    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}

/**
  * @brief          �õ������б�
  * @param[in]      none
  * @retval         error_list��ָ��
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

extern void OLED_com_reset(void);
static void detect_init(uint32_t time)
{
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ�
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            {30, 40, 15},   //SBUS
            {10, 10, 14},   //motor1
            {10, 10, 13},   //motor2
            {10, 10, 12},   //motor3
            {10, 10, 11},   //motor4
            {10, 10, 10},   //motor5
            {10, 10, 9},    //motor6
            {10, 10, 8},    //motor7
            {10, 10, 7},    //motor8
            {10, 10, 2},    //rm imu
        };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }
}


