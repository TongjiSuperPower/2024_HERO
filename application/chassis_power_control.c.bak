/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "usart_debug.h"

#define POWER_LIMIT         80.0f

#define WARNING_POWER_BUFF  55.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, ���ж��ܵ�������
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f    //�������ܵ�������
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f    //���ʵ�������
fp32 current_scale;
/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */

		
void chassis_power_control(chassis_move_t *chassis_power_control)
{
	float power_limit,warning_power;

	if(ext_robot_status.robot_level==1)
	{
		if(ext_robot_status.chassis_power_limit == 70)
		{
			warning_power = 55.0;
		}
		else
		{
			warning_power = 45.0;
		}
	}
	if(ext_robot_status.robot_level==2)
	{
		if(ext_robot_status.chassis_power_limit == 90)
		{
			warning_power = 70.0;
		}
		else
		{
			warning_power = 50.0;
		}
	}
	if(ext_robot_status.robot_level==3)
	{
		if(ext_robot_status.chassis_power_limit == 120)
		{
			warning_power = 100.0;
		}
		else
		{
			warning_power = 55.0;
		}
	}
	
	power_limit=ext_robot_status.chassis_power_limit;
	
	
	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;
	fp32 total_current_limit = 0.0f;
	fp32 total_current = 0.0f;
	
    uint8_t robot_id = get_robot_id();
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        //���ʳ���80w �ͻ�������С��warning buff,��Ϊ��������С��warning buff��ζ�Ź��ʳ���85
        if(chassis_power_buffer < WARNING_POWER_BUFF)
        {
            fp32 power_scale;
//            if(chassis_power_buffer > 5.0f)
            {
                //scale down WARNING_POWER_BUFF
                //��СWARNING_POWER_BUFF
                power_scale = chassis_power_buffer / WARNING_POWER_BUFF;

            }
            //scale down
            //��С
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
        }
        else//������������50
        {
            //power > warning_power
            //���ʴ���warning_power
            if(chassis_power > warning_power)
            {
                fp32 power_scale;
                //power < 80w
                //����С��80w
                if(chassis_power < power_limit)
                {
                    //scale down
                    //��С
                    power_scale = (power_limit - chassis_power) / (power_limit - warning_power);
                }
                //power > 80w
                //���ʴ���80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //power < warning_power
            //����С��warning_power
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        } 
    }

    
    total_current = 0.0f;
    //calculate the original motor current set
    //����ԭ����������趨
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
    
    if(total_current > total_current_limit)
    {
        current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
	//UART_DMA_SEND(ext_power_heat_data.chassis_power_buffer);
}
