/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "math.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define SERVO_UP_PWM 1250
#define SERVO_DOWN_PWM 1525
#define pi acos(-1)

#define SERVO_KEY  KEY_PRESSED_OFFSET_R

const RC_ctrl_t *servo_rc;
const gimbal_control_t  *pitch;
fp32 absolute_angle;
servo_control position;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
   	pitch = get_gimbal_control_point();
	  position = servo_up;
    while(1)
    {
			absolute_angle = pitch->gimbal_pitch_motor.absolute_angle;
			if(absolute_angle <= 0)
			{
				while(1)
				{
			    servo_pwm_set( - absolute_angle * 550 + 1625 ,3); 
			    osDelay(50);
					break;
				}
			}
			
			
//      if((servo_rc->key.v & SERVO_KEY) == SERVO_KEY)
//			 {
//			 if((servo_rc->rc.s[0])==3)
//			  {
//				  if(position == servo_up)
//					{
//					  position = servo_down;
//					  servo_pwm_set(SERVO_DOWN_PWM,3);
//						osDelay(1000);
//					}
//			    else if(position == servo_down)
//					{
//			      position = servo_up;
//					  servo_pwm_set(SERVO_UP_PWM,3);
//				    osDelay(1000);
//					}
//				}
//			    else
//						{
//			      }
//				//}
		}
}


