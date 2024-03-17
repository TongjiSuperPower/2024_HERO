

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������?.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���?
  *	 V2.0.0			Jan-14-2024			lyf							2024HERO
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "usart_debug.h"
#include "autoaim.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off()    fric_off()      //�ر�����Ħ����


//��ʱû��//
//--------------------------------------------------------------------------------
#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��?
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)
//--------------------------------------------------------------------------------


/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�?1�η���1��
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���?
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���?
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
static void fric_control_loop(void);

shoot_control_t shoot_control;          //�������?
fric_motor_t fric_left_motor;			//Ħ��������
fric_motor_t fric_right_motor;
int fric_right_motor_last_speed=0;

static uint16_t cooling_heat;
static uint16_t cooling_heat_limit;
//static uint16_t remain_times;


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��?
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
		static const fp32 fric_left_speed_pid[3] = {FRIC_LEFT_SPEED_PID_KP, FRIC_LEFT_SPEED_PID_KI, FRIC_LEFT_SPEED_PID_KD};
		static const fp32 fric_right_speed_pid[3] = {FRIC_RIGHT_SPEED_PID_KP, FRIC_RIGHT_SPEED_PID_KI, FRIC_RIGHT_SPEED_PID_KD};	
			
		static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��?
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		fric_left_motor.fric_motor_measure = get_fricl_motor_measure_point();
		fric_right_motor.fric_motor_measure = get_fricr_motor_measure_point();
    //��ʼ��PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_init(&fric_left_motor.fric_motor_pid, PID_POSITION, fric_left_speed_pid, FRIC_LEFT_PID_MAX_OUT, FRIC_LEFT_PID_MAX_OUT);
		PID_init(&fric_right_motor.fric_motor_pid, PID_POSITION, fric_right_speed_pid, FRIC_RIGHT_PID_MAX_OUT, FRIC_RIGHT_PID_MAX_OUT);		
		
		PID_init(&shoot_control.trigger_angle_motor_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		
    //��������
    shoot_feedback_update();

    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE19;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
		shoot_control.move_flag2 = 0;
    shoot_control.set_angle = shoot_control.shoot_motor_measure->relative_angle_19laps;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		shoot_control.v_key = 0;
		shoot_control.last_v_key = 0;
		shoot_control.autoaim_mode = 1;
		shoot_control.shoot_flag = 0;
		shoot_control.last_shoot_flag = 0;
		shoot_control.autoaim_data = get_autoaim_data_point_changeable();
}

void fric_control_loop(void)
{
	fric_left_motor.speed = fric_left_motor.fric_motor_measure->speed_rpm;
	fric_right_motor.speed = fric_right_motor.fric_motor_measure->speed_rpm;
	/**�ٶȷ���ֵ�˲�����ȫ**/
	if (shoot_control.fric_state == FRIC_ON)
	{
		fric_left_motor.speed_set = -FRIC_SPEED;
		fric_right_motor.speed_set = FRIC_SPEED;
		if((shoot_control.shoot_rc->key.v & HEAT_FRIC_KEY) && fric_right_motor.fric_motor_measure->temperate <= 47.0f) //Ħ���ַ�ת����
		{
			fric_left_motor.speed_set = -20000;
			fric_right_motor.speed_set = 20000;
		}
	}
	else
	{
		fric_left_motor.speed_set = 0;
		fric_right_motor.speed_set = 0;
	}
    fric_left_motor.give_current = (int16_t)(PID_calc(&fric_left_motor.fric_motor_pid, fric_left_motor.speed, fric_left_motor.speed_set));		
    fric_right_motor.give_current = (int16_t)(PID_calc(&fric_right_motor.fric_motor_pid, fric_right_motor.speed, fric_right_motor.speed_set));		

	CAN_cmd_fric(fric_left_motor.give_current, fric_right_motor.give_current);
	UART_DMA_SEND(fric_right_motor.speed);
}

/**
  * @brief          ���ѭ��?(������������������)
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
		
		CAN_send_sth_to_computer(ext_shoot_data.initial_speed,shoot_control.autoaim_mode);

		fric_right_motor_last_speed=fric_right_motor.speed;
	
		
	//####################################//����״̬��ѡȡ������һ������:��ÿ�������Ŀ��Ƕȣ�//#######################################//
	
	
    if (shoot_control.shoot_mode == SHOOT_STOP) //stopģʽ����gimbal����ĳЩ���⣬������ң����down״̬��У׼�׶εȣ���Ҫȫ��ֹͣ���ϵ�?
    {
				shoot_control.move_flag2 = 0;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
				if(shoot_control.move_flag2 == 0)//�����̨ûɶ���⣬�򱣳ֵ�ǰλ�ã���������������ֹ�����е��������ƶ������ַ��?
				{
						shoot_control.set_angle = shoot_control.shoot_motor_measure->relative_angle_19laps;
				}
				else
				{
						shoot_control.speed_set = 0.0f;
				}
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
				shoot_control.move_flag2 = 1;
        shoot_control.trigger_angle_motor_pid.max_out =  10000.0f;
        shoot_control.trigger_angle_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)//downģʽ��ʾ���һ����������ڹ��ɣ�����һ�����н���һ��С�Ƕȷ�ת��Ŀ�����ò���������ɲ��������ֹͣ
    {
        shoot_control.set_angle = rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps-PI_THREE/10.0f);
    }

	//########################################################################################################//
		
		
	//####################################//����״̬��ѡȡ������һ�����������㷢�͵���ֵ//#######################################//

		
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
				shoot_control.fric_state = FRIC_OFF;		
        shoot_control.given_current = 0;
    }
    else
    {
				shoot_control.fric_state = FRIC_ON;
        //���㲦���ֵ��PID
				shoot_control.speed_set = shoot_PID_calc(&shoot_control.trigger_angle_motor_pid,rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps),rad_format(shoot_control.set_angle), shoot_control.speed);
				trigger_motor_turn_back();
				shoot_control.given_current = PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
				if(shoot_control.shoot_rc->key.v & CRAZY_SHOOT_KEY)
				{
						shoot_control.speed_set = 8000;
						shoot_control.given_current = PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
				}
		
        if(shoot_control.shoot_mode < SHOOT_READY_FRIC)
        {
            shoot_control.given_current = 0;
        }
    }
		
		fric_control_loop();
    
		return shoot_control.given_current;
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�?1�η���1��
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
		shoot_control.v_key = shoot_control.shoot_rc->key.v & AUTOAIM_SHOOT_KEY;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;				
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		
		//����Ħ���ֺ�
		if(shoot_control.shoot_mode == SHOOT_READY_FRIC)  
		{
					//�²�һ�λ�����갴��һ�Σ��������״̬
			if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.autoaim_data->shoot == AUTOAIM_FIRE_FLAG)) //&& (shoot_control.press_r) && shoot_control.v_key))
			{
				shoot_control.shoot_mode = SHOOT_BULLET;//���״�?
				cooling_heat=ext_power_heat_data.shooter_42mm_barrel_heat;				//��ǰǹ������
				cooling_heat_limit=ext_robot_status.shooter_barrel_heat_limit;	//ǹ����������
				shoot_control.autoaim_data->shoot = 0;
				if(cooling_heat_limit - cooling_heat < 100.0)		//�����ǰǹ������С�ڷ���һ�������������ޣ��򷵻ص�ready״̬
				{
					shoot_control.shoot_mode = SHOOT_READY_FRIC;
				}				
			}
		}
		
		//������
    else if(shoot_control.shoot_mode == SHOOT_DONE)
		{
			shoot_control.move_flag = 0;
			shoot_control.block_time = 0;
			shoot_control.key_time++;
			if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)  //�ֶ���ʱһ��ʱ������������readyģʽ���Է�ֹ��
			{
				shoot_control.key_time = 0;
				shoot_control.shoot_mode = SHOOT_READY_FRIC;
			}
		}
		
    //�����̨״̬��? ����״̬���͹ر����?
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];//��һʱ�̵��󲦸�ֵ
		shoot_control.last_v_key = shoot_control.v_key;
}
/**
  * @brief          ������ݸ���?
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��?
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //��갴��?
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time >= BLOCK_TIME) //��תһ��ʱ��󣬸�����ת�ٶ�?
    {
        shoot_control.speed_set = -0.5;
    }
		
		if(fabs(shoot_control.speed) >= BLOCK_TRIGGER_SPEED)
		{}// �ж��ٶ��Ƿ��㹻С��������Ϊ�ǿ���
			
		//����ǣ����Ҵ������ģʽ	
		else if(shoot_control.shoot_mode == SHOOT_BULLET)
		{
			if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME) //�ж϶�תʱ��
			{
					shoot_control.block_time++;
					shoot_control.reverse_time = 0; //û��ȷ���Ƕ�ת���򲻷�����ת����תʱ������
			}
			else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME) //ȷ���Ƕ�ת����ʼ��¼��תʱ��
			{
					shoot_control.reverse_time++;
			}
			else //��תʱ�䳬�ޣ���Ϊ�޷���ת������downģʽ�����¿�ʼ��ʼ׼���´δ���
			{
				shoot_control.block_time = 0;
				shoot_control.shoot_mode = SHOOT_DONE;
			}
		}
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���?
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    //ÿ�β������� 1/3PI �ĽǶȣ�ʹ����һ���ܴ��ȥ������һ���ܵ���Ŀ��ֵ��ͨ��down�з�ת����ֹͣ��ֹ˫��

	if (shoot_control.move_flag == 0) //�����ʾ���Ըı�Ŀ��Ƕȣ�˵���ѽ����downģʽ������flag���㣬���Կ�ʼ��һ�β���
  {
			shoot_control.set_angle = rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps + PI_THREE*4.72f/3.0f*66.0f/38.0f);
			shoot_control.move_flag = 1; //�ı�Ŀ��ǶȺ󼴸ı�flag����ֹ�ڵ�ǰ�ֲ���������Ŀ��ֵ�����仯
  }
		//Ħ����ת���½������һ������Ϊ��������?
	if(fric_right_motor.fric_motor_measure->speed_rpm<(FRIC_SPEED-200))  
	{
			shoot_control.shoot_mode = SHOOT_DONE; 
	}
	//����Ƕ��жϣ��������Ŀ��Ƕȸ�������Ϊ��������?
	if((rad_format(shoot_control.set_angle - shoot_control.shoot_motor_measure->relative_angle_19laps) < 0.1f) && (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED))
	{
			shoot_control.shoot_mode = SHOOT_DONE;
	}
}






