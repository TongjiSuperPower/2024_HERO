

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
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

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
static void fric_control_loop(void);

shoot_control_t shoot_control;          //射击数据
fric_motor_t fric_left_motor;			//摩擦轮数据
fric_motor_t fric_right_motor;
int fric_right_motor_last_speed=0;

static uint16_t cooling_heat;
static uint16_t cooling_heat_limit;
static uint16_t remain_times;


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
		static const fp32 fric_left_speed_pid[3] = {FRIC_LEFT_SPEED_PID_KP, FRIC_LEFT_SPEED_PID_KI, FRIC_LEFT_SPEED_PID_KD};
		static const fp32 fric_right_speed_pid[3] = {FRIC_RIGHT_SPEED_PID_KP, FRIC_RIGHT_SPEED_PID_KI, FRIC_RIGHT_SPEED_PID_KD};	
			
		static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		fric_left_motor.fric_motor_measure = get_fricl_motor_measure_point();
		fric_right_motor.fric_motor_measure = get_fricr_motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_init(&fric_left_motor.fric_motor_pid, PID_POSITION, fric_left_speed_pid, FRIC_LEFT_PID_MAX_OUT, FRIC_LEFT_PID_MAX_OUT);
		PID_init(&fric_right_motor.fric_motor_pid, PID_POSITION, fric_right_speed_pid, FRIC_RIGHT_PID_MAX_OUT, FRIC_RIGHT_PID_MAX_OUT);		
		
		PID_init(&shoot_control.trigger_angle_motor_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		
    //更新数据
    shoot_feedback_update();

    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE19;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
		shoot_control.move_flag2 = 0;
    shoot_control.set_angle = shoot_control.shoot_motor_measure->relative_angle_19laps;
		shoot_control.angle_begin = shoot_control.shoot_motor_measure->relative_angle_19laps;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		shoot_control.move_time = 0;
		shoot_control.v_key = 0;
		shoot_control.last_v_key = 0;
}

void fric_control_loop(void)
{
	fric_left_motor.speed = fric_left_motor.fric_motor_measure->speed_rpm;
	fric_right_motor.speed = fric_right_motor.fric_motor_measure->speed_rpm;
	/**速度反馈值滤波待补全**/
	if (shoot_control.fric_state == FRIC_ON)
	{
		fric_left_motor.speed_set = -FRIC_SPEED;
		fric_right_motor.speed_set = FRIC_SPEED;
		if((shoot_control.shoot_rc->key.v & HEAT_FRIC_KEY) && fric_right_motor.fric_motor_measure->temperate <= 47.0f)
		{
			fric_left_motor.speed_set = -20000;
			fric_right_motor.speed_set = 20000;
		}
//		fric_left_motor.speed_set = -5350;
//		fric_right_motor.speed_set = 5350;
		
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
  * @brief          射击循环(！！！主函数！！！)
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据


		fric_right_motor_last_speed=fric_right_motor.speed;
	
		//根据状态机选取进行下一步操作
    if (shoot_control.shoot_mode == SHOOT_STOP) //stop模式代表gimbal出现某些问题，例如在遥控器down状态、校准阶段等，需要全面停止射击系统
    {
			shoot_control.move_flag2 = 0;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
			if(shoot_control.move_flag2 == 0)
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

	
    else if(shoot_control.shoot_mode == SHOOT_DONE)//down模式表示完成一次射击，在这一过程中
    {
        shoot_control.set_angle = rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps-PI_THREE/10.0f);
    }


		
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
				shoot_control.fric_state = FRIC_OFF;		
//        shoot_laser_off();
        shoot_control.given_current = 0;
    }

    else
    {
				shoot_control.fric_state = FRIC_ON;
//        shoot_laser_on(); 
        //计算拨弹轮电机PID

				shoot_control.speed_set = shoot_PID_calc(&shoot_control.trigger_angle_motor_pid,rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps),rad_format(shoot_control.set_angle), shoot_control.speed);
				trigger_motor_turn_back();
				shoot_control.given_current = PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
				if(shoot_control.shoot_rc->key.v & CRAZY_SHOOT_KEY)
				{
					shoot_control.speed_set = 8000;
					shoot_control.given_current = PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
				}
		

        if(shoot_control.shoot_mode < SHOOT_READY_FRIC)//_BULLET)
        {
            shoot_control.given_current = 0;
        }

    }
		fric_control_loop();
    
		return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
		shoot_control.v_key = shoot_control.shoot_rc->key.v & AUTOAIM_SHOOT_KEY;


    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;				
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		
		//开启摩擦轮后
		if(shoot_control.shoot_mode == SHOOT_READY_FRIC)  
		{
					//下拨一次或者鼠标按下一次，进入射击状态
			if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (get_autoaim_flag() == AUTOAIM_FIRE_FLAG && (shoot_control.press_r) && shoot_control.v_key))
			{
				shoot_control.shoot_mode = SHOOT_BULLET;//射击状态
				cooling_heat=ext_power_heat_data.shooter_id1_42mm_cooling_heat;				//当前枪口热量
				cooling_heat_limit=ext_game_robot_status.shooter_id1_42mm_cooling_limit;	//枪口热量上限
				
				if(cooling_heat_limit - cooling_heat < 100.0)		//如果当前枪口热量小于发射一颗所需热量上限，则返回到ready状态
				{
					shoot_control.shoot_mode = SHOOT_READY_FRIC;
				}				
			}
		}
		
		//射击完成
    else if(shoot_control.shoot_mode == SHOOT_DONE)
		{
			shoot_control.move_flag = 0;
			shoot_control.block_time = 0;
			shoot_control.key_time++;
			if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)  //手动延时一段时间再允许进入ready模式，以防止误触
			{
				shoot_control.key_time = 0;
				shoot_control.shoot_mode = SHOOT_READY_FRIC;
			}
	}

    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];//上一时刻的左拨杆值
		shoot_control.last_v_key = shoot_control.v_key;
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
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

    //射击开关下档时间计时
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
    if( shoot_control.block_time >= BLOCK_TIME) //堵转一定时间后，给定反转速度
    {
        shoot_control.speed_set = -1.5;
    }
		
		if(fabs(shoot_control.speed) >= BLOCK_TRIGGER_SPEED)
		{}// 判断速度是否足够小，可以认为是卡弹
			
		//如果是：并且处于射击模式	
		else if(shoot_control.shoot_mode == SHOOT_BULLET)
		{
			if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME) //判断堵转时间
			{
					shoot_control.block_time++;
					shoot_control.reverse_time = 0; //没有确定是堵转，则不发生反转，反转时间清零
			}
			else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME) //确定是堵转，开始记录反转时间
			{
					shoot_control.reverse_time++;
			}
			else //反转时间超限，认为无法反转，进入down模式，重新开始开始准备下次触发
			{
				shoot_control.block_time = 0;
				shoot_control.shoot_mode = SHOOT_DONE;
			}
		}
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
	fric_right_motor_last_speed=fric_right_motor.speed;
	
    //每次拨动 1/3PI的角度

	if (shoot_control.move_flag == 0) //置零表示可以改变目标角度，说明已进入过down模式，并且flag清零，可以开始下一次拨弹
  {
			shoot_control.set_angle = rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps + PI_THREE*4.72f/3.0f);
			shoot_control.angle_begin = shoot_control.set_angle;

			shoot_control.move_flag = 1; //改变目标角度后即改变flag，防止在当前轮拨弹过程中目标值发生变化
  }
	
	//摩擦轮转速下降，打出一发，认为拨弹结束
	if(fric_right_motor.fric_motor_measure->speed_rpm<(FRIC_SPEED-200))  
	{
			shoot_control.shoot_mode = SHOOT_DONE; 
	}
	//到达角度判断，如果到达目标角度附近则认为拨弹结束
	if((rad_format(shoot_control.set_angle - shoot_control.shoot_motor_measure->relative_angle_19laps) < 0.1f) && (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED))
	{
			shoot_control.shoot_mode = SHOOT_DONE;
	}
}






