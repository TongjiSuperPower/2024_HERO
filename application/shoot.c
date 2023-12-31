

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

void hero_shoot_bullet_control(void);



int shoot_derta=0;
//pid_type_def trigger_angle_motor_pid;
int shoot_i = 0;
int shoot_flag = 0;
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
	shoot_control.z_key = 0;
	shoot_control.last_z_key = 0;
}

void fric_control_loop(void)
{
	fric_left_motor.speed = fric_left_motor.fric_motor_measure->speed_rpm;
	fric_right_motor.speed = fric_right_motor.fric_motor_measure->speed_rpm;
	/**速度反馈值滤波待补全**/
	if (shoot_control.fric_state == FRIC_ON)
	{
		fric_left_motor.speed_set = -5650;
		fric_right_motor.speed_set = 5650;
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
//	UART_DMA_SEND(fric_right_motor.speed);
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据

/*第一个if判断 用于根据模式设定拨弹轮的set数据（位置环/速度环）*/
	fric_right_motor_last_speed=fric_right_motor.speed;
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
//        shoot_control.speed_set = 0.0f;
		shoot_control.debug_flag = 1;
		shoot_control.move_flag2 = 0;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
		if(shoot_control.move_flag2 == 0)
		{
			shoot_control.set_angle = shoot_control.shoot_motor_measure->relative_angle_19laps;
		}
		else
		{
			shoot_control.speed_set = 0.0f;
			shoot_control.debug_flag = 2;
		}
		
    }
		
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
		shoot_control.move_flag2 = 1;
		shoot_control.debug_flag = 3;
        shoot_control.trigger_angle_motor_pid.max_out =  10000.0f;
        shoot_control.trigger_angle_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }

	
	
	
	
	
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
				shoot_control.debug_flag = 5;
        shoot_control.set_angle = rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps-PI_THREE/10.0f);
    }

/*第二个if判断 只区分是否down 如果不down掉就进行控制循环*/
		
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
				shoot_control.fric_state = FRIC_OFF;		
        shoot_laser_off();
        shoot_control.given_current = 0;
    }
		//------gamggaide-------
//		else if((shoot_control.shoot_mode == SHOOT_BULLET) &&(shoot_control.block_time < BLOCK_TIME))
//		{
//				PID_calc(&trigger_angle_motor_pid,shoot_delta, 1365.3*19/3);
//        shoot_control.given_current = (int16_t)(trigger_angle_motor_pid.out);
//		}

    else
    {
		shoot_control.fric_state = FRIC_ON;
        shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
//        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
//        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

//			if(shoot_i==0)
//			{
//		

		shoot_control.speed_set = shoot_PID_calc(&shoot_control.trigger_angle_motor_pid,rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps),rad_format(shoot_control.set_angle), shoot_control.speed);
		trigger_motor_turn_back();
		shoot_control.given_current = PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
		if(shoot_control.shoot_rc->key.v & CRAZY_SHOOT_KEY)
		{
			shoot_control.speed_set = 8000;
			shoot_control.given_current = PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
		}
		
		
		
//				shoot_flag = 1;
//			}
//			if(shoot_flag)
//			{
//				shoot_i++;
//			}
//			if(shoot_i == 3000)
//			{
//				shoot_i=0;
//				shoot_flag = 0;
//			}
////		


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
	shoot_control.z_key = shoot_control.shoot_rc->key.v & AUTOAIM_SHOOT_KEY;


    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;				
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

	if(shoot_control.shoot_mode == SHOOT_READY_FRIC)  
    {
        //下拨一次或者鼠标按下一次，进入射击状态
		if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (get_autoaim_flag() == AUTOAIM_FIRE_FLAG && (shoot_control.press_r) && shoot_control.z_key))
		{
			shoot_control.shoot_mode = SHOOT_BULLET;
			cooling_heat=ext_power_heat_data.shooter_id1_42mm_cooling_heat;				//当前枪口热量
			cooling_heat_limit=ext_game_robot_status.shooter_id1_42mm_cooling_limit;	//枪口热量上限
			if(cooling_heat_limit - cooling_heat < 100.0)
			{
				shoot_control.shoot_mode = SHOOT_READY_FRIC;
			}
			
			
		}
		//上位机发送打弹信息
//		if(get_autoaim_flag() == AUTOAIM_FIRE_FLAG)
//		{		
//			shoot_control.shoot_mode = SHOOT_BULLET;
//		}
	}
    else if(shoot_control.shoot_mode == SHOOT_DONE)
	{
		shoot_control.block_time = 0;
		shoot_control.key_time++;
		if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
		{
			shoot_control.key_time = 0;
			shoot_control.shoot_mode = SHOOT_READY_FRIC;
			
		}
	}
//	if(shoot_control.shoot_mode >= SHOOT_READY_FRIC)
//    {
//        //鼠标长按一直进入射击状态 保持连发
//        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
//        {
//            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//        }
//        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
//        {
//            shoot_control.shoot_mode =SHOOT_READY_FRIC;//BULLET;
//        }
//    }
/*
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_FRIC;//BULLET;
        }
    }*/
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
	shoot_control.last_z_key = shoot_control.z_key;
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

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count--;
//    }
//    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
//    {
//        shoot_control.ecd_count++;
//    }

//    if (shoot_control.ecd_count == FULL_COUNT)
//    {
//        shoot_control.ecd_count = -(FULL_COUNT - 1);
//    }
//    else if (shoot_control.ecd_count == -FULL_COUNT)
//    {
//        shoot_control.ecd_count = FULL_COUNT - 1;
//    }
	
	
//	if (shoot_control.ecd_count == 19)
//    {
//        shoot_control.ecd_count = 0;
//    }

	
	

    //计算输出轴角度
//	shoot_control.angle = shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd;
//	while(shoot_control.angle>=(8192.0f*3592.0f/187.0f))
//	{
//		shoot_control.angle -= 8192.0f*3592.0f/187.0f;
//	}
//	shoot_control.angle = rad_format(shoot_control.angle*MOTOR_ECD_TO_ANGLE);
	
//	
//    shoot_control.angle = rad_format((shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE);
//	shoot_control.angle = rad_format((shoot_control.shoot_motor_measure->ecd) * 2.0f*3.1415926f/8192.0f);//MOTOR_ECD_TO_ANGLE;
    //微动开关
    shoot_control.key = BUTTEN_TRIG_PIN;
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

    //鼠标右键按下加速摩擦轮，使得左键低速射击，右键高速射击
    static uint16_t up_time = 0;
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;
    }
    if (up_time > 0)
    {
//        shoot_control.fric1_ramp.max_value = FRIC_UP;
//        shoot_control.fric2_ramp.max_value = FRIC_UP;
        up_time--;
    }
    else
    {
//        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
//        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    }
}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
//		if(shoot_control.move_flag == 0)
        shoot_control.speed_set = shoot_control.speed_set;
//		shoot_control.set_angle = shoot_control.set_angle;
    }
    else
    {
        shoot_control.speed_set = -1.5;//-3.0;
//		if(shoot_control.move_flag2 == 0 )
//		{
//			shoot_control.set_angle = rad_format(rad_format(shoot_control.set_angle) - 2*PI_THREE);
//			shoot_control.move_flag2 = 1;
//		
//		}
    }
	if(fabs(shoot_control.speed) >= BLOCK_TRIGGER_SPEED){
	}
	else if(shoot_control.shoot_mode == SHOOT_BULLET)
	{
		if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
		{
			
				shoot_control.block_time++;
			
				shoot_control.reverse_time = 0;
			
		}
		else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
		{
			
				shoot_control.reverse_time++;
			
		}
		else
		{
			shoot_control.block_time = 0;
			shoot_control.shoot_mode = SHOOT_DONE;
			shoot_control.move_flag = 0;
	//		shoot_control.move_flag2 = 0;
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
//	shoot_control.move_time++;
	
    //每次拨动 1/3PI的角度

	if (shoot_control.move_flag == 0) 
    {
		shoot_control.set_angle = rad_format(shoot_control.shoot_motor_measure->relative_angle_19laps + PI_THREE*4.72f/3.0f);
		
		shoot_control.angle_begin = shoot_control.set_angle;
//        shoot_control.move_flag2 ++;
        shoot_control.move_flag = 1;
    }
    if(fric_right_motor.fric_motor_measure->speed_rpm<5450)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
		shoot_control.move_flag = 0;
    }
//    //到达角度判断
	if((rad_format(shoot_control.set_angle - shoot_control.shoot_motor_measure->relative_angle_19laps) < 0.1f) && (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED))
    {
        shoot_control.shoot_mode = SHOOT_DONE;
		shoot_control.move_flag = 0;
    }
    if (rad_format(shoot_control.set_angle - shoot_control.shoot_motor_measure->relative_angle_19laps) > 0.1f)// //yuanlai0.05
    {
//        shoot_control.trigger_speed_set = TRIGGER_SPEED;
//        trigger_motor_turn_back();
//		if(shoot_control.move_flag == 1 && shoot_control.move_flag2 ==0)
//		{
//			if(shoot_control.move_time == MOVE_TIME)
//			{
//				shoot_control.move_flag = 0;
//			}
//		}
    }
//    else
//    {
//        shoot_control.move_flag = 0;
////		shoot_control.move_flag2 = 0;
////		shoot_control.block_time = 0;
////		shoot_control.move_time = 0;

//    }
	
	
	
}






