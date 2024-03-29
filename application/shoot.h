/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
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

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

#define New_Hero

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

////射击摩擦轮激光打开 关闭
//#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     150
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#ifndef HALF_ECD_RANGE
#define HALF_ECD_RANGE              4096
#endif
#ifndef ECD_RANGE
#define ECD_RANGE                   8191
#endif
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE19          0.000039929622401811f//0.00004036791547f//0.000021305288720633905968306772076277f
#define FULL_COUNT                  10//18
//拨弹速度
#define TRIGGER_SPEED               0.2f//5.0f
#define CONTINUE_TRIGGER_SPEED      5.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  1500//700
#define REVERSE_TIME                500//800//500
#define REVERSE_SPEED_LIMIT         13.0f
#define MOVE_TIME                   1500
#define FRIC_SPEED                  5650


#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f
#define PI_SEVEN					0.897f
#define PI_THREE						1.047197551196597746f//1.02f

#ifdef Old_Hero
//拨弹轮电机PID
#define TRIGGER_SPEED_PID_KP        500.0f
#define TRIGGER_SPEED_PID_KI        0.0f
#define TRIGGER_SPEED_PID_KD        0.0f

#define TRIGGER_ANGLE_PID_KP        40.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        -5.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

#endif

#ifdef New_Hero                                     //新英雄！！的拨弹轮pid
#define TRIGGER_SPEED_PID_KP        400.0f
#define TRIGGER_SPEED_PID_KI        0.0f
#define TRIGGER_SPEED_PID_KD        0.0f

#define TRIGGER_ANGLE_PID_KP        78.3f//85.0//79.0f//78.3f//70.0f//73.0f//66.0f//40.0f//37.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        -6.5f//-6.0f//-5.0f//-4.1f

#define TRIGGER_BULLET_PID_MAX_OUT  16384.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   16384.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

#endif

#define FRIC_LEFT_SPEED_PID_KP      32.0f
#define FRIC_LEFT_SPEED_PID_KI		2.5f
#define FRIC_LEFT_SPEED_PID_KD		0.5f
#define FRIC_RIGHT_SPEED_PID_KP		15.5f
#define FRIC_RIGHT_SPEED_PID_KI     2.0f
#define FRIC_RIGHT_SPEED_PID_KD     0.3f

#define FRIC_LEFT_PID_MAX_OUT   30000.0f
#define FRIC_LEFT_PID_MAX_IOUT  5000.0f
#define FRIC_RIGHT_PID_MAX_OUT   30000.0f
#define FRIC_RIGHT_PID_MAX_IOUT  5000.0f


#define SHOOT_HEAT_REMAIN_VALUE     80

#define AUTOAIM_SHOOT_KEY   KEY_PRESSED_OFFSET_V
#define CRAZY_SHOOT_KEY     KEY_PRESSED_OFFSET_B
#define HEAT_FRIC_KEY       KEY_PRESSED_OFFSET_G

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
//    SHOOT_READY_BULLET,
//    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,//英雄不能连续射击
    SHOOT_DONE,
} shoot_mode_e;

typedef enum
{
	FRIC_OFF = 0,
	FRIC_ON = 1,
} fric_state_e;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	pid_type_def fric_motor_pid;
} fric_motor_t;




typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
		pid_type_def trigger_angle_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;
	
	
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;
	
    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;
		bool_t move_flag2;
    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
		fric_state_e fric_state;
		uint8_t debug_flag;
		int16_t last_v_key;
		int16_t v_key;
		int16_t autoaim_mode;
		bool_t shoot_flag;
		bool_t last_shoot_flag;
		autoaim_data_t *autoaim_data;
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行

extern void shoot_init(void);
extern int16_t shoot_control_loop(void);




#endif
