/**
  ****************(C) COPYRIGHT 2021 TJRM_SUPERPOWER****************************
  * @file       supercap_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     SEPT-2-2021     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
	****************(C) COPYRIGHT 2021 TJRM_SUPERPOWER****************************
	*/
#include "string.h"
#include "cmsis_os.h"

#include "supercap_task.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "referee.h"
#include "chassis_behaviour.h"
#include "usart_debug.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t supercap_high_water;
#endif

int32_t Power_Limitation_Num=55000;
int32_t Residue_Power=55000/25;//55W
int32_t Last_reside_power=0;
int8_t CAN2_Cap_flag=0;
int8_t Cap_Switch_Flag=0;
int8_t cap_on=1;

int32_t Lost_Connection_Count=0;
uint8_t Cap_Switch_Count=0;

int32_t Cap_Toutuous_Uppest=25000;
int32_t Cap_Toutuous_Up=24000;
int32_t Cap_Toutuous_Down=20000;

int32_t Chassis_Power=0;

cap_state cap_FSM;
cap_state cap_FSM_ex;
cap_control_t cap_control;
chassis_state_t chassis_state;

/**
  * @brief          超级电容初始化函数
  * @param[in]      pvParameters: 空
  * @retval         none
  */
static void Cap_init(cap_control_t *cap_control_init);

/**
  * @brief          超级电容控制函数，间隔 SUPERCAP_CONTROL_TIME 1ms
  * @param[in]      cap_control_init: cap_control_t变量指针
  * @retval         none
  */
static void Cap_Contorl(cap_control_t *cap_control);

/**
  * @brief          超级电容任务，间隔 SUPERCAP_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void supercap_task(void const * argument)
{
	vTaskDelay(SUPERCAP_TASK_INIT_TIME);
	Cap_init(&cap_control);
	static const fp32 Cap_pid[3] = {CAP_PID_KP, CAP_PID_KI, CAP_PID_KD};
	PID_init(&cap_control.cap_charge_pid, PID_POSITION, Cap_pid, CAP_PID_MAX_OUT, CAP_PID_MAX_IOUT);
	
	while(1)
	{
		Cap_Contorl(&cap_control);	//选择运行模式
		CAN_cmd_supercap(cap_FSM);	//发送运行模式通道
		cap_FSM_ex = cap_FSM;	//状态机更新

		vTaskDelay(SUPERCAP_CONTROL_TIME);
		
#if INCLUDE_uxTaskGetStackHighWaterMark		//1
        supercap_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**
  * @brief          超级电容初始化函数
  * @param[in]      cap_control_init: cap_control_t变量指针
  * @retval         none
  */
static void Cap_init(cap_control_t *cap_control_init)
{
	if (cap_control_init == NULL)
    {
        return;
    }
		
	//获取遥控器指针
    cap_control_init->cap_rc = get_remote_control_point();
	cap_control_init->cap_message = get_cap_measure_point();
}


/**
  * @brief          超级电容控制函数，间隔 SUPERCAP_CONTROL_TIME 1ms
  * @param[in]      cap_control_init: cap_control_t变量指针
  * @retval         none
  */
static void Cap_Contorl(cap_control_t *cap_control)
{

	get_chassis_power_and_buffer(&chassis_state.Chassis_power, &chassis_state.chassis_power_buffer);
	chassis_state.Chassis_power_limit=ext_game_robot_status.chassis_power_limit;
	chassis_state.remain_power=chassis_state.Chassis_power_limit-chassis_state.Chassis_power;
	//chassis_state.remain_buffer=chassis_state.chassis_power_buffer;
	if(ext_game_robot_status.mains_power_chassis_output==0)//如果底盘被裁判系统断电，电容不放电
	{
		if(cap_control->cap_message->cap_vol<Cap_Toutuous_Uppest)
			cap_FSM=cap_dis_charge;//只充不放
		else
			cap_FSM=cap_dis_ucharge;//不充不放
		cap_on=0;
	}
	else
		cap_on=1;

	
	cap_control->cap_charge_set=chassis_state.Chassis_power_limit-8;

	if(((cap_control->cap_rc->key.v & KEY_PRESSED_OFFSET_W)== KEY_PRESSED_OFFSET_W)||((cap_control->cap_rc->key.v & KEY_PRESSED_OFFSET_S)== KEY_PRESSED_OFFSET_S)
		||spinning_state)
		cap_control->cap_charge_set*=0.3f;//运动中降低充电能力
	if(chassis_state.chassis_power_buffer<=30)
		cap_control->cap_charge_set*=0.75f;//防止充电超功率
	

	if(cap_control->cap_message->cap_vol<25000)
	{
		PID_calc(&cap_control->cap_charge_pid,chassis_state.Chassis_power,cap_control->cap_charge_set);
		Residue_Power=cap_control->cap_charge_pid.out;
	}
	if(chassis_state.chassis_power_buffer<=10 ||Residue_Power<0)
	{
		Residue_Power=0;
	}
	
	if(cap_on==1&&SUPER_CAP_ON)
	{
		if(switch_is_mid(cap_control->cap_rc->rc.s[CHASSIS_MODE_CHANNEL])||switch_is_up(cap_control->cap_rc->rc.s[CHASSIS_MODE_CHANNEL]))  //右边拨杆处于中间或者上面
		{
			if(cap_control->cap_message->cap_vol>26000)//电容电量26000
			{
				cap_FSM=cap_en_ucharge;//不充但放
			}
			else
			{
				if((cap_control->cap_rc->key.v & KEY_PRESSED_OFFSET_F ) == KEY_PRESSED_OFFSET_F ||(cap_control->cap_rc->rc.ch[4]>4500))
				{
					cap_FSM=cap_en_ucharge;
				}
				else{
					cap_FSM=cap_dis_charge;//不放但充
				}
			}
		}
		
		else if(switch_is_down(cap_control->cap_rc->rc.s[CHASSIS_MODE_CHANNEL]))//down
		{
			if(cap_control->cap_message->cap_vol<Cap_Toutuous_Uppest)
			{
				cap_FSM = cap_dis_charge;//电容电量小于25000 充
			}
			else//电容电量大于25000
			{
				cap_FSM = cap_dis_ucharge;//不充电
				CAN2_Cap_flag=0;
			}
		}
		
		
		else//读不到遥控器值？
		{
			cap_FSM = cap_dis_ucharge;//不充电且用电池放电
		}
	}

}

