/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "supercap_task.h"
#include "chassis_task.h"
#include "arm_math.h"



//static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
//{
//    int32_t relative_ecd = ecd - offset_ecd;
//    if (relative_ecd >= 0 && relative_ecd <=8191)
//    {
//        
//    }
//    else
//    {
//        relative_ecd += 8192;
//    }

//    return relative_ecd;
//}

int count_i = 0;
int start_up_i = 0;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
				(ptr)->last_last_last_ecd = (ptr)->last_last_ecd;               \
				(ptr)->last_last_ecd = (ptr)->last_ecd;                         \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
//cap data read
static void get_cap_data(supercap_module_receive *ptr, uint8_t *data)
{
		(ptr)->charge_enabled_state = (data)[0] & 0x0001;
		(ptr)->residue_power = (data)[1];
		(ptr)->charge_power  = (uint16_t)((data)[2]<<8 | (data)[3])*25;
		(ptr)->chassis_power = (int16_t) ((data)[4]<<8 | (data)[5])*25;
		(ptr)->cap_vol 			= (int16_t) ((data)[6]<<8 | (data)[7])*1.25;
}


void get_autoaim_data(autoaim_data_t *ptr, uint8_t data[8])
{
	ptr->control = data[0];
	ptr->shoot = (data[1]==1);
	ptr->yaw = (int16_t)(data[2] << 8 | data[3]) / 1e4f;
	ptr->pitch = (int16_t)(data[4] << 8 | data[5]) / 1e4f;
}


//				SCM_rx_message.charge_enabled_state = rx_data[0] & 0x0001;
//				SCM_rx_message.residue_power = rx_data[1];
//				SCM_rx_message.charge_power  = (uint16_t)(rx_data[2]<<8 | rx_data[3])*25;
//				SCM_rx_message.chassis_power = (int16_t) (rx_data[4]<<8 | rx_data[5])*25;
//				SCM_rx_message.cap_vol 			 = (int16_t) (rx_data[6]<<8 | rx_data[7])*1.25;
		
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
						4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机, 2:底盘电机3 3508电机, 3:底盘电机4 3508电机;
					4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机;  6:拨弹电机 2006/3508电机
					7:左摩擦轮 3508电机（去减速箱） 8: 右摩擦轮 3508电机（去减速箱） */
static motor_measure_t motor_chassis[9];
//supercap data
supercap_module_receive SCM_rx_message;

static CAN_TxHeaderTypeDef  fric_tx_message;
static uint8_t              fric_can_send_data[8];
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  supercap_tx_message;
static uint8_t              supercap_can_send_data[8];

static autoaim_data_t autoaim_data;
static CAN_TxHeaderTypeDef  autoaim_tx_message;
static uint8_t              autoaim_send_data[8];


CAN_RxHeaderTypeDef rxMsg;
CAN_TxHeaderTypeDef txMsg;
uint8_t rx_data2[8];       //接收数据
uint32_t Motor_Can_ID;    //接收数据电机ID
uint8_t byte[4];          //转换临时数据
uint32_t send_mail_box = {0};//NONE
    
#define can_txd() HAL_CAN_AddTxMessage(&hcan1, &txMsg, tx_data, &send_mail_box)//CAN发送宏定义

motor_measure_t mi_motor[1];//预先定义小米电机



/**
  * @brief          浮点数转4字节函数
  * @param[in]      f:浮点数
  * @retval         4字节数组
  * @description  : IEEE 754 协议
  */
static uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}

/**
  * @brief          小米电机回文16位数据转浮点
  * @param[in]      x:16位回文
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
  * @brief          小米电机发送浮点转16位数据
  * @param[in]      x:浮点
  * @param[in]      x_min:对应参数下限
  * @param[in]      x_max:对应参数上限
  * @param[in]      bits:参数位数
  * @retval         返回浮点值
  */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if(x > x_max) x=x_max;
  else if(x < x_min) x= x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          电机回复帧数据处理函数
  * @param[in]      Motor:对应控制电机结构体   
  * @param[in]      DataFrame:数据帧
  * @param[in]      IDFrame:扩展ID帧
  * @retval         None
  */
static void Motor_Data_Handler(motor_measure_t *Motor,uint8_t DataFrame[8],uint32_t IDFrame)
{	
		Motor->last_ecd = Motor->ecd;
		Motor->ecd=DataFrame[0]<<8|DataFrame[1];
		Motor->speed_rpm=uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
		Motor->Torque=uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
		Motor->Temp=(DataFrame[6]<<8|DataFrame[7])*Temp_Gain;
		Motor->error_code=(IDFrame&0x1F0000)>>16;	
}
		
/**
  * @brief          提取电机回复帧扩展ID中的电机CANID
  * @param[in]      CAN_ID_Frame:电机回复帧中的扩展CANID   
  * @retval         电机CANID
  */
static uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame)
{
	return (CAN_ID_Frame&0xFFFF)>>8;
}

		
		
	





/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */

CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if (hcan == &hcan2)
	{
		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + i);
				
				break;
			}
			
			case CAN_YAW_MOTOR_ID:
			{
				
				get_motor_measure(&motor_chassis[4], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + 4);
				
				break;
			}
			case CAN_TRIGGER_MOTOR_ID:
			{
				get_motor_measure(&motor_chassis[6], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + 6);
				if(start_up_i<=10)
				{
					motor_chassis[6].ecd_count = 0;
				}
				else
				{
					if (motor_chassis[6].ecd - motor_chassis[6].last_ecd > HALF_ECD_RANGE)
					{
						motor_chassis[6].ecd_count--;
					}
					else if (motor_chassis[6].ecd - motor_chassis[6].last_ecd < -HALF_ECD_RANGE)
					{
						motor_chassis[6].ecd_count++;
					}
				}
				start_up_i ++;
				
				
				motor_chassis[6].relative_angle_19laps = motor_chassis[6].ecd_count * ECD_RANGE + motor_chassis[6].ecd;
				while(motor_chassis[6].relative_angle_19laps>=(8192.0f*3592.0f/187.0f))
				{
					motor_chassis[6].relative_angle_19laps -= 8192.0f*3592.0f/187.0f;
				}
				motor_chassis[6].relative_angle_19laps = rad_format(motor_chassis[6].relative_angle_19laps*MOTOR_ECD_TO_ANGLE19);
				break;
			}
			
			case CAN_CAP_ID:
			{
				get_cap_data(&SCM_rx_message, rx_data);
				Lost_Connection_Count = 0;
				break;
			}
			default:
				break;
		}
	}
	else if (hcan == &hcan1)
	{
		if(rx_header.IDE == 0)
		{
			switch (rx_header.StdId)
			{
				case CAN_3508_FRICL_ID:
				{
					static uint8_t i = 7;
					get_motor_measure(&motor_chassis[i], rx_data);
					break;
				}
				case CAN_3508_FRICR_ID:
				{
					static uint8_t i = 8;
					get_motor_measure(&motor_chassis[i], rx_data);
					break;
				}
				case CAN_AUTOAIM_DATA:
				{
					get_autoaim_data(&autoaim_data, rx_data);
					break;
				}
				default:
					break;
			}
		}

	}

	if(rx_header.IDE == CAN_ID_STD)
	{}
	else
	{
		Motor_Can_ID=Get_Motor_ID(rx_header.ExtId);//首先获取回传电机ID信息  
		switch(Motor_Can_ID)                   //将对应ID电机信息提取至对应结构体
		{
				case 0X06:  
						if(rx_header.ExtId>>24 != 0)               //检查是否为广播模式
								Motor_Data_Handler(&mi_motor[0],rx_data,rx_header.ExtId);
						else 
								mi_motor[0].MCU_ID = rx_data[0];
						break;           
				default:
						break;		
		}
	}	
}
	
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxMsg, rx_data2);//接收数据
//	if (hcan == &hcan1)
//	{
//		if(rxMsg.StdId!=CAN_3508_FRICL_ID && rxMsg.StdId!=CAN_3508_FRICR_ID && rxMsg.StdId!=CAN_AUTOAIM_DATA && rxMsg.StdId!=CAN_3508_M1_ID && rxMsg.StdId!=CAN_3508_M2_ID &&
//			rxMsg.StdId!=CAN_3508_M3_ID && rxMsg.StdId!=CAN_3508_M4_ID && rxMsg.StdId!=CAN_TRIGGER_MOTOR_ID && rxMsg.StdId!=CAN_YAW_MOTOR_ID && rxMsg.StdId!=CAN_CAP_ID)
//		{
//			Motor_Can_ID=Get_Motor_ID(rxMsg.ExtId);//首先获取回传电机ID信息  
//			switch(Motor_Can_ID)                   //将对应ID电机信息提取至对应结构体
//			{
//					case 0X02:  
//							if(rxMsg.ExtId>>24 != 0)               //检查是否为广播模式
//									Motor_Data_Handler(&mi_motor[0],rx_data2,rxMsg.ExtId);
//							else 
//									mi_motor[0].MCU_ID = rx_data2[0];
//							break;           
//					default:
//							break;		
//			}
//		}
//	}




//	{		
//		if(rxMsg.IDE == 0x00)
//		{
//		}
//		else
//		{
//			Motor_Can_ID=Get_Motor_ID(rxMsg.ExtId);//首先获取回传电机ID信息  
//			switch(Motor_Can_ID)                   //将对应ID电机信息提取至对应结构体
//			{
//					case 0X06:  
//							if(rxMsg.ExtId>>24 != 0)               //检查是否为广播模式
//									Motor_Data_Handler(&mi_motor[0],rx_data2,rxMsg.ExtId);
//							else 
//									mi_motor[0].MCU_ID = rx_data2[0];
//							break;           
//					default:
//							break;		
//			}
//		}
//	}
//}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      rev: (0x208) 保留，电机控制电流,新英雄pitch的左边电机6020
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t pitch2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (pitch2 >> 8);
    gimbal_can_send_data[7] = pitch2;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x201,0x202)
  * @param[in]      fricl: (0x201) 
  * @param[in]      fricr: (0x202) 
  * @retval         none
  */
void CAN_cmd_fric(int16_t fricl, int16_t fricr)
{
    uint32_t send_mail_box;
    fric_tx_message.StdId = 0x200;
    fric_tx_message.IDE = CAN_ID_STD;
    fric_tx_message.RTR = CAN_RTR_DATA;
    fric_tx_message.DLC = 0x08;
    fric_can_send_data[0] = (fricl >> 8);
    fric_can_send_data[1] = fricl;
    fric_can_send_data[2] = (fricr >> 8);
    fric_can_send_data[3] = fricr;
    fric_can_send_data[4] = 0;
    fric_can_send_data[5] = 0;
    fric_can_send_data[6] = 0;
    fric_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &fric_tx_message, fric_can_send_data, &send_mail_box);
}


/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          根据通道选择电容工作模式(0-1-2-3)
  * @param[in]      cap_flag : (0-1-2-3)
  * @retval         none
  */
void CAN_cmd_supercap(int16_t flag)
{
    uint32_t send_mail_box;
	uint8_t Powersourses_Charge=0x0001;
	uint8_t Powersourses_Uncharge=0x0000;
	uint8_t Capsourses_Charge=0x0003;
	uint8_t Capsourses_Uncharge=0x0002;
		
    supercap_tx_message.StdId = 0x300;
    supercap_tx_message.IDE = CAN_ID_STD;
    supercap_tx_message.RTR = CAN_RTR_DATA;
    supercap_tx_message.DLC = 0x08;
	
	if (flag==1){ //电容没充满，有剩余功率
		supercap_can_send_data[0] = (unsigned char)(Powersourses_Charge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)(Residue_Power >> 8);
		supercap_can_send_data[3] = (unsigned char)Residue_Power;
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	else if(flag==0){	//电池供电
		supercap_can_send_data[0] = (unsigned char)(Powersourses_Uncharge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)0;
		supercap_can_send_data[3] = (unsigned char)0;
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	else if(flag==2){	//用电容供电,电池同时供电
		supercap_can_send_data[0] = (unsigned char)(Capsourses_Uncharge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)0;
		supercap_can_send_data[3] = (unsigned char)0;
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	else if(flag==3){	//用电容供电
		supercap_can_send_data[0] = (unsigned char)(Capsourses_Charge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)(Residue_Power>>8);
		supercap_can_send_data[3] = (unsigned char)(Residue_Power);
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	
	  HAL_CAN_AddTxMessage(&hcan2, &supercap_tx_message, supercap_can_send_data, &send_mail_box);

}

/**
  * @brief          自瞄can发送函数,发四元数，1000hz
  * @param[in]      四元数
  * @retval         NULL
  */
void CAN_send_imu_to_computer(fp32 x, fp32 y, fp32 z, fp32 w)
{
  uint32_t send_mail_box;
	autoaim_tx_message.StdId = CAN_AUTOAIM_IMU;
	autoaim_tx_message.IDE = CAN_ID_STD;
	autoaim_tx_message.RTR = CAN_RTR_DATA;
	autoaim_tx_message.DLC = 0x08;	
	autoaim_send_data[0] = (int16_t)(x * 1e4f) >> 8;
	autoaim_send_data[1] = (int16_t)(x * 1e4f);
	autoaim_send_data[2] = (int16_t)(y * 1e4f) >> 8;
	autoaim_send_data[3] = (int16_t)(y * 1e4f);
	autoaim_send_data[4] = (int16_t)(z * 1e4f) >> 8;
	autoaim_send_data[5] = (int16_t)(z * 1e4f);
	autoaim_send_data[6] = (int16_t)(w * 1e4f) >> 8;
	autoaim_send_data[7] = (int16_t)(w * 1e4f);
	HAL_CAN_AddTxMessage(&hcan1, &autoaim_tx_message, autoaim_send_data, &send_mail_box);
}	

/**
  * @brief          自瞄can发送函数,发枪管热量，控制模式等，50hz
  * @param[in]      枪管射速，功能模式，1为自瞄、2为打符、3为推塔
  * @retval         NULL
  */
void CAN_send_sth_to_computer(fp32 speed, uint8_t mode)
{
  uint32_t send_mail_box;
	autoaim_tx_message.StdId = CAN_AUTOAIM_STH;
	autoaim_tx_message.IDE = CAN_ID_STD;
	autoaim_tx_message.RTR = CAN_RTR_DATA;
	autoaim_tx_message.DLC = 0x08;	
	autoaim_send_data[0] = (uint16_t)(speed * 1e2f) >> 8;
	autoaim_send_data[1] = (uint16_t)(speed * 1e2f);
	autoaim_send_data[2] = mode;
	HAL_CAN_AddTxMessage(&hcan1, &autoaim_tx_message, autoaim_send_data, &send_mail_box);
}	



/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &mi_motor[0];
}

motor_measure_t *get_pitch_gimbal_motor_measure_point_for_init(void)
{
    return &mi_motor[0];
}
/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

const motor_measure_t *get_fricl_motor_measure_point(void)
{
    return &motor_chassis[7];
}

const motor_measure_t *get_fricr_motor_measure_point(void)
{
    return &motor_chassis[8];
}

/**
  * @brief          返回超级电容数据指针
  * @param[in]      null
  * @retval         电机数据指针
  */
const supercap_module_receive *get_cap_measure_point(void)
{
	return &SCM_rx_message;
}

/**
  * @brief          返回自瞄数据指针
  * @param[in]      null
  * @retval         自瞄数据指针
  */
const autoaim_data_t *get_autoaim_data_point(void)
{
    return &autoaim_data;
}

autoaim_data_t *get_autoaim_data_point_changeable(void)
{
    return &autoaim_data;
}





/**
  * @brief          写入电机参数
  * @param[in]      Motor:对应控制电机结构体
  * @param[in]      Index:写入参数对应地址
  * @param[in]      Value:写入参数值
  * @param[in]      Value_type:写入参数数据类型
  * @retval         none
  */
static void Set_Motor_Parameter(motor_measure_t *Motor,uint16_t Index,float Value,char Value_type){
	uint8_t tx_data[8];
	txMsg.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	tx_data[0]=Index;
	tx_data[1]=Index>>8;
	tx_data[2]=0x00;
	tx_data[3]=0x00;
	if(Value_type == 'f'){
		Float_to_Byte(Value);
		tx_data[4]=byte[3];
		tx_data[5]=byte[2];
		tx_data[6]=byte[1];
		tx_data[7]=byte[0];		
	}
	else if(Value_type == 's'){
		tx_data[4]=(uint8_t)Value;
		tx_data[5]=0x00;
		tx_data[6]=0x00;
		tx_data[7]=0x00;				
	}
	can_txd();	
}




/**
  * @brief          小米电机ID检查
  * @param[in]      id:  控制电机CAN_ID【出厂默认0x7F】
  * @retval         none
  */
void chack_cybergear(uint8_t ID)
{
    uint8_t tx_data[8] = {0};
    txMsg.ExtId = Communication_Type_GetID<<24|Master_CAN_ID<<8|ID;
    can_txd();
}

/**
  * @brief          使能小米电机
  * @param[in]      Motor:对应控制电机结构体   
  * @retval         none
  */
void start_cybergear(motor_measure_t *Motor)
{
    uint8_t tx_data[8] = {0}; 
    txMsg.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|Motor->CAN_ID;
    can_txd();
}

/**
  * @brief          停止电机
  * @param[in]      Motor:对应控制电机结构体   
  * @param[in]      clear_error:清除错误位（0 不清除 1清除）
  * @retval         None
  */
void stop_cybergear(motor_measure_t *Motor,uint8_t clear_error)
{
	uint8_t tx_data[8]={0};
	tx_data[0]=clear_error;//清除错误位设置
	txMsg.ExtId = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|Motor->CAN_ID;
    can_txd();
}

/**
  * @brief          设置电机模式(必须停止时调整！)
  * @param[in]      Motor:  电机结构体
  * @param[in]      Mode:   电机工作模式（1.运动模式Motion_mode 2. 位置模式Position_mode 3. 速度模式Speed_mode 4. 电流模式Current_mode）
  * @retval         none
  */
void set_mode_cybergear(motor_measure_t *Motor,uint8_t Mode)
{	
	Set_Motor_Parameter(Motor,Run_mode,Mode,'s');
}

/**
  * @brief          电流控制模式下设置电流
  * @param[in]      Motor:  电机结构体
  * @param[in]      Current:电流设置
  * @retval         none
  */
void set_current_cybergear(motor_measure_t *Motor,float Current)
{
	Set_Motor_Parameter(Motor,Iq_Ref,Current,'f');
}

/**
  * @brief          设置电机零点
  * @param[in]      Motor:  电机结构体
  * @retval         none
  */
void set_zeropos_cybergear(motor_measure_t *Motor)
{
	uint8_t tx_data[8]={0};
	txMsg.ExtId = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|Motor->CAN_ID;
	can_txd();		
}

/**
  * @brief          设置电机CANID
  * @param[in]      Motor:  电机结构体
  * @param[in]      Motor:  设置新ID
  * @retval         none
  */
void set_CANID_cybergear(motor_measure_t *Motor,uint8_t CAN_ID)
{
	uint8_t tx_data[8]={0};
	txMsg.ExtId = Communication_Type_CanID<<24|CAN_ID<<16|Master_CAN_ID<<8|Motor->CAN_ID;
    Motor->CAN_ID = CAN_ID;//将新的ID导入电机结构体
    can_txd();	
}
/**
  * @brief          小米电机初始化
  * @param[in]      Motor:  电机结构体
  * @param[in]      Can_Id: 小米电机ID(默认0x7F)
  * @param[in]      Motor_Num: 电机编号
  * @param[in]      mode: 电机工作模式（0.运动模式Motion_mode 1. 位置模式Position_mode 2. 速度模式Speed_mode 3. 电流模式Current_mode）
  * @retval         none
  */
void init_cybergear(motor_measure_t *Motor,uint8_t Can_Id, uint8_t mode)
{
    txMsg.StdId = 0;            //配置CAN发送：标准帧清零 
    txMsg.ExtId = 0;            //配置CAN发送：扩展帧清零     
    txMsg.IDE = CAN_ID_EXT;     //配置CAN发送：扩展帧
    txMsg.RTR = CAN_RTR_DATA;   //配置CAN发送：数据帧
    txMsg.DLC = 0x08;           //配置CAN发送：数据长度
    
	Motor->CAN_ID=Can_Id;       //ID设置 
	set_mode_cybergear(Motor,mode);//设置电机模式
	start_cybergear(Motor);        //使能电机
}

/**
  * @brief          小米运控模式指令
  * @param[in]      Motor:  目标电机结构体
  * @param[in]      torque: 力矩设置[-12,12] N*M
  * @param[in]      MechPosition: 位置设置[-12.5,12.5] rad
  * @param[in]      speed: 速度设置[-30,30] rpm
  * @param[in]      kp: 比例参数设置
  * @param[in]      kd: 微分参数设置
  * @retval         none
  */
void motor_controlmode(motor_measure_t *Motor,float torque, float MechPosition, float speed, float kp, float kd)
{   
    uint8_t tx_data[8];//发送数据初始化
    //装填发送数据
    tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;  
    tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);  
    tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;  
    tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);  
    tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;  
    tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);  
    tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;  
    tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16); 
    
    txMsg.ExtId = Communication_Type_MotionControl<<24|float_to_uint(torque,T_MIN,T_MAX,16)<<8|Motor->CAN_ID;//装填扩展帧数据
    can_txd();
}

