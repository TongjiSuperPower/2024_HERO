/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

//#define CHASSIS_CAN hcan1
//#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x06,
    CAN_TRIGGER_MOTOR_ID = 0x207,
		CAN_CAP_ID = 0X301,

    CAN_GIMBAL_ALL_ID = 0x1FF,

} can2_msg_id_e;

typedef enum
{
    CAN_3508_FRICL_ID = 0x201,
    CAN_3508_FRICR_ID = 0x202,
		//���鷢�ͱ�ʶ
		CAN_AUTOAIM_IMU = 0x100,				//imu����
		CAN_AUTOAIM_STH = 0x101,				//ǹ������&����ģʽ
		//����������ݱ�ʶ��
		CAN_AUTOAIM_DATA = 0x0FF,

} can1_msg_id_e;

//new_autoaim data
typedef struct
{
	uint8_t control;
	bool_t shoot;
	fp32 yaw;
	fp32 pitch;	
} autoaim_data_t;


//rm motor data
typedef struct
{
	uint32_t last_last_last_ecd;
	uint32_t last_last_ecd;
    uint32_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int32_t last_ecd;
	fp32 relative_angle_19laps;
	int16_t ecd_count;
	uint8_t CAN_ID;
	uint8_t MCU_ID;
	float Torque;
	float Temp;
	uint8_t error_code;
} motor_measure_t;

//supercap data
typedef struct
{
	uint8_t charge_enabled_state;
	uint8_t residue_power;
	uint32_t charge_power;
	int32_t chassis_power;
	int32_t cap_vol;
}supercap_module_receive;

#define SCM_rx_mes supercap_module_receive  //�ĸ���




//���Ʋ�����ֵ����������
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
#define MAX_P 720
#define MIN_P -720
//����CANID����
#define Master_CAN_ID 0x00                      //����ID
//��������궨��
#define Communication_Type_GetID 0x00           //��ȡ�豸��ID��64λMCUΨһ��ʶ��
#define Communication_Type_MotionControl 0x01 	//�������������Ϳ���ָ��
#define Communication_Type_MotorRequest 0x02	//���������������������״̬
#define Communication_Type_MotorEnable 0x03	    //���ʹ������
#define Communication_Type_MotorStop 0x04	    //���ֹͣ����
#define Communication_Type_SetPosZero 0x06	    //���õ����е��λ
#define Communication_Type_CanID 0x07	        //���ĵ�ǰ���CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//��ȡ��������
#define Communication_Type_SetSingleParameter 0x12	//�趨��������
#define Communication_Type_ErrorFeedback 0x15	    //���Ϸ���֡
//������ȡ�궨��
#define Run_mode 0x7005	
#define Iq_Ref   0x7006
#define Spd_Ref  0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK 0X01

enum CONTROL_MODE   //����ģʽ����
{
    Motion_mode = 0,//�˿�ģʽ  
    Position_mode,  //λ��ģʽ
    Speed_mode,     //�ٶ�ģʽ  
    Current_mode    //����ģʽ
};
enum ERROR_TAG      //����ش�����
{
    OK                 = 0,//�޹���
    BAT_LOW_ERR        = 1,//Ƿѹ����
    OVER_CURRENT_ERR   = 2,//����
    OVER_TEMP_ERR      = 3,//����
    MAGNETIC_ERR       = 4,//�ű������
    HALL_ERR_ERR       = 5,//HALL�������
    NO_CALIBRATION_ERR = 6//δ�궨
};

//typedef struct{           //С�׵���ṹ��
//	uint8_t CAN_ID;       //CAN ID
//    uint8_t MCU_ID;       //MCUΨһ��ʶ��[��8λ����64λ]
//	float ecd;          //�ش��Ƕ�
//	float speed_rpm;          //�ش��ٶ�
//	float Torque;         //�ش�����
//	float Temp;			  //�ش��¶�
//	
//	uint16_t set_current;
//	uint16_t set_speed;
//	uint16_t set_position;
//	
//	uint8_t error_code;
//	
//	float Angle_Bias;

//	
//}pitch_motor_measure_t;
extern motor_measure_t mi_motor[1];//Ԥ�ȶ����ĸ�С�׵��

extern void chack_cybergear(uint8_t ID);
extern void start_cybergear(motor_measure_t *Motor);
extern void stop_cybergear(motor_measure_t *Motor, uint8_t clear_error);
extern void set_mode_cybergear(motor_measure_t *Motor, uint8_t Mode);
extern void set_current_cybergear(motor_measure_t *Motor, float Current);
extern void set_zeropos_cybergear(motor_measure_t *Motor);
extern void set_CANID_cybergear(motor_measure_t *Motor, uint8_t CAN_ID);
extern void init_cybergear(motor_measure_t *Motor, uint8_t Can_Id, uint8_t mode);
extern void motor_controlmode(motor_measure_t *Motor,float torque, float MechPosition, float speed, float kp, float kd);




/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 3508/2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_fric(int16_t fricl, int16_t fricr);

/**
  * @brief          ����ͨ��ѡ����ݹ���ģʽ(0-1-2-3)
  * @param[in]      cap_flag : (0-1-2-3)
  * @retval         none
  */
extern void CAN_cmd_supercap(int16_t flag);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern motor_measure_t *get_pitch_gimbal_motor_measure_point_for_init(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_fricl_motor_measure_point(void);
extern const motor_measure_t *get_fricr_motor_measure_point(void);
extern const supercap_module_receive *get_cap_measure_point(void);
extern const autoaim_data_t *get_autoaim_data_point(void);
extern autoaim_data_t *get_autoaim_data_point_changeable(void);


extern void CAN_send_imu_to_computer(fp32 x, fp32 y, fp32 z, fp32 w);
extern void CAN_send_sth_to_computer(fp32 speed, uint8_t mode);
#endif
