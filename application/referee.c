/******************
裁判信息解读与透传数据帧封装程序
update: 2017.5.7
        视情况调用最后三个函数
        全局变量说明见RefereeInfo.h
        支持上传3个float数据
******************/
#include "bsp_usart.h"
#include "chassis_task.h"
#include "shoot.h"
#include "referee.h"
#include <cstdint>
#include "gimbal_task.h"
#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
//#include "Task.h"
u16 cmd_id;  // 数据包ID
// 裁判信息相关结构体  
ext_game_status_t                          ext_game_status;// 比赛状态数据（0x0001）
ext_game_result_t                          ext_game_result;//比赛结果数据(0x0002)
ext_game_robot_HP_t                 	   ext_game_robot_HP;//机器人血量数据（0x0003）


ext_event_data_t                           ext_event_data;//场地事件数据（0x0101）
ext_supply_projectile_action_t             ext_supply_projectile_action;//补给站动作标识数据（0x0102）

ext_referee_warning_t                      ext_referee_warning;//裁判警告数据(0x0104)
ext_dart_info_t                  		ext_dart_info;//飞镖发射相关数据(0x0105)


ext_robot_status_t                    		 ext_robot_status;//机器人性能体系数据(0x0201)
ext_power_heat_data_t                      ext_power_heat_data;//实时底盘功率和枪口热量数据（0x0202）
ext_robot_pos_t                       		 ext_robot_pos;//机器人位置数据（0x0203）
ext_buff_t                                 ext_buff;//机器人增益数据（0x0204）
ext_air_support_data_t                     ext_air_support_data;//空中支援时间数据（0x0205）
ext_hurt_data_t                            ext_hurt_data;//伤害状态数据（0x0206）
ext_shoot_data_t                           ext_shoot_data;//实时射击数据（0x0207）
ext_projectile_allowance_t                 ext_projectile_allowance;//允许发弹量(0x0208)
ext_rfid_status_t                          ext_rfid_status;//机器人 RFID 模块状态(0x0209)
ext_dart_client_cmd_t                      ext_dart_client_cmd;//飞镖选手端指令数据(0x020A)
ext_ground_robot_position_t                ext_ground_robot_position;//地面机器人位置数据(0x020B)
ext_radar_mark_data_t                      ext_radar_mark_data;//雷达标记进度数据(0x020C)
ext_sentry_info_t                          ext_sentry_info;//哨兵自主决策信息同步(0x020D)
ext_radar_info_t                      		 ext_radar_info;//雷达自主决策信息同步(0x020E)

//-------------0x0301部分开始-------------------
ext_robot_interactive_header_data_t        ext_robot_interactive_header_data;//机器人交互数据（0x0301）
robot_interactive_data_t                   robot_interactive_data;//机器人间交互数据，内容 ID:0x0200~0x02FF
ext_interaction_layer_delete_t             ext_interaction_layer_delete;//客户端删除图形，内容 ID:0x0100;
graphic_data_struct_t                      graphic_data_struct;//图形数据
ext_interaction_figure_t         					 ext_interaction_figure;//客户端绘制一个图形
ext_interaction_figure_double_t            ext_interaction_figure_double;//客户端绘制两个图形
ext_interaction_figure_five_t              ext_interaction_figure_five;//客户端绘制五个图形
ext_interaction_figure_seven_t             ext_interaction_figure_seven;//客户端绘制七个图形
ext_client_custom_character_t              ext_client_custom_character;//客户端绘制字符
//-------------0x0301部分结束-------------------


ext_custom_robot_data_t                    ext_custom_robot_data;//自定义控制器与机器人交互数据(0x0302)
ext_map_command_t                          ext_map_command; //选手端小地图交互数据(0x0303)/*发送频率：触发时发送.*/
ext_remote_control_t                       ext_remote_control;//键鼠遥控数据(0x0304)
ext_map_robot_data_t                       ext_map_robot_data; //选手端小地图接收雷达数据(0x0305)
ext_custom_client_data_t                   ext_custom_client_data; //自定义控制器与选手端交互数据(0x0306)
ext_map_data_t                             ext_map_data; //选手端小地图接收哨兵数据(0x0307)
ext_custom_info_t                          ext_custom_info; //选手端小地图接收机器人数据(0x0308)

frame_header_struct_t ext_referee_receive_header;
frame_header_struct_t ext_referee_send_header;

uint8_t USART6_dma[80];		//DMA接收数据
uint8_t Personal_Data[128];	//DMA发送数据
extern SCM_rx_mes *SCM_rx_message;//超级电容数据
const u8 sof = 0xA5;  // 帧头
u8 custom_info_counter = 0;  // 自定义数据包序号
u8 seq = 0;  // 发过来的包序号
u8 seq_real = 0;

Bytes2Float bytes2float;  // float和字节互转
Bytes2U32 bytes2u32;  // flaot和u16互转

u8 referee_message[64];  // 完整数据帧存放, 理论44就够。
int message_byte_counter = 0;  // 字节存放位置计数
u8 blood_counter = 0;  // (debug)被打计数

int	shoot_seq = 0;//shoot number

extern shoot_control_t shoot_control;//shoot state
extern bool spinning_state;



uint32_t last_time_tick_1ms_1 = 0;
uint32_t time_interval_1 = 0;
char str1[30];
char str2[30];
//int MY_CLIENT_ID = clientid_red_hero;
ext_id_t MY_CLIENT_ID = clientid_red_hero;
int MY_ROBOT_ID = robotid_red_hero;
bool UI_if_init = 0;
int UI_count = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_ob;

void init_referee_struct_data(void)
{
    memset(&ext_referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&ext_game_status, 0, sizeof(ext_game_status_t));
    memset(&ext_game_result, 0, sizeof(ext_game_result_t));
    memset(&ext_game_robot_HP, 0, sizeof(ext_game_robot_HP_t));

    memset(&ext_event_data, 0, sizeof(ext_event_data_t));
    memset(&ext_supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&ext_referee_warning, 0, sizeof(ext_referee_warning_t));
		memset(&ext_dart_info, 0, sizeof(ext_dart_info));

    memset(&ext_robot_status, 0, sizeof(ext_robot_status_t));
    memset(&ext_power_heat_data, 0, sizeof(ext_power_heat_data_t));
    memset(&ext_robot_pos, 0, sizeof(ext_robot_pos_t));
    memset(&ext_buff, 0, sizeof(ext_buff));
    memset(&ext_air_support_data, 0, sizeof(ext_air_support_data));
    memset(&ext_hurt_data, 0, sizeof(ext_hurt_data_t));
    memset(&ext_shoot_data, 0, sizeof(ext_shoot_data_t));
    memset(&ext_projectile_allowance, 0, sizeof(ext_projectile_allowance_t));
	  memset(&ext_rfid_status, 0, sizeof(ext_rfid_status));
	  memset(&ext_dart_client_cmd, 0, sizeof(ext_dart_client_cmd));
		memset(&ext_ground_robot_position, 0, sizeof(ext_ground_robot_position));
		memset(&ext_radar_mark_data, 0, sizeof(ext_radar_mark_data));
		memset(&ext_sentry_info, 0, sizeof(ext_sentry_info));
		memset(&ext_radar_info, 0, sizeof(ext_radar_info));
		

    memset(&ext_robot_interactive_header_data, 0, sizeof(ext_robot_interactive_header_data_t));
		memset(&robot_interactive_data, 0, sizeof(robot_interactive_data));
		memset(&ext_custom_robot_data, 0, sizeof(ext_custom_robot_data));
		memset(&ext_map_command, 0, sizeof(ext_map_command));
		memset(&ext_remote_control, 0, sizeof(ext_remote_control));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&ext_referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID: // 0x0001
        {
            memcpy(&ext_game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID: // 0x0002
        {
            memcpy(&ext_game_result, frame + index, sizeof(ext_game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID: // 0x0003
        {
            memcpy(&ext_game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;
        case EVENTS_DATA_CMD_ID: // 0x0101
        {
            memcpy(&ext_event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID: // 0x0102
        {
            memcpy(&ext_supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID: // 0x0104
        {
            memcpy(&ext_referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;
				case DART_INFO_CMD_ID: // 0x0105
        {
            memcpy(&ext_dart_info, frame + index, sizeof(ext_dart_info_t));
        }
        break;
        case ROBOT_STATUS_CMD_ID: // 0x0201
        {
            memcpy(&ext_robot_status, frame + index, sizeof(ext_robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID: // 0x0202
        {
            memcpy(&ext_power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID: // 0x0203
        {
            memcpy(&ext_robot_pos, frame + index, sizeof(ext_robot_pos_t));
        }
        break;
        case BUFF_CMD_ID: // 0x0204
        {
            memcpy(&ext_buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case AIR_SUPPORT_DATA_CMD_ID: // 0x0205
        {
            memcpy(&ext_air_support_data, frame + index, sizeof(ext_air_support_data_t));
        }
        break;
        case HURT_DATA_CMD_ID: // 0x0206
        {
            memcpy(&ext_hurt_data, frame + index, sizeof(ext_hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID: // 0x0207
        {
            memcpy(&ext_shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case PROJECTILE_ALLOWANCE_CMD_ID: // 0x0208
        {
            memcpy(&ext_projectile_allowance, frame + index, sizeof(ext_projectile_allowance_t));
        }
        break;
        case RFID_STATUS_CMD_ID: // 0x0209
        {
            memcpy(&ext_rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
				case DART_CLIENT_CMD_CMD_ID: // 0x020A
        {
            memcpy(&ext_dart_client_cmd, frame + index, sizeof( ext_dart_client_cmd_t));
        }
        break;
        case GROUND_ROBOT_POSITION_CMD_ID: // 0x020B
        {
            memcpy(&ext_ground_robot_position, frame + index, sizeof(ext_ground_robot_position_t));
        }
        break;
				case RADAR_MARK_DATA_CMD_ID: // 0x020C
        {
            memcpy(&ext_radar_mark_data, frame + index, sizeof(ext_radar_mark_data_t));
        }
        break;
        case SENTRY_INFO_CMD_ID: // 0x020D
        {
            memcpy(&ext_sentry_info, frame + index, sizeof(ext_sentry_info_t));
        }
        break;
				case RADAR_INFO_CMD_ID: // 0x020E
        {
            memcpy(&ext_radar_info, frame + index, sizeof(ext_radar_info_t));
        }
        break;
				case ROBOT_INTERACTIVE_DATA_CMD_ID: // 0x0301
        {
            memcpy(&ext_robot_interactive_header_data, frame + index, sizeof(ext_robot_interactive_header_data_t));
        }
				case CUSTOM_ROBOT_DATA_CMD_ID: // 0x0302
        {
            memcpy(&ext_custom_robot_data , frame + index, sizeof(ext_custom_robot_data_t));
        }
        break;
				case MAP_COMMAND_CMD_ID: // 0x0303
        {
            memcpy(&ext_map_command, frame + index, sizeof(ext_map_command_t));
        }
        break;
				case ROBOT_COMMAND_CMD_ID: // 0x0304
        {
            memcpy(&ext_remote_control, frame + index, sizeof(ext_remote_control_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = ext_power_heat_data.chassis_power;
    *buffer = ext_power_heat_data.buffer_energy;
}

uint8_t get_robot_id(void)
{
    return ext_robot_status.robot_id;
}

//FROM INFANTRY:
void UI_send_init(){

	if(	UI_if_init == 0)//the first time to draw, need init
	{
		switch(ext_robot_status.robot_id)
		{
			case robotid_red_hero:{
					MY_CLIENT_ID = clientid_red_hero;
					MY_ROBOT_ID = robotid_red_hero;	
				  break;
			}
			case robotid_red_engineer:{
					MY_CLIENT_ID = clientid_red_engineer;
					MY_ROBOT_ID = robotid_red_engineer;	
				  break;
			}
			case robotid_red_infantry_1:{
					MY_CLIENT_ID = clientid_red_infantry_1;
					MY_ROBOT_ID = robotid_red_infantry_1;	
				  break;
			}
			case robotid_red_infantry_2:{
					MY_CLIENT_ID = clientid_red_infantry_2;
					MY_ROBOT_ID = robotid_red_infantry_2;	
				  break;					
			}
			case robotid_red_infantry_3:{
					MY_CLIENT_ID = clientid_red_infantry_3;
					MY_ROBOT_ID = robotid_red_infantry_3;
				  break;					
			}
			case robotid_red_aerial:{
					MY_CLIENT_ID = clientid_red_aerial;
					MY_ROBOT_ID = robotid_red_aerial;
				  break;					
			} 
		
			case robotid_blue_hero:{
					MY_CLIENT_ID = clientid_blue_hero;
					MY_ROBOT_ID = robotid_blue_hero;	
				  break;
			}
				case robotid_blue_engineer:{
					MY_CLIENT_ID = clientid_blue_engineer;
					MY_ROBOT_ID = robotid_blue_engineer;	
				  break;
			}
			case robotid_blue_infantry_1:{
					MY_CLIENT_ID = clientid_blue_infantry_1;
					MY_ROBOT_ID = robotid_blue_infantry_1;	
				  break;
			}
			case robotid_blue_infantry_2:{
					MY_CLIENT_ID = clientid_blue_infantry_2;
					MY_ROBOT_ID = robotid_blue_infantry_2;
				  break;					
			}
			case robotid_blue_infantry_3:{
					MY_CLIENT_ID = clientid_blue_infantry_3;
					MY_ROBOT_ID = robotid_blue_infantry_3;
				  break;					
			}
			case robotid_blue_aerial:{
					MY_CLIENT_ID = clientid_blue_aerial;
					MY_ROBOT_ID = robotid_blue_aerial;
				  break;					
			} 
 
		}
		send_multi_graphic();//aim line瞄准线
    }
}


void UI_send_update(){
	if (UI_if_init == 1)
	{
		if(UI_count == 0)
		{
			if(24000<SCM_rx_message->cap_vol)
			{
				send_single_icon("x", 1800, 700, 2, 2);//绿色
			}                                      //24000以上
			else if(21000<SCM_rx_message->cap_vol)
			{
				send_single_icon("x", 1800, 700, 2, 1);//红蓝主色
			}	                                     //21000~24000
			else 
			{
				send_single_icon("x", 1800, 700, 2, 4);//紫红色
			}	     
			UI_count = 1;
		}
		else if(UI_count == 1)
		{
			UI_count = 2;
		}
		else if(UI_count == 2)
		{
			UI_count = 0;
		}
	}
}

void init_referee_info() {
	ext_robot_pos.x = 0;
	ext_robot_pos.y = 0;
	ext_robot_pos.angle = 0;
	ext_power_heat_data.shooter_17mm_1_barrel_heat = 0;
	ext_power_heat_data.shooter_17mm_2_barrel_heat = 0;
	ext_power_heat_data.shooter_42mm_barrel_heat = 0;
}

//crc8 生成多项式:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT2 = 0xff;
const unsigned char CRC8_TAB[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


//crc16 生成多项式: 未知
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC8 校验码生成
** Input: 信息数据, 信息数据长, CRC8_INIT2(0xff)
** Output: CRC8校验码
*/
unsigned char Get_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
	unsigned char ucIndex;
	while (dwLength--) {
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}


/*
** Descriptions: CRC8 校验程序
** Input: 需校验数据, 数据长度(= 信息长度 + 校验码长度)
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength) {
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2))
		return false;
	ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT2);
	return (ucExpected == pchMessage[dwLength - 1]);  // 末位校验码判断
}


/*
** Descriptions: 增添 CRC8 至信息数据尾部
** Input: 信息数据(尾部需留空1位以加入校验码), 信息数据长度
** Output: 带校验码的数据
*/
void Append_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength) {
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2))
		return;
	ucCRC = Get_CRC8_Check_Sum((unsigned char*)pchMessage, dwLength - 1, CRC8_INIT2);  // 校验码生成
	pchMessage[dwLength - 1] = ucCRC;  // 增添至尾部
}


/*
** Descriptions: CRC16 校验码生成
** Input: 信息数据, 信息数据长, CRC_INIT(0xffff)
** Output: CRC16校验码
*/
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL) {
		return 0xFFFF;
	}
	while (dwLength--) {
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}


/*
** Descriptions: CRC16 校验程序
** Input: 需校验数据, 数据长度(= 信息长度 + 校验码长度, 即包含了校验码的长度)
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength) {
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)) {
		return false;
	}
	wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: 增添 CRC16 至信息数据尾部
** Input: 信息数据(尾部需留空2位以加入校验码), 信息数据长度
** Output: 带校验码的数据
*/
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength) {
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)) {
		return;
	}
	wCRC = Get_CRC16_Check_Sum((u8*)pchMessage, dwLength - 2, CRC_INIT);
	pchMessage[dwLength - 2] = (u8)(wCRC & 0x00ff);
	pchMessage[dwLength - 1] = (u8)((wCRC >> 8) & 0x00ff);
}

// 单字节数组转u16(2字节), 高低位反序
uint16_t _bytes2u16(uint8_t* chosen_Message) {
	uint32_t temp = 0;
	// 长度为2，高低位逆序拼合
	temp = *chosen_Message;
	chosen_Message++;
	temp += (*chosen_Message << 8);
	return temp;
}

// 单字节数组转u32(4字节), 高低位未知
uint32_t _bytes4u32(uint8_t* chosen_Message) {
	bytes2u32.b[0] = chosen_Message[0];
	bytes2u32.b[1] = chosen_Message[1];
	bytes2u32.b[2] = chosen_Message[2];
	bytes2u32.b[3] = chosen_Message[3];
	return bytes2u32.u32_value;
}

// 单字节数组转float(4字节), 高低位正常
float _bytes2float(uint8_t* chosen_Message) {
	bytes2float.b[0] = chosen_Message[0];
	bytes2float.b[1] = chosen_Message[1];
	bytes2float.b[2] = chosen_Message[2];
	bytes2float.b[3] = chosen_Message[3];
	return bytes2float.f;
}
//0~7bit是从左往右还是从右往左？？如果是从右往左，这里是不是有问题？

// float转4长度单字节数组
void float2bytes(float chosen_value, u8* res_message) {
	int i;
	bytes2float.f = chosen_value;
	for (i = 0; i < 4; i++)
		res_message[i] = bytes2float.b[i];
}
// 比赛状态数据（0x0001）, 发送频率为10Hz。
void ext_game_status_interpret(uint8_t* ext_game_status_Message)
{
	
	uint8_t* a;
	memcpy(a, ext_game_status_Message, 1);
	ext_game_status.game_type = (*a) >> 4;//取高四位
	ext_game_status.game_progress = (*a) & 0x000f;//取低四位

	memcpy((uint8_t*)&ext_game_status.stage_remain_time, ext_game_status_Message + 1, 2);
	memcpy((uint8_t*)&ext_game_status.SyncTimeStamp, ext_game_status_Message + 3, 8);
}

//比赛结果数据(0x0002)
void  ext_game_result_interpret(uint8_t* ext_game_result_t_Message)
{
	ext_game_result.winner = *ext_game_result_t_Message;

}

//机器人血量数据（0x0003）
void ext_game_robot_HP_interpret(uint8_t* ext_game_robot_HP_t_Message)
{
	memcpy((uint8_t*)&ext_game_robot_HP, ext_game_robot_HP_t_Message, 2);

}



//场地事件数据（0x0101）
void ext_event_data_interpret(uint8_t* ext_event_data_t_Message)
{
	memcpy((uint8_t*)&ext_event_data.event_data, ext_event_data_t_Message, 4);
}

//补给站动作标识数据（0x0102）
void ext_supply_projectile_action_interpret(uint8_t* ext_supply_projectile_action_Message)
{
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_id, ext_supply_projectile_action_Message, 1);
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_robot_id, ext_supply_projectile_action_Message + 1, 1);
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_step, ext_supply_projectile_action_Message + 2, 1);
	memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_num, ext_supply_projectile_action_Message + 3, 1);
}

//裁判警告数据(0x0104)
void ext_referee_warning_interpret(uint8_t* ext_referee_warning_t_Message)
{
	memcpy((uint8_t*)&ext_referee_warning.level, ext_referee_warning_t_Message, 1);
	memcpy((uint8_t*)&ext_referee_warning.offending_robot_id, ext_referee_warning_t_Message + 1, 1);
	memcpy((uint8_t*)&ext_referee_warning.count, ext_referee_warning_t_Message + 2, 1);

}

//飞镖发射相关数据(0x0105)
void ext_dart_info_interpret(uint8_t* ext_dart_info_t_Message)
{
	memcpy((uint8_t*)&ext_dart_info.dart_remaining_time, ext_dart_info_t_Message, 1);
	memcpy((uint8_t*)&ext_dart_info.dart_info, ext_dart_info_t_Message+1, 2);
}

//机器人性能体系数据(0x0201)
void ext_robot_status_interpret(uint8_t* ext_robot_status_Message)
{
	memcpy((uint8_t*)&ext_robot_status.robot_id, ext_robot_status_Message, 1);
	memcpy((uint8_t*)&ext_robot_status.robot_level, ext_robot_status_Message + 1, 1);
	memcpy((uint8_t*)&ext_robot_status.current_HP, ext_robot_status_Message + 2, 2);
	memcpy((uint8_t*)&ext_robot_status.maximum_HP, ext_robot_status_Message + 4, 2);
	memcpy((uint8_t*)&ext_robot_status.shooter_barrel_cooling_value, ext_robot_status_Message + 6, 2);
	memcpy((uint8_t*)&ext_robot_status.shooter_barrel_heat_limit, ext_robot_status_Message + 8, 2);


	memcpy((uint8_t*)&ext_robot_status.chassis_power_limit, ext_robot_status_Message + 10, 2);
	uint8_t a;
	memcpy(&a, ext_robot_status_Message + 12, 1);
	ext_robot_status.power_management_gimbal_output = a >> 7;
	ext_robot_status.power_management_chassis_output = (a >> 6) & 0x0001;
	ext_robot_status.power_management_shooter_output = (a >> 5) & 0x0001;

}

//实时底盘功率和枪口热量数据（0x0202）
void ext_power_heat_data_interpret(uint8_t* ext_power_heat_data_Message)
{

	memcpy((uint8_t*)&ext_power_heat_data.chassis_voltage, ext_power_heat_data_Message, 2);
	memcpy((uint8_t*)&ext_power_heat_data.chassis_current, ext_power_heat_data_Message + 2, 2);
	memcpy((float*)&ext_power_heat_data.chassis_power, ext_power_heat_data_Message + 4, 4);
	memcpy((uint8_t*)&ext_power_heat_data.buffer_energy, (ext_power_heat_data_Message + 8), 2);
	memcpy((uint8_t*)&ext_power_heat_data.shooter_17mm_1_barrel_heat, ext_power_heat_data_Message + 10, 2);
	memcpy((uint8_t*)&ext_power_heat_data.shooter_17mm_2_barrel_heat, ext_power_heat_data_Message + 12, 2);
	memcpy((uint8_t*)&ext_power_heat_data.shooter_42mm_barrel_heat, ext_power_heat_data_Message + 14, 2);
}


//机器人位置数据s（0x0203）
void ext_robot_pos_interpret(uint8_t* ext_robot_pos_Message)
{
	memcpy((uint8_t*)&ext_robot_pos.x, ext_robot_pos_Message, 4);
	memcpy((uint8_t*)&ext_robot_pos.y, ext_robot_pos_Message + 4, 4);
	memcpy((uint8_t*)&ext_robot_pos.angle, ext_robot_pos_Message + 12, 4);
}

//机器人增益数据（0x0204）
void ext_buff_interpret(uint8_t* ext_buff_Message)
{
	memcpy((uint8_t*)&ext_buff.recovery_buff, ext_buff_Message, 1);
	memcpy((uint8_t*)&ext_buff.cooling_buff, ext_buff_Message + 1, 1);
	memcpy((uint8_t*)&ext_buff.defence_buff, ext_buff_Message + 2, 1);
	memcpy((uint8_t*)&ext_buff.vulnerability_buff, ext_buff_Message + 3, 1);
	memcpy((uint8_t*)&ext_buff.attack_buff, ext_buff_Message + 4, 2);

}

//空中支援时间数据（0x0205）
void air_support_data_interpret(uint8_t* air_support_data_Message)
{
  memcpy((uint8_t*)&ext_air_support_data.airforce_status, air_support_data_Message, 1);
	memcpy((uint8_t*)&ext_air_support_data.time_remain, air_support_data_Message + 1, 1);
}

//伤害状态数据（0x0206）
void ext_hurt_data_interpret(uint8_t* ext_hurt_data_Message)
{
	uint8_t a;
	memcpy((uint8_t*)&a, ext_hurt_data_Message, 1);

	ext_hurt_data.armor_id = a >> 4;
	ext_hurt_data.HP_deduction_reason = a & 0x000f;
}

//实时射击数据（0x0207）
void ext_shoot_data_interpret(uint8_t* ext_shoot_data_Message)
{

	memcpy((uint8_t*)&ext_shoot_data.bullet_type, ext_shoot_data_Message, 1);
	memcpy((uint8_t*)&ext_shoot_data.shooter_number, ext_shoot_data_Message + 1, 1);
	memcpy((uint8_t*)&ext_shoot_data.launching_frequency, ext_shoot_data_Message + 2, 1);
	memcpy((uint8_t*)&ext_shoot_data.initial_speed, ext_shoot_data_Message + 3, 4);  //子弹射速

	shoot_seq++;
}

//允许发弹量(0x0208)
void ext_projectile_allowance_interpret(uint8_t* ext_projectile_allowance_Message)
{
	memcpy((uint8_t*)&ext_projectile_allowance.projectile_allowance_17mm, ext_projectile_allowance_Message, 2);
	memcpy((uint8_t*)&ext_projectile_allowance.projectile_allowance_42mm, ext_projectile_allowance_Message + 2, 2);
	memcpy((uint8_t*)&ext_projectile_allowance.remaining_gold_coin, ext_projectile_allowance_Message + 4, 2);
}

//机器人 RFID 模块状态(0x0209)
void ext_rfid_status_interpret(uint8_t* ext_rfid_status_t_Message)
{
	memcpy((uint8_t*)&ext_rfid_status.rfid_status, ext_rfid_status_t_Message, 4);
}

//飞镖选手端指令数据(0x020A)
void ext_dart_client_cmd_interpret(uint8_t* ext_dart_client_cmd_Message)
{
	memcpy((uint8_t*)&ext_dart_client_cmd.dart_launch_opening_status, ext_dart_client_cmd_Message, 1);
	memcpy((uint8_t*)&ext_dart_client_cmd.target_change_time, ext_dart_client_cmd_Message + 2, 2);
	memcpy((uint8_t*)&ext_dart_client_cmd.operate_launch_cmd_time, ext_dart_client_cmd_Message + 4, 2);
}

//地面机器人位置数据(0x020B)
void ext_ground_robot_position_interpret(uint8_t* ext_ground_robot_position_Message)
{
	memcpy((uint8_t*)&ext_ground_robot_position.hero_x, ext_ground_robot_position_Message, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.hero_y, ext_ground_robot_position_Message + 4, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.engineer_x, ext_ground_robot_position_Message + 8, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.engineer_y, ext_ground_robot_position_Message + 12, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.standard_3_x, ext_ground_robot_position_Message + 16, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.standard_3_y, ext_ground_robot_position_Message + 20, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.standard_4_x, ext_ground_robot_position_Message + 24, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.standard_4_y, ext_ground_robot_position_Message + 28, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.standard_5_x, ext_ground_robot_position_Message + 32, 4);
	memcpy((uint8_t*)&ext_ground_robot_position.standard_5_y, ext_ground_robot_position_Message + 36, 4);
}

//雷达标记进度数据(0x020C)
void ext_radar_mark_data_interpret(uint8_t* ext_radar_mark_data_Message)
{
	memcpy((uint8_t*)&ext_radar_mark_data.mark_hero_progress, ext_radar_mark_data_Message, 1);
	memcpy((uint8_t*)&ext_radar_mark_data.mark_engineer_progress, ext_radar_mark_data_Message + 1, 1);
	memcpy((uint8_t*)&ext_radar_mark_data.mark_standard_3_progress, ext_radar_mark_data_Message + 2, 1);
	memcpy((uint8_t*)&ext_radar_mark_data.mark_standard_4_progress, ext_radar_mark_data_Message + 3, 1);
	memcpy((uint8_t*)&ext_radar_mark_data.mark_standard_5_progress, ext_radar_mark_data_Message + 4, 1);
	memcpy((uint8_t*)&ext_radar_mark_data.mark_sentry_progress, ext_radar_mark_data_Message + 5, 1);

}

//哨兵自主决策信息同步(0x020D)
void ext_sentry_info_interpret(uint8_t* ext_sentry_info_Message)
{
	memcpy((uint8_t*)&ext_sentry_info.sentry_info, ext_sentry_info_Message, 4);
}

//雷达自主决策信息同步(0x020E)
void ext_radar_info_interpret(uint8_t* ext_radar_info_Message)
{
	memcpy((uint8_t*)&ext_radar_info.radar_info, ext_radar_info_Message, 1);
}

////交互数据接收信息（0x0301）
//先实现随着内容ID（data_cmd_id）变化的各内容数据段的读取函数，最后利用switch
//统一在ext_student_interactive_header_data_interpret函数中读取。

//内容ID：0x0200~0x02FF:己方机器人间通信
void robot_interactive_data_interpret(uint8_t* robot_interactive_data_Message)
{
	memcpy((uint8_t*)robot_interactive_data.data, robot_interactive_data_Message, sizeof(robot_interactive_data.data));
}

//内容ID：0x0100:客户端删除图形
void ext_interaction_layer_delete_interpret(uint8_t* ext_interaction_layer_delete_Message)
{
	memcpy((uint8_t*)&ext_interaction_layer_delete.delete_type, ext_interaction_layer_delete_Message, 1);
	memcpy((uint8_t*)&ext_interaction_layer_delete.layer, ext_interaction_layer_delete_Message+1, 1);
}

//内容ID：0x0101:客户端绘制一个图形
void ext_interaction_figure_interpret(uint8_t* ext_interaction_figure_Message)
{
	memcpy((uint8_t*)&ext_interaction_figure.graphic_data_struct, ext_interaction_figure_Message, 15 * 1);
}
//内容ID：0x0102:客户端绘制二个图形
void ext_interaction_figure_double_interpret(uint8_t* ext_interaction_figure_double_Message)
{
	memcpy((uint8_t*)ext_interaction_figure_double.graphic_data_struct, ext_interaction_figure_double_Message, 15 * 2);
}
//内容ID：0x0103:客户端绘制五个图形
void ext_interaction_figure_five_interpret(uint8_t* ext_interaction_figure_five_Message)
{
	memcpy((uint8_t*)ext_interaction_figure_five.graphic_data_struct, ext_interaction_figure_five_Message, 15 * 5);
}
//内容ID：0x0104:客户端绘制七个图形
void ext_interaction_figure_seven_interpret(uint8_t* ext_interaction_figure_seven_Message)
{
	memcpy((uint8_t*)ext_interaction_figure_seven.graphic_data_struct, ext_interaction_figure_seven_Message, 15 * 7);
}
//内容ID：0x0110:客户端绘制字符图形
void ext_client_custom_character_interpret(uint8_t* ext_client_custom_character_Message)
{
	memcpy((uint8_t*)&ext_client_custom_character.graphic_data_struct, ext_client_custom_character_Message, 15);
	memcpy((uint8_t*)ext_client_custom_character.data, ext_client_custom_character_Message + 15, 30);
}


void ext_robot_interactive_header_data_interpret(uint8_t* ext_robot_interactive_header_data_Message)
{
	memcpy((uint8_t*)&ext_robot_interactive_header_data.data_cmd_id, ext_robot_interactive_header_data_Message, 2);//数据段的内容ID
	memcpy((uint8_t*)&ext_robot_interactive_header_data.sender_ID, ext_robot_interactive_header_data_Message + 2, 2);
	memcpy((uint8_t*)&ext_robot_interactive_header_data.receiver_ID, ext_robot_interactive_header_data_Message + 4, 2);
	uint8_t* content = ext_robot_interactive_header_data_Message + 6;
	switch (ext_robot_interactive_header_data.data_cmd_id)
	{
		//内容ID：0x0100:客户端删除图形
		case 0x0100:ext_interaction_layer_delete_interpret(content); break;
		//内容ID：0x0101:客户端绘制一个图形
		case 0x0101:ext_interaction_figure_interpret(content); break;
		//内容ID：0x0102:客户端绘制二个图形
		case 0x0102:ext_interaction_figure_double_interpret(content); break;
		//内容ID：0x0103:客户端绘制五个图形
		case 0x0103:ext_interaction_figure_five_interpret(content); break;
		//内容ID：0x0104:客户端绘制七个图形
		case 0x0104:ext_interaction_figure_seven_interpret(content); break;
		//内容ID：0x0110:客户端绘制字符图形
		case 0x0110:ext_client_custom_character_interpret(content); break;
		default:  break;
	};
	//内容ID：0x0200~0x02FF:己方机器人间通信
	if (ext_robot_interactive_header_data.data_cmd_id > 0x0199 && ext_robot_interactive_header_data.data_cmd_id < 0x0300)
	{
		robot_interactive_data_interpret(content);
	}
}



//选手端小地图交互数据(0x0303)
void ext_map_command_interpret(uint8_t * ext_map_command_Message)
{
	memcpy((float*)&ext_map_command.target_position_x, ext_map_command_Message, 4);
	memcpy((float*)&ext_map_command.target_position_y, ext_map_command_Message + 4, 4);
	memcpy((uint8_t*)&ext_map_command.cmd_keyboard, ext_map_command_Message + 12, 1);
	memcpy((uint8_t*)&ext_map_command.target_robot_ID, ext_map_command_Message + 13, 1);
	memcpy((uint8_t*)&ext_map_command.cmd_source, ext_map_command_Message + 14, 2);
}



////图传链路信息标识（0x0304）
void ext_remote_control_interpret(uint8_t * ext_remote_control_Message)
{
	memcpy((uint8_t*)&ext_remote_control.mouse_x, ext_remote_control_Message, 2);
	memcpy((uint8_t*)&ext_remote_control.mouse_y, ext_remote_control_Message + 2, 2);
	memcpy((uint8_t*)&ext_remote_control.mouse_z, ext_remote_control_Message + 4, 2);
	memcpy((uint8_t*)&ext_remote_control.left_button_down, ext_remote_control_Message + 6, 1);
	memcpy((uint8_t*)&ext_remote_control.right_button_down, ext_remote_control_Message + 7, 1);
	memcpy((uint8_t*)&ext_remote_control.keyboard_value, ext_remote_control_Message + 8, 2);
}

////小地图接收信息标识(0x0305)
//void ext_client_map_command_interpret(uint8_t* ext_client_map_command_Message)
//{
//	memcpy((uint8_t*)&ext_client_map_command.target_robot_ID, ext_client_map_command_Message, 2);
//	memcpy((float*)&ext_client_map_command.target_position_x, ext_client_map_command_Message+2, 4);
//	memcpy((float*)&ext_client_map_command.target_position_y, ext_client_map_command_Message+6, 4);
//}


// 完整校验数据帧, CRC8和CRC16
u8 Verify_frame(uint8_t* frame) {
	int frame_length;
	if (frame[0] != sof) return false;
	frame_length = _bytes2u16(&frame[1]) + 5 + 2 + 2;
	if (Verify_CRC8_Check_Sum(frame, 5) && Verify_CRC16_Check_Sum(frame, frame_length)) {

		return true;
	}
	else {
		return false;
	}
}

void update_from_dma(void) {
	
	u8 USART6_dma_x2[2 * USART6_dma_rx_len];
	memcpy(USART6_dma_x2, USART6_dma, USART6_dma_rx_len);
	memcpy(USART6_dma_x2 + USART6_dma_rx_len, USART6_dma, USART6_dma_rx_len);

	frame_interpret(USART6_dma_x2, 2 * USART6_dma_rx_len);
	return;
}


bool vrerify_frame(uint8_t* frame) {
	/* frame length = header(5) + cmd_id(2) + data(n) + crc16(2) */
	uint16_t frame_length = 5 + 2 + _bytes2u16(&frame[1]) + 2;
	return (Verify_CRC8_Check_Sum(frame, sizeof(ext_frame_header_t))
		&& Verify_CRC16_Check_Sum(frame, frame_length));
}
bool frame_interpret(uint8_t* _frame, uint16_t size) {
	for (uint16_t i = 0; i < size; i++) {
		if (_frame[i] == REFEREE_FRAME_HEADER_SOF && vrerify_frame(&_frame[i])) {

			uint8_t* frame = &_frame[i];
			uint16_t length = (uint16_t)(frame[1] | (frame[2] << 8));
			i += length + 7;

			ext_cmd_id_t cmd_id = (ext_cmd_id_t)(frame[5] | (frame[6] << 8));


			switch (cmd_id) {
			case 0x0001:ext_game_status_interpret(&frame[7]); break;
			case 0x0002:ext_game_result_interpret(&frame[7]); break;
			case 0x0003:ext_game_robot_HP_interpret(&frame[7]); break;
			case 0x0101:ext_event_data_interpret(&frame[7]); break;
			case 0x0102:ext_supply_projectile_action_interpret(&frame[7]); break;
			case 0x0201:ext_robot_status_interpret(&frame[7]); break;
			case 0x0202:ext_power_heat_data_interpret(&frame[7]); break;
			case 0x0203:ext_robot_pos_interpret(&frame[7]); break;
			case 0x0204:ext_buff_interpret(&frame[7]); break;
			case 0x0205:air_support_data_interpret(&frame[7]); break;
			case 0x0206:ext_hurt_data_interpret(&frame[7]); break;
			case 0x0207:ext_shoot_data_interpret(&frame[7]); break;
			case 0x0301:ext_robot_interactive_header_data_interpret(&frame[7]); break;
			
			default: break;
			}

		}

	}
	return true;
}

void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw) 
{
	static ext_robot_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6+15;
	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0101;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.graphic_data = *graphic_draw;
	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));//帧头CRC校验

	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));

}

void referee_send_client_character(ext_id_t target_id, ext_client_custom_character_t* character_data) {


	static ext_robot_character_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6+15+30;
	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));

	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0110;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.character_data= *character_data;
	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));

}

void referee_send_two_graphic(ext_id_t target_id, ext_interaction_figure_double_t* graphic_draw) {

	static ext_robot_two_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 2 * 15;
	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0102;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去
}

void referee_send_five_graphic(ext_id_t target_id, ext_interaction_figure_five_t* graphic_draw) {

	static ext_robot_five_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 5 * 15;
	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0103;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去

}

void referee_send_multi_graphic(ext_id_t target_id, ext_interaction_figure_seven_t* graphic_draw) {

	static ext_robot_seven_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 7 * 15;
	Append_CRC8_Check_Sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0104;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	Append_CRC16_Check_Sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (u8*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去

}

bool first_pitch_draw = true;

void send_string_test()
{
	ext_client_custom_character_t character_draw;
	int32_t pitch_angle = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd - 1730;
	int32_t one = pitch_angle%10;
	int32_t ten = ((pitch_angle - one)/10)%10;
	int32_t hun = ((pitch_angle - ten*10 - one)/100)%10;
	char stringname[3] = "001";
	character_draw.graphic_data_struct.figure_name[0] =stringname[0];
	character_draw.graphic_data_struct.figure_name[1] =stringname[1];
	character_draw.graphic_data_struct.figure_name[2] =stringname[2];
	if(first_pitch_draw){
	character_draw.graphic_data_struct.operate_type = 1;
  first_pitch_draw = false;
	}
	else{
	character_draw.graphic_data_struct.operate_type = 2;
	}
	character_draw.graphic_data_struct.graphic_type =7;
	character_draw.graphic_data_struct.layer =0;
	character_draw.graphic_data_struct.color =1;
	
	character_draw.graphic_data_struct.details_a =30;
	character_draw.graphic_data_struct.details_b = 30;
	character_draw.graphic_data_struct.width =2;
	character_draw.graphic_data_struct.start_x =10;
	character_draw.graphic_data_struct.start_y =400;
	character_draw.graphic_data_struct.details_c =0;
	character_draw.graphic_data_struct.details_d = 0;
	character_draw.graphic_data_struct.details_e = 0;
	
	character_draw.data[0] = 'a';
	character_draw.data[1] = 'b';
	character_draw.data[2] = 'c';
	character_draw.data[3] = 'd';
	character_draw.data[4] = 'e';
	character_draw.data[5] = 'f';
	character_draw.data[6] = 'g';
	character_draw.data[7] = 'h';
	character_draw.data[8] = 'i';
	character_draw.data[9] = 'j';
	character_draw.data[10] = 'k';
	character_draw.data[11] = 'l';
	character_draw.data[12] = 'm';
	character_draw.data[13] = 'n';
	character_draw.data[14] = ' ';
	character_draw.data[15] = ' ';
	character_draw.data[16] = ' ';
	character_draw.data[17] = ' ';
	character_draw.data[18] = ' ';
	character_draw.data[19] = ' ';
	character_draw.data[20] = ' ';
	character_draw.data[21] = 'p';
	character_draw.data[22] = 'i';
	character_draw.data[23] = 't';
	character_draw.data[24] = 'c';
	character_draw.data[25] = 'h';
	character_draw.data[26] = ':';
	character_draw.data[27] = hun+'0';
	character_draw.data[28] = ten+'0';
	character_draw.data[29] = one+'0';
	
	referee_send_client_character(MY_CLIENT_ID, &character_draw);
}


void send_float_test()
{
	graphic_data_struct_t graphic_draw;
	int32_t fudian = 23211;
	
	char pitchname[3] = "806";
	graphic_draw.figure_name[0] =pitchname[0];
	graphic_draw.figure_name[1] =pitchname[1];
	graphic_draw.figure_name[2] =pitchname[2];
	
	graphic_draw.graphic_type = 5;
	graphic_draw.color = 1;
	graphic_draw.layer = 0;

	graphic_draw.details_a = 80;
	graphic_draw.details_a = 3;
	graphic_draw.width = 1;
	graphic_draw.start_x = 90;
	graphic_draw.start_y = 300;
	graphic_draw.details_c = fudian;
	
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

bool changes = true;

void send_change_test(int lenth0)
{
	graphic_data_struct_t graphic_draw;
	
	if(changes){
		graphic_draw.operate_type = 1;
		changes = false;
	}
	else{
		graphic_draw.operate_type = 2;
	}
	char pitchname[3] = "606";
	graphic_draw.figure_name[0] =pitchname[0];
	graphic_draw.figure_name[1] =pitchname[1];
	graphic_draw.figure_name[2] =pitchname[2];
	graphic_draw.graphic_type = 0;
	graphic_draw.layer = 0;
	graphic_draw.color = 2;
	//graphic_draw.start_angle=0;
	//graphic_draw.end_angle = 0;
	graphic_draw.width = 3; //线条宽度
	graphic_draw.start_x = 100;
	graphic_draw.start_y = 200;
	//graphic_draw.radius = 0;
	graphic_draw.details_d = 100+lenth0;
	graphic_draw.details_e = 200;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}



void send_bool_state_graphic(char graphname[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	graphic_draw.figure_name[0] = graphname[0];
	graphic_draw.figure_name[1] = graphname[1];
	graphic_draw.figure_name[2] = graphname[2];
	
	graphic_draw.operate_type = type; // 操作类型
	
	graphic_draw.color = color;
	graphic_draw.graphic_type = 2; // 正圆
	
	graphic_draw.layer = 0;
	graphic_draw.width = 15; //线条宽度
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.details_c = 15;  // 半径
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_snipeMode(int type)
{
	graphic_data_struct_t graphic_draw;
	char snipename[3] = "508";
	graphic_draw.figure_name[0] = snipename[0];
	graphic_draw.figure_name[1] = snipename[1];
	graphic_draw.figure_name[2] = snipename[2];
	
	graphic_draw.operate_type = type;
	
	graphic_draw.color = 4;
	graphic_draw.graphic_type = 1;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 15; //线条宽度
	
	graphic_draw.start_x = 50;
	graphic_draw.start_y = 600;
	graphic_draw.details_d = 70;
	graphic_draw.details_e = 620;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);

}

void send_frame1_graphic(char graphname[3], int x, int y)
{
	
	graphic_data_struct_t graphic_draw;
	
	switch(ext_robot_status.robot_id){
				case robotid_red_hero:{
						MY_CLIENT_ID = clientid_red_hero;
						MY_ROBOT_ID = robotid_red_hero;	
					  break;
				}
				case robotid_red_infantry_1:{
						MY_CLIENT_ID = clientid_red_infantry_1;
						MY_ROBOT_ID = robotid_red_infantry_1;	
					  break;
				}
				case robotid_red_infantry_2:{
						MY_CLIENT_ID = clientid_red_infantry_2;
						MY_ROBOT_ID = robotid_red_infantry_2;	
					  break;					
				}
				case robotid_red_infantry_3:{
						MY_CLIENT_ID = clientid_red_infantry_3;
						MY_ROBOT_ID = robotid_red_infantry_3;
					  break;					
				}
			
				case robotid_blue_hero:{
						MY_CLIENT_ID = clientid_blue_hero;
						MY_ROBOT_ID = robotid_blue_hero;	
					  break;
				}
				case robotid_blue_infantry_1:{
						MY_CLIENT_ID = clientid_blue_infantry_1;
						MY_ROBOT_ID = robotid_blue_infantry_1;	
					  break;
				}
				case robotid_blue_infantry_2:{
						MY_CLIENT_ID = clientid_blue_infantry_2;
						MY_ROBOT_ID = robotid_blue_infantry_2;
					  break;					
				}
				case robotid_blue_infantry_3:{
						MY_CLIENT_ID = clientid_blue_infantry_3;
						MY_ROBOT_ID = robotid_blue_infantry_3;
					  break;					
				}
			}
	
	graphic_draw.figure_name[0] = graphname[0];
	graphic_draw.figure_name[1] = graphname[1];
	graphic_draw.figure_name[2] = graphname[2];

	graphic_draw.color = 8;
	graphic_draw.graphic_type = 2;
	graphic_draw.operate_type = 1;

	graphic_draw.layer = 0;
	graphic_draw.width = 3; //线条宽度
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.details_c = 33;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_frame2_graphic(char graphname[3], int startx, int starty, int endx, int endy)
{
	
	graphic_data_struct_t graphic_draw;
	
	
	graphic_draw.figure_name[0] = graphname[0];
	graphic_draw.figure_name[1] = graphname[1];
	graphic_draw.figure_name[2] = graphname[2];

	graphic_draw.color = 8;
	graphic_draw.graphic_type = 1;
	graphic_draw.operate_type = 1;

	graphic_draw.layer = 0;
	graphic_draw.width = 3; //线条宽度
	
	graphic_draw.start_x = startx;
	graphic_draw.start_y = starty;
	graphic_draw.details_d = endx;
	graphic_draw.details_e = endy;
	
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_double_text()
{
	ext_interaction_figure_double_t graphic_draw;
	char name1[3] = "221";
	graphic_draw.graphic_data_struct[0].figure_name[0] = name1[0];
	graphic_draw.graphic_data_struct[0].figure_name[1] = name1[1];
	graphic_draw.graphic_data_struct[0].figure_name[2] = name1[2];
	graphic_draw.graphic_data_struct[0].operate_type = 1;
	graphic_draw.graphic_data_struct[0].graphic_type = 0;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 2;
	graphic_draw.graphic_data_struct[0].width =2;
	graphic_draw.graphic_data_struct[0].start_x =556;
	graphic_draw.graphic_data_struct[0].start_y =0;
	graphic_draw.graphic_data_struct[0].details_d =706;
	graphic_draw.graphic_data_struct[0].details_e =240;
	
	char name2[3] = "223";
	graphic_draw.graphic_data_struct[1].figure_name[0] = name2[0];
	graphic_draw.graphic_data_struct[1].figure_name[1] = name2[1];
	graphic_draw.graphic_data_struct[1].figure_name[2] = name2[2];
	graphic_draw.graphic_data_struct[1].operate_type = 1;
	graphic_draw.graphic_data_struct[1].graphic_type = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 2;
	graphic_draw.graphic_data_struct[1].width =2;
	graphic_draw.graphic_data_struct[1].start_x =1364;
	graphic_draw.graphic_data_struct[1].start_y =0;
	graphic_draw.graphic_data_struct[1].details_d =1214;
	graphic_draw.graphic_data_struct[1].details_e =240;
	
	referee_send_two_graphic(MY_CLIENT_ID, &graphic_draw);
	
}

void send_text_graphic(char GraphName[3])
{
	
	graphic_data_struct_t graphic_draw;
	graphic_draw.figure_name[0] =GraphName[0];
	graphic_draw.figure_name[1] =GraphName[1];
	graphic_draw.figure_name[2] =GraphName[2];
	
	graphic_draw.operate_type = 1;
	
	graphic_draw.graphic_type = 0;
	
	graphic_draw.layer = 0;
	graphic_draw.color = 2;
	//graphic_draw.start_angle=0;
	//graphic_draw.end_angle = 0;
	graphic_draw.width = 2; //线条宽度
	graphic_draw.start_x = 556;
	graphic_draw.start_y = 0;
	//graphic_draw.radius = 0;
	graphic_draw.details_d = 706;
	graphic_draw.details_e = 240;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_five_graphic()
{
	ext_interaction_figure_five_t graphic_draw;
	
	graphic_draw.graphic_data_struct[0].operate_type = 1;
	graphic_draw.graphic_data_struct[1].operate_type = 1;
	graphic_draw.graphic_data_struct[2].operate_type = 1;
	graphic_draw.graphic_data_struct[3].operate_type = 1;
	graphic_draw.graphic_data_struct[4].operate_type = 1;
	
	char five1[3] = "501";
	graphic_draw.graphic_data_struct[0].figure_name[0] = five1[0];
	graphic_draw.graphic_data_struct[0].figure_name[1] = five1[1];
	graphic_draw.graphic_data_struct[0].figure_name[2] = five1[2];
	graphic_draw.graphic_data_struct[0].graphic_type = 1;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 1;
	graphic_draw.graphic_data_struct[0].width = 1;
	graphic_draw.graphic_data_struct[0].start_x = 50;
	graphic_draw.graphic_data_struct[0].start_y = 400;
	graphic_draw.graphic_data_struct[0].details_d = 80;
	graphic_draw.graphic_data_struct[0].details_e = 440;
	
	char five2[3] = "502";
	graphic_draw.graphic_data_struct[1].figure_name[0] = five2[0];
	graphic_draw.graphic_data_struct[1].figure_name[1] = five2[1];
	graphic_draw.graphic_data_struct[1].figure_name[2] = five2[2];
	graphic_draw.graphic_data_struct[1].graphic_type = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 1;
	graphic_draw.graphic_data_struct[1].width = 1;
	graphic_draw.graphic_data_struct[1].start_x = 50;
	graphic_draw.graphic_data_struct[1].start_y = 400;
	graphic_draw.graphic_data_struct[1].details_d = 50;
	graphic_draw.graphic_data_struct[1].details_e = 360;
	
	char five3[3] = "503";
	graphic_draw.graphic_data_struct[1].figure_name[0] = five3[0];
	graphic_draw.graphic_data_struct[1].figure_name[1] = five3[1];
	graphic_draw.graphic_data_struct[1].figure_name[2] = five3[2];
	graphic_draw.graphic_data_struct[1].graphic_type = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 1;
	graphic_draw.graphic_data_struct[1].width = 1;
	graphic_draw.graphic_data_struct[1].start_x = 50;
	graphic_draw.graphic_data_struct[1].start_y = 340;
	graphic_draw.graphic_data_struct[1].details_d = 65;
	graphic_draw.graphic_data_struct[1].details_e = 300;
	
	char five4[3] = "504";
	graphic_draw.graphic_data_struct[1].figure_name[0] = five4[0];
	graphic_draw.graphic_data_struct[1].figure_name[1] = five4[1];
	graphic_draw.graphic_data_struct[1].figure_name[2] = five4[2];
	graphic_draw.graphic_data_struct[1].graphic_type = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 1;
	graphic_draw.graphic_data_struct[1].width = 1;
	graphic_draw.graphic_data_struct[1].start_x = 80;
	graphic_draw.graphic_data_struct[1].start_y = 340;
	graphic_draw.graphic_data_struct[1].details_d = 65;
	graphic_draw.graphic_data_struct[1].details_e = 300;
	
	char five5[3] = "505";
	graphic_draw.graphic_data_struct[1].figure_name[0] = five5[0];
	graphic_draw.graphic_data_struct[1].figure_name[1] = five5[1];
	graphic_draw.graphic_data_struct[1].figure_name[2] = five5[2];
	graphic_draw.graphic_data_struct[1].graphic_type = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 1;
	graphic_draw.graphic_data_struct[1].width = 1;
	graphic_draw.graphic_data_struct[1].start_x = 65;
	graphic_draw.graphic_data_struct[1].start_y = 300;
	graphic_draw.graphic_data_struct[1].details_d = 65;
	graphic_draw.graphic_data_struct[1].details_e = 260;
	
	referee_send_five_graphic(MY_CLIENT_ID, &graphic_draw);
}

bool auto_state = true;
void send_autoaim_state()
{
	graphic_data_struct_t graphic_draw;
	char capname[3] = "409";
	graphic_draw.figure_name[0] = capname[0];
	graphic_draw.figure_name[1] = capname[1];
	graphic_draw.figure_name[2] = capname[2];
	
	if(auto_state){
		graphic_draw.operate_type = 1;
		auto_state = false;
	}
	else
		graphic_draw.operate_type = 2;
	
	graphic_draw.graphic_type = 1;
	graphic_draw.layer = 0;
	if(gimbal_ob == GIMBAL_AUTO)
		graphic_draw.color = 2;
	else
		graphic_draw.color = 1;
	
	graphic_draw.width = 10; 
	graphic_draw.start_x = 1790;
	graphic_draw.start_y = 610;
	
	graphic_draw.details_d = 1810;
	graphic_draw.details_e = 590;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

bool capdraw = true;
void send_capvol_graphic(int capvols)
{
	graphic_data_struct_t graphic_draw;
	char capname[3] = "209";
	graphic_draw.figure_name[0] = capname[0];
	graphic_draw.figure_name[1] = capname[1];
	graphic_draw.figure_name[2] = capname[2];
	if(capdraw){
	graphic_draw.operate_type = 1;
	capdraw = false;
	}
	else
	{
	graphic_draw.operate_type = 2;
	}
	
	graphic_draw.color = 1;
	graphic_draw.graphic_type = 0;
	graphic_draw.layer = 0;

	graphic_draw.width = 18; //线条宽度
	graphic_draw.start_x = 761;
	graphic_draw.start_y = 70;
	
	graphic_draw.details_d = capvols;
	graphic_draw.details_e = 70;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}



void send_single_icon(char* name, int x, int y, int type, int color)
{ //参数依次为：name：图形名称，作为客户端索引 
	//x,y:绘图起点坐标
	//type :图形类型，如直线、矩形、字符等
	//color:图形颜色
	graphic_data_struct_t graphic_draw;
	//float res = (float)residue_power/1800.0*100.0;
//	int pitchang=(MIDDLE_PITCH-CAN2_206.Current_position)*360/8192.0;
	graphic_draw.color = color;

	/*	graphic_draw.graphic_tpye = 0;

		if(first_cap_draw){
			graphic_draw.operate_tpye = 1;
			first_cap_draw = false;
		}
		else{
			graphic_draw.operate_tpye = 2;
		}*/
	graphic_draw.operate_type = type;
	graphic_draw.layer = 0;


	graphic_draw.width = 20; //线条宽度
	//graphic_draw.start_angle=10;
	//graphic_draw.end_angle=10;
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	graphic_draw.details_d = graphic_draw.start_x;
	graphic_draw.details_e = graphic_draw.start_y + 20;//+pitchang*10;
	//graphic_draw.radius = 30;
	//graphic_draw.text_lenght = strlen(value);
	memcpy(graphic_draw.figure_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}


//static bool first_multi_draw = true;
void send_multi_graphic()//向客户端发送七个图形的数据。每个图形包含起点终点、操作类型、颜色、线宽等属性。
{
	/*  
          	|
		————|————
	    ———|———
	      ——|——
	        —|—
	          |
	          |
	          |
	          |
	*/
	ext_interaction_figure_seven_t graphic_draw;
	
	//graph 1：一条线宽为3的黄色直线。起点为（960，200）终点为（960，540）
	
	//graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
//	if (first_multi_draw)
//	{//第一次画则设置模式为新增
//		first_multi_draw = false;
		graphic_draw.graphic_data_struct[0].operate_type = 1;
		graphic_draw.graphic_data_struct[1].operate_type = 1;
		graphic_draw.graphic_data_struct[2].operate_type = 1;
		graphic_draw.graphic_data_struct[3].operate_type = 1;
		graphic_draw.graphic_data_struct[4].operate_type = 1;
		graphic_draw.graphic_data_struct[5].operate_type = 1;
		graphic_draw.graphic_data_struct[6].operate_type = 1;
//	}

/*	else
	{//不是第一次画则设置模式为修改
		graphic_draw.graphic_data_struct[0].operate_tpye = 2;
		graphic_draw.graphic_data_struct[1].operate_tpye = 2;
		graphic_draw.graphic_data_struct[2].operate_tpye = 2;
		graphic_draw.graphic_data_struct[3].operate_tpye = 2;
		graphic_draw.graphic_data_struct[4].operate_tpye = 2;
		graphic_draw.graphic_data_struct[5].operate_tpye = 2;
		graphic_draw.graphic_data_struct[6].operate_tpye = 2;
	}*/
	char name1[3] = "701";
	graphic_draw.graphic_data_struct[0].figure_name[0] = name1[0];
	graphic_draw.graphic_data_struct[0].figure_name[1] = name1[1];
	graphic_draw.graphic_data_struct[0].figure_name[2] = name1[2];
	graphic_draw.graphic_data_struct[6].graphic_type = 0;
	//graphic_draw.graphic_data_struct[6].operate_tpye = 1;
	graphic_draw.graphic_data_struct[6].layer = 0;
	graphic_draw.graphic_data_struct[6].color = 2;
	graphic_draw.graphic_data_struct[6].width = 1;
	//graphic_draw.graphic_data_struct[6].start_angle=10;
	//graphic_draw.graphic_data_struct[6].end_angle=10;
	graphic_draw.graphic_data_struct[6].start_x = 900;
	graphic_draw.graphic_data_struct[6].start_y = 330;
	graphic_draw.graphic_data_struct[6].details_d = 1020;
	graphic_draw.graphic_data_struct[6].details_e = 330;
	//graphic_draw.graphic_data_struct[0].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[0].graphic_name, (uint8_t*)name1, strlen(name1));
	
	
	//graph 2：一条线宽为3的黄色直线，起点为（850，540），终点为（1070，540）
	char name2[3] = "702";
	graphic_draw.graphic_data_struct[1].figure_name[0] = name2[0];
	graphic_draw.graphic_data_struct[1].figure_name[1] = name2[1];
	graphic_draw.graphic_data_struct[1].figure_name[2] = name2[2];
	graphic_draw.graphic_data_struct[0].graphic_type = 0;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 2;
	graphic_draw.graphic_data_struct[0].width = 1;
	//graphic_draw.graphic_data_struct[0].start_angle=10;
	//graphic_draw.graphic_data_struct[0].end_angle=10;
	graphic_draw.graphic_data_struct[0].start_x = 954;
	graphic_draw.graphic_data_struct[0].start_y = 200;
	graphic_draw.graphic_data_struct[0].details_d = 954;
	graphic_draw.graphic_data_struct[0].details_e = 540;
	

	//graphic_draw.graphic_data_struct[1].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[1].graphic_name, (uint8_t*)name2, strlen(name2));
	
	
	//graph 3：也是线宽为3的黄色直线，起点为（860，520），终点为（1060，520）
	char name3[3] = "703";
	graphic_draw.graphic_data_struct[2].figure_name[0] = name3[0];
	graphic_draw.graphic_data_struct[2].figure_name[1] = name3[1];
	graphic_draw.graphic_data_struct[2].figure_name[2] = name3[2];
	graphic_draw.graphic_data_struct[2].graphic_type = 0;
	//graphic_draw.graphic_data_struct[2].operate_type = 1;
	graphic_draw.graphic_data_struct[2].layer = 0;
	graphic_draw.graphic_data_struct[2].color = 2;
	graphic_draw.graphic_data_struct[2].width = 1;
	//graphic_draw.graphic_data_struct[2].start_angle=10;
	//graphic_draw.graphic_data_struct[2].end_angle=10;
	graphic_draw.graphic_data_struct[2].start_x = 910;
	graphic_draw.graphic_data_struct[2].start_y = 390;
	graphic_draw.graphic_data_struct[2].details_d = 1010;
	graphic_draw.graphic_data_struct[2].details_e = 390;
	//graphic_draw.graphic_data_struct[2].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[2].graphic_name, (uint8_t*)name3, strlen(name3));
	
	
	//graph 4
	char name4[3] = "704";
	graphic_draw.graphic_data_struct[3].figure_name[0] = name4[0];
	graphic_draw.graphic_data_struct[3].figure_name[1] = name4[1];
	graphic_draw.graphic_data_struct[3].figure_name[2] = name4[2];
	graphic_draw.graphic_data_struct[3].graphic_type = 0;
	//graphic_draw.graphic_data_struct[3].operate_tpye = 1;
	graphic_draw.graphic_data_struct[3].layer = 0;
	graphic_draw.graphic_data_struct[3].color = 2;
	graphic_draw.graphic_data_struct[3].width = 1;
	//graphic_draw.graphic_data_struct[3].start_angle=10;
	//graphic_draw.graphic_data_struct[3].end_angle=10;
	graphic_draw.graphic_data_struct[3].start_x = 900;
	graphic_draw.graphic_data_struct[3].start_y = 400;
	graphic_draw.graphic_data_struct[3].details_d = 1020;
	graphic_draw.graphic_data_struct[3].details_e = 400;
	//graphic_draw.graphic_data_struct[3].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[3].graphic_name, (uint8_t*)name4, strlen(name4));
	//graph 5
	char name5[3] = "705";
	graphic_draw.graphic_data_struct[4].figure_name[0] = name5[0];
	graphic_draw.graphic_data_struct[4].figure_name[1] = name5[1];
	graphic_draw.graphic_data_struct[4].figure_name[2] = name5[2];
	graphic_draw.graphic_data_struct[4].graphic_type = 0;
	//graphic_draw.graphic_data_struct[4].operate_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;
	graphic_draw.graphic_data_struct[4].color = 2;
	graphic_draw.graphic_data_struct[4].width = 1;
	//graphic_draw.graphic_data_struct[4].start_angle=10;
	//graphic_draw.graphic_data_struct[4].end_angle=10;
	graphic_draw.graphic_data_struct[4].start_x = 900;
	graphic_draw.graphic_data_struct[4].start_y = 300;
	graphic_draw.graphic_data_struct[4].details_d = 1020;
	graphic_draw.graphic_data_struct[4].details_e = 300;
	//graphic_draw.graphic_data_struct[4].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[4].graphic_name, (uint8_t*)name5, strlen(name5));
	//graph 6
	char name6[3] = "706";
	graphic_draw.graphic_data_struct[5].figure_name[0] = name6[0];
	graphic_draw.graphic_data_struct[5].figure_name[1] = name6[1];
	graphic_draw.graphic_data_struct[5].figure_name[2] = name6[2];
	graphic_draw.graphic_data_struct[5].graphic_type = 0;
	//graphic_draw.graphic_data_struct[5].operate_tpye = 1;
	graphic_draw.graphic_data_struct[5].layer = 0;
	graphic_draw.graphic_data_struct[5].color = 2;
	graphic_draw.graphic_data_struct[5].width = 1;
	//graphic_draw.graphic_data_struct[5].start_angle=10;
	//graphic_draw.graphic_data_struct[5].end_angle=10;
	graphic_draw.graphic_data_struct[5].start_x = 890;
	graphic_draw.graphic_data_struct[5].start_y = 310;
	graphic_draw.graphic_data_struct[5].details_d = 1030;
	graphic_draw.graphic_data_struct[5].details_e = 310;
	//graphic_draw.graphic_data_struct[5].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[5].graphic_name, (uint8_t*)name6, strlen(name6));
	//graph 7
	char name7[3] = "707";
	graphic_draw.graphic_data_struct[6].figure_name[0] = name7[0];
	graphic_draw.graphic_data_struct[6].figure_name[1] = name7[1];
	graphic_draw.graphic_data_struct[6].figure_name[2] = name7[2];
	graphic_draw.graphic_data_struct[6].graphic_type = 0;
	//graphic_draw.graphic_data_struct[6].operate_tpye = 1;
	graphic_draw.graphic_data_struct[6].layer = 0;
	graphic_draw.graphic_data_struct[6].color = 2;
	graphic_draw.graphic_data_struct[6].width = 1;
	//graphic_draw.graphic_data_struct[6].start_angle=10;
	//graphic_draw.graphic_data_struct[6].end_angle=10;
	graphic_draw.graphic_data_struct[6].start_x = 880;
	graphic_draw.graphic_data_struct[6].start_y = 320;
	graphic_draw.graphic_data_struct[6].details_d = 1040;
	graphic_draw.graphic_data_struct[6].details_e = 320;
	//graphic_draw.graphic_data_struct[6].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[6].graphic_name, (uint8_t*)name7, strlen(name7));

	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
}



void Send_Middle_rectangle(int level, int color, int x_length, int y_length)
{
	graphic_data_struct_t graphic_draw2;
	char name_temp = (char)(level + 30);
	char* name = &name_temp;
	graphic_draw2.operate_type = 1;
	graphic_draw2.graphic_type = 2;
	graphic_draw2.layer = level;
	graphic_draw2.color = color;
	graphic_draw2.width = 2;
	graphic_draw2.start_x = (int)(960 - x_length / 2.0f);
	graphic_draw2.start_y = (int)(540 - y_length / 2.0f);
	graphic_draw2.details_d = (int)(960 + x_length / 2.0f);
	graphic_draw2.details_e = (int)(540 + y_length / 2.0f);
	memcpy(graphic_draw2.figure_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw2);
}

//declared but never used
//static bool first_string_draw = true;
 void  send_string(char* str, char* name, int x, int y, int upd, int colour)
{
	int len = strlen(str) + 1;
	ext_client_custom_character_t character_data;
	character_data.graphic_data_struct.graphic_type = 7;//string

	character_data.graphic_data_struct.operate_type = upd;//modify

	character_data.graphic_data_struct.layer = 0;

	character_data.graphic_data_struct.color = colour;//green

	character_data.graphic_data_struct.width = 2;
	character_data.graphic_data_struct.details_a = 10;//size
	character_data.graphic_data_struct.details_b = len;//length
	character_data.graphic_data_struct.start_x = x;
	character_data.graphic_data_struct.start_y = y;

	memcpy(character_data.data, (uint8_t*)str, len);
	memcpy(character_data.graphic_data_struct.figure_name, (uint8_t*)name, strlen(name));

	referee_send_client_character(MY_CLIENT_ID, &character_data);
}

