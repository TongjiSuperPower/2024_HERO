/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM����ϵͳ���ݴ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "shoot.h"
#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "supercap_task.h"
#include "gimbal_behaviour.h"
extern gimbal_behaviour_e gimbal_ob;

extern cap_control_t cap_control;

/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

//extern ext_robot_command_t     ext_robot_command; 
extern shoot_control_t shoot_control;//shoot state
extern bool spinning_state;
extern bool snipeMode;
extern bool capdraw;
extern bool first_pitch_draw;
extern bool auto_state;
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ����ϵͳ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
//int UI_flag = 0 ;
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();//Ϊ����ʢװ���ݵĸ����ṹ�����ռ䡣
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);//��ʼ�����ڽ�������ϵͳ��Ϣ�Ķ��л���
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);//ͨ��6�Ŵ����շ����Ե�Դ����ģ������ݣ���������ģ��ͨ��WIFI�������ͨ�š�
		//USART6ͨ�������жϽ������ݡ�
	
		uint16_t UI_PushUp_Counter = 261;
		/* ����ϵͳ��ʼ�� */
		vTaskDelay(300);
	
    while(1)
    {		
				referee_unpack_fifo_data();//��������
				vTaskDelay(10);
			
				UI_PushUp_Counter++;
		
				
			if(UI_PushUp_Counter>=1000)
			{UI_PushUp_Counter = 10;}
			
			
		  //aim line
			if(UI_PushUp_Counter % 500 == 0){
				capdraw = true;
				first_pitch_draw = true;
				auto_state = true;
				send_multi_graphic();
				
				continue;
			}
			if(UI_PushUp_Counter % 600 == 0){
				
				//send_text_graphic("009");
				send_double_text();
				continue;
			}
			if(UI_PushUp_Counter % 7 ==0){
				//REST ENERGY
			send_string_test();
			
			continue;
			}
		/*	if(UI_PushUp_Counter % 47 ==0){
				
				if(snipeMode ==1)
				{send_snipeMode(1);}
				else
				{send_snipeMode(3);}
				
			continue;
			}*/
			if(UI_PushUp_Counter % 17 ==0){
				//REST ENERGY
			send_capvol_graphic(761+(cap_control.cap_message->cap_vol-18000)/90*4);
			
			continue;
			}
		
		
			
			if(UI_PushUp_Counter % 37 ==0){
				//STATE OF RUB
				if (shoot_control.fric_state == FRIC_ON)
				{
					send_rub_graphic("004", 1800,500,2,1);//green
				}
				else
				{
					send_rub_graphic("004", 1800,500,2,3);
				}
			
				continue;
			}
			if(UI_PushUp_Counter % 23 ==0){
					send_autoaim_state();
				continue;
			}
				
			if(UI_PushUp_Counter % 31 ==0){
				//STATE OF SPINNING
				if (spinning_state == 1)
			  {
					send_spinning_graphic("003", 1800,700,2,1);//green
				}
				else
				{
					send_spinning_graphic("003", 1800,700,3,3);
				}
				continue;
			}

    }
}




/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
    }
}



