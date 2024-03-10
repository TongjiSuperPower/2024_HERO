/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
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

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

const error_t *error_list_test_local;

/**
 * @brief          test task
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          test任务
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void test_task(void const *argument)
{
//    static uint8_t error, last_error;
//    static uint8_t error_num;
    error_list_test_local = get_error_list_point();
    while (1)
    {
        // ??????????????????
        osDelay(10);
    }
}
