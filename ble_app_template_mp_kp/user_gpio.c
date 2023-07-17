/**@file	user_gpio.c
 * 
 * @brief	本文件为gpio相关功能的用户层文件
 *
 * @detail	本文件包含application使用gpio相关功能的接口函数。
 *			同时作为nrf5 SDK 的上层封装
 */
 
/**< private include. */
#include "user_gpio.h"

 
/**@brief	初始化IO引脚为输出
 * 
 * @detail	引脚状态：
 *			DIR 	--> OUTPUT
 *			INPUT 	--> DISCONNECT
 *			PULL 	--> NO PULL
 * 			DRIVE 	--> STANDARD 0, STANDARD 1
 *			SENSE 	--> NO SENSE 
 *
 *@param[in]	pin_num	引脚号
 *
 *@retval	NONE
 */
void user_gpio_output_init(u_pin_num_t pin_num)
{
	nrf_gpio_cfg_output(pin_num);
}



/**/
