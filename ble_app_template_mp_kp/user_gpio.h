/**@file
 *
 */
 


#ifndef __USER_GPIO_H__
#define __USER_GPIO_H__

/**< private include >*/
#include "nrf_gpio.h"

/**< private define >*/
#define u_pin_num_t	uint32_t

/**@brief	Macro for set pin to high.
 *
 * @param[in]	_pin_num	Number of pin to be set.
 */
#define USER_GPIO_HIGH(_pin_num)	\
		nrf_gpio_pin_set(_pin_num)
		
/**@brief	Macro for set pin to low.
 *
 * @param[in]	_pin_num	Number of pin to be set.
 */
#define USER_GPIO_LOW(_pin_num)		\
		nrf_gpio_pin_clear(_pin_num)

/**< export variable >*/


/**< export function prototype. */

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
 void user_gpio_output_init(u_pin_num_t pin_num);

#endif // __USER_GPIO_H__


