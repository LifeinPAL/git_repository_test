/**@file	user_gpio.c
 * 
 * @brief	���ļ�Ϊgpio��ع��ܵ��û����ļ�
 *
 * @detail	���ļ�����applicationʹ��gpio��ع��ܵĽӿں�����
 *			ͬʱ��Ϊnrf5 SDK ���ϲ��װ
 */
 
/**< private include. */
#include "user_gpio.h"

 
/**@brief	��ʼ��IO����Ϊ���
 * 
 * @detail	����״̬��
 *			DIR 	--> OUTPUT
 *			INPUT 	--> DISCONNECT
 *			PULL 	--> NO PULL
 * 			DRIVE 	--> STANDARD 0, STANDARD 1
 *			SENSE 	--> NO SENSE 
 *
 *@param[in]	pin_num	���ź�
 *
 *@retval	NONE
 */
void user_gpio_output_init(u_pin_num_t pin_num)
{
	nrf_gpio_cfg_output(pin_num);
}



/**/
