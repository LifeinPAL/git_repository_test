/**@file
 *
 */
 


#ifndef __MAIN_H__
#define __MAIN_H__
/**< private include >*/
#include "nrf.h"


/**< global define >*/
typedef struct{
	const char* p_frame_head;
	uint8_t door_state;
	uint16_t battery_voltage;
	uint8_t wkp_source;
	uint16_t timestamp;
	uint32_t g_packet_no;
	uint8_t err_code;
	const char* p_frame_end;	
}g_data_collection;



/**< private define >*/


#define USER_PIN_LED_1		NRF_GPIO_PIN_MAP(0,15)
#define USER_PIN_LED_2      NRF_GPIO_PIN_MAP(0,17)


#define USER_PIN_MAGNET		NRF_GPIO_PIN_MAP(0,20)

#define USER_PIN_UART_RX	NRF_GPIO_PIN_MAP(0,11)
#define USER_PIN_UART_TX	NRF_GPIO_PIN_MAP(1,9)

#define USER_PIN_BM_ADC		NRF_GPIO_PIN_MAP(0,5)
#define USER_PIN_CE_ADC		NRF_GPIO_PIN_MAP(0,4)



/**< export variable >*/


/**< export function prototype. */


#endif // __MAIN_H__


