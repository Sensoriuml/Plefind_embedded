/*
 * HW_Hardware.h
 *
 *  Created on: 12.09.2018
 *      Author: Adam
 */

#ifndef HW_HARDWARE_H_
#define HW_HARDWARE_H_

#define HW_VERSION 1

#define HW_DIGIPOT 2 // 1 - nieadresowalny 7bit
					 // 2 - adresowalny 8bit MCP4552


uint8_t HW_StoreData(uint32_t * aBuffer, uint8_t aLength);
void 	HW_GetData	(uint32_t * aBuffer, uint8_t aLength);


uint8_t HW_ReadConfig(void);
void    HW_StoreConfig(void);


uint8_t HW_ReadConfig(void);

void 	HW_OutSendText(uint8_t *aBuffer, uint16_t aLength);

//-----------------------------------------------
//     uarty
#define BLE_UART 		USART2
#define USER_UART 		USART3
//-----------------------------------------------

//     potencjometry
#define DIGIPOT1_I2C_handle 	hi2c3
#define DIGIPOT2_I2C_handle 	hi2c3
//-----------------------------------------------

//     przycisk
#define USER_BUTTON_ON 		(!(USER_BUTTON_GPIO_Port>IDR & USER_BUTTON_Pin))

#define USER_BUTTON_DOWN() 		(!(LL_GPIO_IsInputPinSet(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin)))
#define USER_BUTTON_UP() 		(LL_GPIO_IsInputPinSet(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin))

//-----------------------------------------------

//     detektory
#define DET1_CHANNEL	LL_ADC_CHANNEL_9
#define DET1_ADC		ADC2

#define DET2_CHANNEL	LL_ADC_CHANNEL_8
#define DET2_ADC		ADC1
//-----------------------------------------------

//     ledy
#define LEDRED_PORT		LED3_RED_GPIO_Port
#define LEDRED_PIN		LED3_RED_Pin

#define LEDGREEN_PORT	LED2_GREEN_GPIO_Port
#define LEDGREEN_PIN	LED2_GREEN_Pin

#define LEDBLUE_PORT	LED1_BLUE_GPIO_Port
#define LEDBLUE_PIN		LED1_BLUE_Pin
//-----------------------------------------------

#define SPI_ACC_handle		hspi1
//-----------------------------------------------

//     led generatora
#define SINOUT1_DAC_DAC		DAC1
#define SINOUT1_CHANNEL_DAC	LL_DAC_CHANNEL_1

#define SINOUT2_DAC_DAC		DAC1
#define SINOUT2_CHANNEL_DAC	LL_DAC_CHANNEL_2

//-----------------------------------------------


//    user button





#endif /* HW_HARDWARE_H_ */
