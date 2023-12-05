/******************************************************************************
**************************Hardware interface layer*****************************
* | file      	:	DEV_Config.h
* |	version		:	V1.0
* | date		:	2017-08-14
* | function	:	
	Provide the hardware underlying interface	
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"

#define USE_SPI_4W 1
#define USE_IIC 0

#define IIC_CMD        0X00
#define IIC_RAM        0X40

//OLED GPIO
#define OLED_CS_0		HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
#define OLED_CS_1		HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)

#define OLED_DC_0		HAL_GPIO_WritePin(DATACMD_OLED_GPIO_Port, DATACMD_OLED_Pin, GPIO_PIN_RESET)
#define OLED_DC_1		HAL_GPIO_WritePin(DATACMD_OLED_GPIO_Port, DATACMD_OLED_Pin, GPIO_PIN_SET)

#define OLED_RST_0		HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_RESET)
#define OLED_RST_1		HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_SET)

//SPI GPIO
#define SPI2_SCK_0		HAL_GPIO_WritePin(SPI2_SCK_GPIO_Port, SPI2_SCK_Pin, GPIO_PIN_RESET)
#define SPI2_SCK_1		HAL_GPIO_WritePin(SPI2_SCK_GPIO_Port, SPI2_SCK_Pin, GPIO_PIN_SET)

#define SPI2_MOSI_0		HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_RESET)
#define SPI2_MOSI_1		HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_SET)
/*------------------------------------------------------------------------------------------------------*/

uint8_t System_Init(void);
void    System_Exit(void);

uint8_t SPI4W_Write_Byte(uint8_t value);
void I2C_Write_Byte(uint8_t value, uint8_t Cmd);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

#endif
