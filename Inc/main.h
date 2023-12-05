/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dac.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern volatile uint32_t AutoPowerOffTimer_100ms;

void PowerOff(char * aText);


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_BLUE_Pin GPIO_PIN_13
#define LED1_BLUE_GPIO_Port GPIOC
#define WKUP_ACK_Pin GPIO_PIN_0
#define WKUP_ACK_GPIO_Port GPIOA
#define USART2_TX_EXT_Pin GPIO_PIN_2
#define USART2_TX_EXT_GPIO_Port GPIOA
#define UART2_RX_EXT_Pin GPIO_PIN_3
#define UART2_RX_EXT_GPIO_Port GPIOA
#define SINUS_OUT1_Pin GPIO_PIN_4
#define SINUS_OUT1_GPIO_Port GPIOA
#define SINUS_OUT2_Pin GPIO_PIN_5
#define SINUS_OUT2_GPIO_Port GPIOA
#define KEYPAD_CS_Pin GPIO_PIN_4
#define KEYPAD_CS_GPIO_Port GPIOC
#define ADC1_IN8_SIN2_Pin GPIO_PIN_0
#define ADC1_IN8_SIN2_GPIO_Port GPIOB
#define ADC2_IN9_SIN1_Pin GPIO_PIN_1
#define ADC2_IN9_SIN1_GPIO_Port GPIOB
#define USER_BUTTON_Pin GPIO_PIN_2
#define USER_BUTTON_GPIO_Port GPIOB
#define OLED_RESET_Pin GPIO_PIN_12
#define OLED_RESET_GPIO_Port GPIOB
#define SPI2_SCK_OLED_Pin GPIO_PIN_13
#define SPI2_SCK_OLED_GPIO_Port GPIOB
#define SPI2_MISO_OLEDKEY_Pin GPIO_PIN_14
#define SPI2_MISO_OLEDKEY_GPIO_Port GPIOB
#define SPI2_MOSI_OLED_Pin GPIO_PIN_15
#define SPI2_MOSI_OLED_GPIO_Port GPIOB
#define BUZZER_OUT_Pin GPIO_PIN_6
#define BUZZER_OUT_GPIO_Port GPIOC
#define DATACMD_OLED_Pin GPIO_PIN_7
#define DATACMD_OLED_GPIO_Port GPIOC
#define OLED_CS_Pin GPIO_PIN_8
#define OLED_CS_GPIO_Port GPIOC
#define I2C3_SDA_PERIPH_Pin GPIO_PIN_9
#define I2C3_SDA_PERIPH_GPIO_Port GPIOC
#define I2C3_SCL_PERIPH_Pin GPIO_PIN_8
#define I2C3_SCL_PERIPH_GPIO_Port GPIOA
#define RESET_NINA_Pin GPIO_PIN_15
#define RESET_NINA_GPIO_Port GPIOA
#define UART3_TX_NNA_Pin GPIO_PIN_10
#define UART3_TX_NNA_GPIO_Port GPIOC
#define UART3_RX_NINA_Pin GPIO_PIN_11
#define UART3_RX_NINA_GPIO_Port GPIOC
#define EXT2_INT_AKC_Pin GPIO_PIN_2
#define EXT2_INT_AKC_GPIO_Port GPIOD
#define SPI1_SCK_AKC_Pin GPIO_PIN_3
#define SPI1_SCK_AKC_GPIO_Port GPIOB
#define SPI1_MISO_AKC_Pin GPIO_PIN_4
#define SPI1_MISO_AKC_GPIO_Port GPIOB
#define SPI1_MOSI_AKC_Pin GPIO_PIN_5
#define SPI1_MOSI_AKC_GPIO_Port GPIOB
#define I2C1_SCL_TMP_Pin GPIO_PIN_6
#define I2C1_SCL_TMP_GPIO_Port GPIOB
#define I2C1_SDA_TMP_Pin GPIO_PIN_7
#define I2C1_SDA_TMP_GPIO_Port GPIOB
#define LED2_GREEN_Pin GPIO_PIN_8
#define LED2_GREEN_GPIO_Port GPIOB
#define LED3_RED_Pin GPIO_PIN_9
#define LED3_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define POWERON_Pin GPIO_PIN_10 // 0 wlaczenie, 1 wylaczenie
#define POWERON_GPIO_Port GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
