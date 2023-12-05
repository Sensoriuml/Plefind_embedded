/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "OLED_Driver.h"
#include "OLED_GUI.h"


#include "DAT_Buffer.h"
#include "SIN_Sinus.h"
#include "VIEW_Code.h"
#include "HW_Hardware.h"
#include "stm32f4xx_it.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void PowerOnInit(void);
void SinusForceZero(void);

volatile uint32_t AutoPowerOffTimer_100ms;

//-------------------------------------------
uint8_t PotError[2];

#define AMP_I2C_ADDRESS  2
void ChangeAmplifier(uint8_t aId, uint8_t aValue)
{
uint8_t buf[3];
   aId&=0x07;
   aValue&=0x7F;
   aValue<<=1;

   buf[0]=0x58+(aId<<1);// adres: 0101 +hw 000 + 0(write)
   buf[1]=0x00; // memadres[7:4] cmd[3:2] extdata[1:0]
   buf[2]=aValue;
   if(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)buf[0],&buf[1], 2, 1000)!= HAL_OK)
   {
	   PotError[aId&0x01]++;
	   return;
	   //if (HAL_I2C_GetError(&hi2c3) != HAL_I2C_ERROR_AF)
	   //{
	   //	       break;//Error_Handler();
	   //}
	     /* Error_Handler() function is called when Timeout error occurs.
	        When Acknowledge failure occurs (Slave don't acknowledge its address)
	        Master restarts communication */
	     //if (HAL_I2C_GetError(&hi2c3) != HAL_I2C_ERROR_AF)
	    //{
	       ;//Error_Handler();
	    // }
   }
   PotError[aId&0x01]=0;

}

void PowerOff(char * aText)
{
	SIN_SinusDisable();
	HW_BuzzOut(1);
    LL_GPIO_ResetOutputPin(LEDRED_PORT,LEDRED_PIN);
    LL_GPIO_ResetOutputPin(LEDBLUE_PORT,LEDGREEN_PIN);
    LL_GPIO_ResetOutputPin(LEDGREEN_PORT,LEDGREEN_PIN);

	DAT_OutSendText("\n\r");
	if(aText) DAT_OutSendText(aText);
	DAT_OutSendText("\n\r");
    Timer_1ms=0;
	while(Timer_1ms<1000);
	AutoPowerOffTimer_100ms=0;
	HW_BuzzOut(0);
	LL_GPIO_SetOutputPin(POWERON_GPIO_Port, POWERON_Pin);
    Timer_1ms=0;
	while(Timer_1ms<2000);
	while(1);


}

void MX_TIM7_Reinit(void)
{

	LL_TIM_DisableIT_UPDATE(TIM7);
	LL_TIM_DisableCounter(TIM7);
	MX_TIM7_Init();
	LL_TIM_EnableIT_UPDATE(TIM7);
	LL_TIM_EnableCounter(TIM7);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  HW_ReadConfig();

  HW_BuzzOut(1);
  HAL_Delay(200);
  HW_BuzzOut(0);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  PowerOnInit();

  HW_BuzzOut(1);
  HAL_Delay(200);
  HW_BuzzOut(0);

  OLED_Init(SCAN_DIR_DFT);
  GUI_Show();

  OLED_Clear(OLED_BACKGROUND);//OLED_BACKGROUND
  OLED_Display();


  HAL_Delay(100);


  LL_TIM_EnableCounter(TIM5);
  LL_TIM_EnableCounter(TIM7);

  LL_TIM_EnableIT_UPDATE(TIM5);
  LL_TIM_EnableIT_UPDATE(TIM7);

  //LL_USART_Enable(USART2);

  LL_USART_EnableIT_RXNE(USER_UART);
  LL_USART_EnableIT_RXNE(BLE_UART);

  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);

  //LL_DAC_Enable(DAC1,  LL_DAC_CHANNEL_1);
  //LL_DAC_Enable(DAC1,  LL_DAC_CHANNEL_2);


  VIEW_Params|=0x80;

  VIEW_LEd12Switch=0;

  ChangeAmplifier(0,VIEW_Amplifier[0]);
  ChangeAmplifier(1,VIEW_Amplifier[1]);

/*
  //     ledy
  #define LEDRED_PORT		LED3_RED_GPIO_Port
  #define LEDRED_PIN		LED3_RED_Pin

  #define LEDGREEN_PORT	LED2_GREEN_GPIO_Port
  #define LEDGREEN_PIN	LED2_GREEN_Pin

  #define LEDBLUE_PORT	LED1_BLUE_GPIO_Port
  #define LEDBLUE_PIN		LED1_BLUE_Pin

#define USER_BUTTON_DOWN() 		(!(LL_GPIO_IsInputPinSet(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin)))
#define USER_BUTTON_UP() 		(LL_GPIO_IsInputPinSet(USER_BUTTON_GPIO_Port,USER_BUTTON_Pin))

#define BLE_UART 		USART3
#define USER_UART 		USART2
*/
  /*aT+umsm=0 -AT COMMAND
   * AT+UMSM=1 - DATA
   */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    LL_GPIO_SetOutputPin(RESET_NINA_GPIO_Port,RESET_NINA_Pin);


	int i;
	uint8_t update,updatefull;

	if(USER_BUTTON_DOWN())
	{
		  LL_USART_DisableIT_RXNE(USER_UART);
		  LL_USART_DisableIT_RXNE(BLE_UART);
		  //LL_TIM_DisableIT_UPDATE(TIM5);
		  LL_TIM_DisableIT_UPDATE(TIM7);
		  i=0;

		  Timer_1ms=0;

		  while (1)
		  {
			  if(Timer_1ms>100000L)
			  {
				  PowerOff("Config power off");
			  }
			  if(i==0)
			  {
				  if(Timer_1ms>5000) LL_GPIO_SetOutputPin(LEDBLUE_PORT,LEDBLUE_PIN);
				  if(!USER_BUTTON_DOWN())
				  {
					  i=1;
					  if(Timer_1ms>5000)
					  {
						  LL_GPIO_ResetOutputPin(LEDBLUE_PORT,LEDBLUE_PIN);
						  LL_USART_EnableIT_RXNE(BLE_UART);
						  LL_USART_EnableIT_RXNE(USER_UART);
						  DAT_OutSendText("BLE configuring...\x0D");

						  DAT_OutSendText("+++");
						  DAT_OutSendText2("+++");
						  Timer_1ms=0;
						  while(Timer_1ms<4000);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  DAT_OutSendText("AT+UBTLE=2\x0D");
						  DAT_OutSendText2("AT+UBTLE=2\x0D");
						  Timer_1ms=0;
						  while(Timer_1ms<400);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  DAT_OutSendText("AT+UBTCM=2\x0D");
						  DAT_OutSendText2("AT+UBTCM=2\x0D");
						  Timer_1ms=0;
						  while(Timer_1ms<400);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  DAT_OutSendText("AT+UBTDM=3\x0D");
						  DAT_OutSendText2("AT+UBTDM=3\x0D");
						  Timer_1ms=0;
						  while(Timer_1ms<400);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  DAT_OutSendText("AT+UMSM=1\x0D");
						  DAT_OutSendText2("AT+UMSM=1\x0D");
						  Timer_1ms=0;
						  while(Timer_1ms<400);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  DAT_OutSendText("AT&W\x0D");
						  DAT_OutSendText2("AT&W\x0D");
						  Timer_1ms=0;
						  while(Timer_1ms<400);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  DAT_OutSendText("AT+CPWROFF\x0D");
						  DAT_OutSendText2("AT+CPWROFF\x0D");
						  Timer_1ms=0;
						  while(Timer_1ms<1000);
						  LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
						  LL_USART_DisableIT_RXNE(USER_UART);
						  LL_USART_DisableIT_RXNE(BLE_UART);

					  }
				  }
			  }
			  if(USER_BUTTON_DOWN())  LL_GPIO_TogglePin(LEDGREEN_PORT,LEDGREEN_PIN);

			  if(LL_USART_IsActiveFlag_RXNE(USER_UART))
			  {
				  uint8_t val = LL_USART_ReceiveData8(USER_UART);
		          LL_GPIO_TogglePin(LEDGREEN_PORT,LEDGREEN_PIN);
				  LL_USART_TransmitData8(USER_UART, val);
				  LL_USART_TransmitData8(BLE_UART, val);
				  Timer_1ms=0;
			  }
				if(LL_USART_IsActiveFlag_RXNE(BLE_UART))
				{
					  uint8_t val = LL_USART_ReceiveData8(BLE_UART);
					LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
					LL_USART_TransmitData8(USER_UART, val);
				}
		  }

	}

	while (1)
	{
	  DAT_Buffer_Loop();
	  if(VIEW_Amplifier[0]&0x80)
	  {
		  VIEW_Amplifier[0]&=0x7F;
		  ChangeAmplifier(0,VIEW_Amplifier[0]);
	  }
	  if(VIEW_Amplifier[1]&0x80)
	  {
		  VIEW_Amplifier[1]&=0x7F;
		  ChangeAmplifier(1,VIEW_Amplifier[1]);
	  }
	  if(VIEW_Params&0x80)
	  {
		  SIN_Stop();
		  if(VIEW_SlowF&0x80)
			  MX_TIM7_Reinit();

		  VIEW_SlowF&=0x7F;
		  VIEW_OutSinMode&=0x01;
		  VIEW_OffMode&=0x7F;
		  VIEW_Params&=0x01;
		  VIEW_PhaseOffset&=0x1FF;
		  VIEW_AvMulti&=0x7F;
		  VIEW_Offset01p[0]&=0x7FFF;
		  VIEW_Sinus01pp[0]&=0x7FFF;
		  VIEW_Offset01p[1]&=0x7FFF;
		  VIEW_Sinus01pp[1]&=0x7FFF;
		  VIEW_Det&=0x7F;
		  VIEW_Run&=0x7F;
		  SIN_Run=VIEW_Run;

		  SIN_GenerateDacTable(VIEW_Offset01p[1], VIEW_Sinus01pp[1], 1 );
		  SIN_GenerateDacTable(VIEW_Offset01p[0], VIEW_Sinus01pp[0], 0 );
		  if(VIEW_OffMode && VIEW_Run)
		  {
			  HW_BuzzOut(1);
			  HAL_Delay(50);
			  HW_BuzzOut(0);

			  SIN_Start(0xFFFFFFFF,(VIEW_SlowF&0x01)?0:8);
			  LL_GPIO_ResetOutputPin(LEDRED_PORT,LEDRED_PIN);
		  }
		  else
		  {
			  LL_GPIO_SetOutputPin(LEDRED_PORT,LEDRED_PIN);
			  LCK_ClearAcumulate();

			  HW_BuzzOut(1);
			  HAL_Delay(20);
			  HW_BuzzOut(0);
			  HAL_Delay(40);
			  HW_BuzzOut(1);
			  HAL_Delay(20);
			  HW_BuzzOut(0);

//			  LCK_OutLockData();
		  }
		  SIN_Mode = (SIN_SwapIt&0x03)+((SIN_Det&0x03)<<4);

	  }
	  if(VIEW_Noice&0x80)
	  {
		  VIEW_Noice&=0x7F;
		  SIN_NoiceLevel=(VIEW_Noice*0xFFFL)/100;
	  }
	  update=0;
	  if(SIN_Det&0x01)
	  {
		  update+= LCK_UpdateSums(&LCK_AcuData[0]);
		  update+= LCK_UpdateSums(&LCK_AcuData[1]);
	  }
#ifdef ADC2_ENABLED
	  if(SIN_Det&0x02)
	  {
		  update+= LCK_UpdateSums(&LCK_AcuData2[0]);
		  update+= LCK_UpdateSums(&LCK_AcuData2[1]);
	  }
#endif
	  if(update)
	  {
		  Timer_ViewUpdate_10ms=0;
		  LCK_OutLockData(0);
	  }
	  if(!Timer_ViewUpdate_10ms)
	  {
		  Timer_ViewUpdate_10ms=20;
		  VIEW_ShowOutSinus();
		  continue;
	  }

	  if(Timer_100ms_Flag)
	  {
		  if(++AutoPowerOffTimer_100ms>(10*60*10L))
		  {
			  PowerOff("AutoPowerOff");
		  }

		  Timer_100ms_Flag=0;
		  CT_ScanButtons_100ms();
		  CT_KeyLoop();
		  DAT_Loop100ms();
		  if(SIN_SquaredSinus && (VIEW_Run&0x01))
			  LL_GPIO_TogglePin(LEDRED_PORT,LEDRED_PIN);
//		  if((VIEW_OffMode&0x7F)==3) LCK_OutData();
	  }
	  if(Timer_800ms_Flag)
	  {
		  Timer_800ms_Flag=0;
		 // if(PotError[0])
			  ChangeAmplifier(0, VIEW_Amplifier[0]);
		 // if(PotError[1])
			  ChangeAmplifier(1, VIEW_Amplifier[1]);
	     if(!SIN_State) LCK_OutLockData(1);
	     LL_GPIO_TogglePin(LEDGREEN_PORT,LEDGREEN_PIN);

	  }
  /***************************************
  while (1)
  {


	  LL_USART_TransmitData8(USER_UART,'*');

	  LL_GPIO_SetOutputPin(RESET_NINA_GPIO_Port,RESET_NINA_Pin);
	  uint32_t vvv;
	  while (1)
	  {
		  if(USER_BUTTON_DOWN())  LL_GPIO_TogglePin(LEDGREEN_PORT,LEDGREEN_PIN);

		  if(LL_USART_IsActiveFlag_RXNE(USER_UART))
		  {
			  uint8_t val = LL_USART_ReceiveData8(USER_UART);
	          LL_GPIO_TogglePin(LEDGREEN_PORT,LEDGREEN_PIN);
			  LL_USART_TransmitData8(USER_UART, val);
			  LL_USART_TransmitData8(BLE_UART, val);
			  if(val==0x33334444) break;
			  vvv++;
		  }
			if(LL_USART_IsActiveFlag_RXNE(BLE_UART))
			{
				  uint8_t val = LL_USART_ReceiveData8(BLE_UART);
				LL_GPIO_TogglePin(LEDBLUE_PORT,LEDBLUE_PIN);
				LL_USART_TransmitData8(USER_UART, val);
				vvv--;
	    		  if(val==0x3334444) break;
			}
	  }
	  *********************************/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration  
  PB0   ------> ADC1_IN8 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  LL_ADC_DisableIT_EOCS(ADC1);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC2 GPIO Configuration  
  PB1   ------> ADC2_IN9 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC2, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  LL_ADC_DisableIT_EOCS(ADC2);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**DAC GPIO Configuration  
  PA4   ------> DAC_OUT1
  PA5   ------> DAC_OUT2 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC channel OUT1 config 
  */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  /** DAC channel OUT2 config 
  */
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  /* TIM5 interrupt Init */
  NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(TIM5_IRQn);

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 80000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM5);
  LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM5);
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 625;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration  
  PC10   ------> USART3_TX
  PC11   ------> USART3_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_BLUE_Pin|KEYPAD_CS_Pin|BUZZER_OUT_Pin|DATACMD_OLED_Pin 
                          |OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_RESET_Pin|LED2_GREEN_Pin|LED3_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_NINA_GPIO_Port, RESET_NINA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_BLUE_Pin KEYPAD_CS_Pin BUZZER_OUT_Pin DATACMD_OLED_Pin 
                           OLED_CS_Pin */
  GPIO_InitStruct.Pin = LED1_BLUE_Pin|KEYPAD_CS_Pin|BUZZER_OUT_Pin|DATACMD_OLED_Pin 
                          |OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RESET_Pin LED2_GREEN_Pin LED3_RED_Pin */
  GPIO_InitStruct.Pin = OLED_RESET_Pin|LED2_GREEN_Pin|LED3_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_NINA_Pin */
  GPIO_InitStruct.Pin = RESET_NINA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_NINA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT2_INT_AKC_Pin */
  GPIO_InitStruct.Pin = EXT2_INT_AKC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT2_INT_AKC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void PowerOnInit(void)
{
//#define POWERON_Pin GPIO_PIN_9 // 0 wlaczenie, 1 wylaczenie
//#define POWERON_GPIO_Port GPIOB

GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = POWERON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(POWERON_GPIO_Port, &GPIO_InitStruct);

	LL_GPIO_ResetOutputPin(POWERON_GPIO_Port, POWERON_Pin);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
