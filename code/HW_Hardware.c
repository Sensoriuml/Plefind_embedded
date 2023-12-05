/*
 * HW_Hardware.c
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */

#include "stdint.h"
#include "stm32f4xx.h"
#include "string.h"

#include "stm32f4xx_hal.h"

#include "main.h"
#include "SIN_Sinus.h"
#include "VIEW_Code.h"


#define HW_SETUP_ADDRESS  		0x8004000
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x8004000) /* Base address of Sector 3, 16 Kbytes */

//--------------------------------------------------------------------------------------------
/**
 * HW_StoreData(uint32_t * aBuffer, uint8_t aLength)
 * Zapis danych do pamieci flash pod adres HW_SETUP_ADDRESS  0x0800C000, sektor 16kbajtow.
 * weryfikacja czy jest potrzeba zapisu, jesli tak
 * to zapis wykonany i zwraca 1, jesli zawartosc identyczna to zwraca 0
 * @param aLength - ilosc 32bitowych slow
 * @param aBuffer - wskaznik do bufora -zapisywane sa pelne 32-bitowe slowa
 */
uint8_t HW_StoreData(uint32_t * aBuffer, uint8_t aLength)
{
uint32_t adres;
uint32_t se = 0;

FLASH_EraseInitTypeDef EraseInitStruct;

	adres = HW_SETUP_ADDRESS|0x08000000;
	if(!memcmp(aBuffer,(uint8_t*)adres,aLength)) return 0;

	  HAL_FLASH_Unlock();

	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
      EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
      EraseInitStruct.Sector        = FLASH_SECTOR_1;
	  EraseInitStruct.NbSectors     = 1;
	  HAL_FLASHEx_Erase(&EraseInitStruct, &se);

	  while(aLength--)
	  {
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adres, *aBuffer++);
	      adres += 4;
	  }

	  // Lock the Flash to disable the flash control register access (recommended
	  //   to protect the FLASH memory against possible unwanted operation)
	  HAL_FLASH_Lock();
	  return 1;

}
//----------------------------------------------
/**
 * uint8_t HW_ReadConfig(void)
 *  Odczyt konfiguracji z pamieci flash z pod adresu  HW_SETUP_ADDRESS  0x0800C000
 *  zwraca 1 gdy ok, 0 gdy error
 */
uint8_t HW_ReadConfig(void)
{
uint8_t buffer[100];
uint16_t val1,val2;
	memcpy((uint32_t*)buffer,(uint8_t*)HW_SETUP_ADDRESS,100);
	if((buffer[0]==0xAA) && (buffer[99]==0x55))
	{
		val1= *(uint16_t*)&buffer[4];
		val2= *(uint16_t*)&buffer[6];
		if((val1>1000)  || (val2>1000)) goto errorit;
		VIEW_Sinus01pp[0]=val1;
		VIEW_Offset01p[0]=val2;

		VIEW_SlowF = 10;//buffer[9]&0x7F;
		//if(!VIEW_SlowF) VIEW_SlowF=10;

		VIEW_OffMode = 1;//buffer[10]&0x7F;
		VIEW_OutSinMode =buffer[11]&0x7F;
		//VIEW_AvMulti = buffer[12]&0x7F;
		VIEW_AvMulti=15;
		//VIEW_PhaseOffset = (*(uint16_t*)&buffer[12])&0x1FF;
		//VIEW_PhaseOffset=(VIEW_PhaseOffset&0x1FF)%360;

		val1= *(uint16_t*)&buffer[16];
		val2= *(uint16_t*)&buffer[18];
		if((val1>1000)  || (val2>1000)) goto errorit;
		VIEW_Sinus01pp[1]=val1;
		VIEW_Offset01p[1]=val2;

		VIEW_Amplifier[0]=buffer[20]&0x7F;
		VIEW_Amplifier[1]=buffer[21]&0x7F;

		VIEW_Det=buffer[22]&0x7F;



		return 1;
	}
	errorit:

	VIEW_Sinus01pp[1] = 300;
	VIEW_Offset01p[1] = 590;

	VIEW_Sinus01pp[0] = 380;
	VIEW_Offset01p[0] = 470;

	VIEW_OffMode = 1;
	VIEW_OutSinMode = 1;
	VIEW_SlowF = 10;
	VIEW_AvMulti = 15;
	VIEW_Amplifier[0] = 0;
	VIEW_Amplifier[1] = 0;
	VIEW_Det=2;



	return 0;
}
//----------------------------------------------
/**
 *   void HW_StoreConfig(void)
 *   Zapis konfiguracji do pamieci flash z pod adresu  HW_SETUP_ADDRESS  0x0800C000, max 100 bajtow, sektor 16kbajtow.
 *   zapisywane w kolejnosci:
 *   [0] 1 bajt id
 *   [1] 3 bajty = 0
 *   [4] 2 bajty modulacja sinusa w 0.1% pp zakresu
 *   [6] 2 bajty offset skladowej stalej w 0.1% zakresu
 *   [7] 1 bajt not used
 *   [9] 1 bajt - tryb sinusa wolno-szybko
 *   [10] 1 bajt - on/OFF
 *   [11] Tryb sinuss/lockin
 *
 *
 *
 */
void HW_StoreConfig(void)
{
uint8_t buffer[100];
	memset(buffer,0,100);
	buffer[0]=0xAA;
	buffer[99]=0x55;
	*(uint16_t*)&buffer[4]=SIN_Sinus01pp[0];
	*(uint16_t*)&buffer[6]=SIN_Offset01p[0];
	buffer[9]=VIEW_SlowF&0x7F;
	buffer[10]=VIEW_OffMode&0x7F;
	buffer[11]=VIEW_OutSinMode&0x7F;
	buffer[12]=VIEW_AvMulti&0x7F;

	*(uint16_t*)&buffer[16]=SIN_Sinus01pp[1];
	*(uint16_t*)&buffer[18]=SIN_Offset01p[1];
	buffer[20]=VIEW_Amplifier[0]&0x7F;
	buffer[21]=VIEW_Amplifier[1]&0x7F;
	buffer[22]=VIEW_Det&0x7F;

	HW_StoreData((uint32_t*)buffer,100/4);
}
//--------------------------------------------------------------------------------------------
/*
extern UART_HandleTypeDef huart3;
void HW_OutSendText(uint8_t *aBuffer, uint16_t aLength)
{
	  HAL_UART_Transmit_IT(&huart3,aBuffer,aLength);
}
*/
//--------------------------------------------------------------------------------------------
void HW_BuzzOut(uint8_t aBuzzing)
{
	if(aBuzzing)
		 LL_GPIO_SetOutputPin(BUZZER_OUT_GPIO_Port,BUZZER_OUT_Pin);
	else LL_GPIO_ResetOutputPin(BUZZER_OUT_GPIO_Port,BUZZER_OUT_Pin);
}


