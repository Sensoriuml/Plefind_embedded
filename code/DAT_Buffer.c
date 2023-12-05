/*
 * BUF_Buffer.c
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "main.h"

#include "DAT_Buffer.h"
#include "PAR_Parser.h"
#include "HW_Hardware.h"

volatile uint32_t  DAT_OutPutIdx, DAT_OutGetIdx;
uint8_t   DAT_OutBuffer	[DAT_OUTSIZE];

volatile uint32_t  DAT_InPutIdx, DAT_InGetIdx;
uint8_t   DAT_InBuffer	[DAT_INSIZE];

volatile uint8_t DAT_InPutData;
uint8_t DAT_UsbLock;

void DAT_AddInBuffer(uint8_t aValue);
//------------------------------------------------------
void DAT_Loop100ms(void)
{
	if(DAT_UsbLock) DAT_UsbLock--;
}
//------------------------------------------------------
/**
 * void DAT_OutBufferClear(void)
 * czyszczenie bufora wysylania
 */

void DAT_OutBufferClear(void)
{
	DAT_OutGetIdx = DAT_OutPutIdx = 0;
}
//------------------------------------------------------
/**
 * void DAT_InBufferClear(void)
 * czyszczenie bufora odbierania
 */

void DAT_InBufferClear(void)
{
	DAT_InGetIdx = DAT_InPutIdx = 0;
}
//------------------------------------------------------
/**
 * uint16_t DAT_OutGetNextData(void)
 * odbior 8bitowej danej z bufora
 * @return - zwraca dan¹, 0xFFFF jesli brak danych
 */

uint16_t DAT_OutGetNextData(void)
{
uint8_t val;
uint32_t idx;
	if(DAT_OutGetIdx == DAT_OutPutIdx) return 0xFFFF;
	val = DAT_OutBuffer[DAT_OutGetIdx];
	idx = DAT_OutGetIdx+1;
	if(idx>=DAT_OUTSIZE) idx = 0;
	DAT_OutGetIdx = idx;
	return val;
}
//------------------------------------------------------
/**
 * void DAT_OutSendByte(uint8_t aData)
 * wstawienie 8bitowej danej do bufora nadawczego i wywolanie przerwania
 *
 */
void DAT_OutSendByte(uint8_t aData)
{
uint32_t nextidx;
	nextidx = (DAT_OutPutIdx+1)&(DAT_OUTSIZE-1);
	if(DAT_OutGetIdx == nextidx) return;
	DAT_OutBuffer[DAT_OutPutIdx] = aData;
	DAT_OutPutIdx = nextidx;
	if(!LL_USART_IsEnabledIT_TXE(BLE_UART))
		LL_USART_EnableIT_TXE(BLE_UART);
}
//------------------------------------------------------
/**
 * void DAT_OutSendBuffer(uint8_t * aData, uint8_t aLength)
 * wstawienie danych do bufora i wywolania przerwania
 * @param aData - wskaznik
 * @param aLength -ilosc bajtow
 *
 */

void DAT_OutSendBuffer(uint8_t * aData, uint8_t aLength)
{
	while(aLength--) DAT_OutSendByte(*aData++);
}
//------------------------------------------------------
/**
 * void DAT_OutSendBuffer(uint8_t * aData, uint8_t aLength)
 * wstawienie danych do bufora i wywolania przerwania
 * @param aData - wskaznik
 * @param aLength -ilosc bajtow
 *
 */

void DAT_OutSendText(char  * aText)
{
	while(*aText)
		DAT_OutSendByte(*(aText++));
}
//------------------------------------------------------
/**
 * void DAT_OutSendTextLN(char  * aText)
 * wstawienie ciagu tekstowego ze znakiem konca linii do bufora i wywolania przerwania
 * @param aData - wskaznik
 * @param aLength -ilosc bajtow
 *
 */

void DAT_OutSendTextLN(char  * aText)
{
	while(*aText)
		DAT_OutSendByte(*(aText++));
	DAT_OutSendByte(0x0D);
	DAT_OutSendByte(0x0A);
}
//------------------------------------------------------
/**
 * void DAT_InAddData(uint8_t aData)
 * wstawienie bajtu do bufora i wywolania przerwania
 */

void DAT_InAddData(uint8_t aData, uint8_t aSource)
{
	DAT_InPutData=aData;
	DAT_UsbLock=30;
	DAT_AddInBuffer(aData);
	//PAR_DataReceived(aData);
}
//------------------------------------------------------
//##################################################################################
volatile uint32_t  DAT_OutPutIdx2, DAT_OutGetIdx2;
uint8_t   DAT_OutBuffer2	[DAT_OUTSIZE];

volatile uint32_t  DAT_InPutIdx2, DAT_InGetIdx2;
uint8_t   DAT_InBuffer2	[DAT_INSIZE];

volatile uint8_t DAT_InPutData2;
//------------------------------------------------------
/**
 * void DAT_OutBufferClear(void)
 * czyszczenie bufora wysylania
 */

void DAT_OutBufferClear2(void)
{
	DAT_OutGetIdx2 = DAT_OutPutIdx2 = 0;
}
//------------------------------------------------------
/**
 * void DAT_InBufferClear(void)
 * czyszczenie bufora odbierania
 */

void DAT_InBufferClear2(void)
{
	DAT_InGetIdx2 = DAT_InPutIdx2 = 0;
}
//------------------------------------------------------
/**
 * uint16_t DAT_OutGetNextData(void)
 * odbior 8bitowej danej z bufora
 * @return - zwraca dan¹, 0xFFFF jesli brak danych
 */

uint16_t DAT_OutGetNextData2(void)
{
uint8_t val;
uint32_t idx;
	if(DAT_OutGetIdx2 == DAT_OutPutIdx2) return 0xFFFF;
	val = DAT_OutBuffer2[DAT_OutGetIdx2];
	idx = DAT_OutGetIdx2+1;
	if(idx>=DAT_OUTSIZE) idx = 0;
	DAT_OutGetIdx2 = idx;
	return val;
}
//------------------------------------------------------
/**
 * void DAT_OutSendByte(uint8_t aData)
 * wstawienie 8bitowej danej do bufora nadawczego i wywolanie przerwania
 *
 */
void DAT_OutSendByte2(uint8_t aData)
{
uint32_t nextidx;

	nextidx = (DAT_OutPutIdx2+1)&(DAT_OUTSIZE-1);
	if(DAT_OutGetIdx2 == nextidx) return;
	DAT_OutBuffer2[DAT_OutPutIdx2] = aData;
	DAT_OutPutIdx2 = nextidx;
	if(!LL_USART_IsEnabledIT_TXE(USER_UART))
		LL_USART_EnableIT_TXE(USER_UART);

}
//------------------------------------------------------
/**
 * void DAT_OutSendBuffer(uint8_t * aData, uint8_t aLength)
 * wstawienie danych do bufora i wywolania przerwania
 * @param aData - wskaznik
 * @param aLength -ilosc bajtow
 *
 */

void DAT_OutSendBuffer2(uint8_t * aData, uint8_t aLength)
{
	while(aLength--) DAT_OutSendByte2(*aData++);
}
//------------------------------------------------------
/**
 * void DAT_OutSendBuffer(uint8_t * aData, uint8_t aLength)
 * wstawienie danych do bufora i wywolania przerwania
 * @param aData - wskaznik
 * @param aLength -ilosc bajtow
 *
 */

void DAT_OutSendText2(char  * aText)
{
	while(*aText)
		DAT_OutSendByte2(*(aText++));
}
/*
void DAT_OutSendText2(char  * aText)
{
	while(*aText)
	{
		if(LL_USART_IsActiveFlag_TC(USART3))
				 LL_USART_TransmitData8(USART3,*(aText++));
	}
}
*/

//------------------------------------------------------
/**
 * void DAT_OutSendTextLN(char  * aText)
 * wstawienie ciagu tekstowego ze znakiem konca linii do bufora i wywolania przerwania
 * @param aData - wskaznik
 * @param aLength -ilosc bajtow
 *
 */

void DAT_OutSendTextLN2(char  * aText)
{
	while(*aText)
		DAT_OutSendByte2(*(aText++));
	DAT_OutSendByte2(0x0D);
	DAT_OutSendByte2(0x0A);
}
//------------------------------------------------------
/**
 * void DAT_InAddData(uint8_t aData)
 * wstawienie bajtu do bufora i wywolania przerwania
 */


uint32_t DAT_InBytes;
void DAT_InAddData2(uint8_t aData, uint8_t aSrc)
{
	DAT_InBytes++;
	if(DAT_UsbLock) return;
	DAT_AddInBuffer(aData);
	//PAR_DataReceived(aData);
	DAT_InPutData2=aData;
}
//------------------------------------------------------
void DAT_AddInBuffer(uint8_t aValue)
{
uint32_t idx;
	idx=(DAT_InPutIdx+1)&(DAT_INSIZE-1);
	if(idx==DAT_InGetIdx) return;
	DAT_InBuffer[DAT_InPutIdx]=aValue;
	DAT_InPutIdx=idx;


}
//------------------------------------------------------
void DAT_Buffer_Loop(void)
{
uint32_t idx;
	while(DAT_InPutIdx!=DAT_InGetIdx)
	{
		PAR_DataReceived(DAT_InBuffer[DAT_InGetIdx]);
		idx=(DAT_InGetIdx+1)&(DAT_INSIZE-1);
		DAT_InGetIdx=idx;
	}
}
//------------------------------------------------------
