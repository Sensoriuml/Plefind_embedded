/*
 * BUF_Buffer.h
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */

#ifndef DAT_BUFFER_H_
#define DAT_BUFFER_H_


#define DAT_OUTSIZE 1024
#define DAT_INSIZE  1024


void DAT_OutBufferClear(void);
void DAT_InBufferClear(void);

uint16_t 	DAT_OutGetNextData		(void);
void 		DAT_InAddData			(uint8_t aData, uint8_t aSource);
void 	DAT_OutSendTextLN		(char  * aText);
void 	DAT_OutSendText			(char  * aText);
void 	DAT_OutSendBuffer		(uint8_t * aData, uint8_t aLength);
void 	DAT_OutSendByte			(uint8_t aData);

extern volatile uint32_t  DAT_OutPutIdx, DAT_OutGetIdx;
extern uint8_t   DAT_OutBuffer	[DAT_OUTSIZE];

extern volatile uint32_t  DAT_InPutIdx, DAT_InGetIdx;
extern uint8_t   DAT_InBuffer	[DAT_INSIZE];

extern volatile uint8_t DAT_InPutData;


//#####################################

void DAT_OutBufferClear2(void);
void DAT_InBufferClear2(void);

uint16_t 	DAT_OutGetNextData2		(void);
void 		DAT_InAddData2			(uint8_t aData, uint8_t aSrc);
void 	DAT_OutSendTextLN2		(char  * aText);
void 	DAT_OutSendText2			(char  * aText);
void 	DAT_OutSendBuffer2		(uint8_t * aData, uint8_t aLength);
void 	DAT_OutSendByte2			(uint8_t aData);

extern volatile uint32_t  DAT_OutPutIdx2, DAT_OutGetIdx2;
extern uint8_t   DAT_OutBuffer2	[DAT_OUTSIZE];

extern volatile uint32_t  DAT_InPutIdx2, DAT_InGetIdx2;
extern uint8_t   DAT_InBuffer2	[DAT_INSIZE];

extern volatile uint8_t DAT_InPutData2;

extern uint32_t DAT_InBytes;

extern uint8_t DAT_UsbLock;

void DAT_Loop100ms(void);

void DAT_Buffer_Loop(void);

#endif /* DAT_BUFFER_H_ */
