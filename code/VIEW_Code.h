/*
 * View_Code.h
 *
 *  Created on: 07.09.2018
 *      Author: Adam
 */

#ifndef VIEW_CODE_H_
#define VIEW_CODE_H_

#include "stdint.h"

void VIEW_ShowOutSinus(void);

extern uint8_t VIEW_OutSinMode,VIEW_OffMode,VIEW_SlowF,VIEW_Save,VIEW_Params,VIEW_Noice,VIEW_AvMulti,VIEW_Det,VIEW_Run;
extern uint16_t VIEW_PhaseOffset;

extern uint16_t VIEW_Offset01p[];
extern uint16_t VIEW_Sinus01pp[];
extern uint16_t VIEW_HalfSinus[]; // w jednostkach DAC

extern uint8_t VIEW_LEd12Switch;

extern uint8_t VIEW_Amplifier[];

extern uint8_t VIEW_DisplayOff;

#endif /* VIEW_CODE_H_ */
