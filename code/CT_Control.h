/*
 * CT_Control.h
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */

#ifndef CT_CONTROL_H_
#define CT_CONTROL_H_

#include "stdint.h"

#define CT_TOTAL_BUTTONS 1


#define CT_BUTTON_SW0  0
#define CT_BUTTON_SW1  1
#define CT_BUTTON_SW2  2
#define CT_BUTTON_SW3  3
#define CT_BUTTON_FUN1 4
#define CT_BUTTON_FUN2 5
#define CT_BUTTON_FUN3 6
#define CT_BUTTON_FUN4 7
#define CT_BUTTON_FUN5 8

#define CT_BUTTON0_DOWN() (!(CT_ButtonState[0]&0x08))
#define CT_BUTTON0_UP()   (CT_ButtonState[0]&0x08)


#define CT_BUTTON_FUN1_DOWN() (!(CT_ButtonState[4]&0x08))
#define CT_BUTTON_FUN1_UP()   (CT_ButtonState[4]&0x08)
#define CT_BUTTON_FUN2_DOWN() (!(CT_ButtonState[5]&0x08))
#define CT_BUTTON_FUN2_UP()   (CT_ButtonState[5]&0x08)
#define CT_BUTTON_FUN3_DOWN() (!(CT_ButtonState[6]&0x08))
#define CT_BUTTON_FUN3_UP()   (CT_ButtonState[6]&0x08)

#define CT_BUTTON0_PUSHED() (BYTE0(CT_ButtonChanged[0])==0x40)
#define CT_BUTTON0_PULLED() (BYTE0(CT_ButtonChanged[0])==0xFF)
#define CT_BUTTON0_CLEAR()  CT_ButtonChanged[0]=0

#define CT_BUTTON_CODE_SW0_PUSHED 0x40  // (wcisniecie SW0 czyli koniec puszczenia przycisku SW0)
#define CT_BUTTON_CODE_SW1_PUSHED 0x41  // (wcisniecie SW1 czyli koniec puszczenia przycisku SW1)
#define CT_BUTTON_CODE_SW2_PUSHED 0x42
#define CT_BUTTON_CODE_SW3_PUSHED 0x43

#define CT_BUTTON_CODE_SW0_PULLED 0xF0  //  (puszczenie FUN1 czyli koniec wcisniecia przycisku FUN1)
#define CT_BUTTON_CODE_SW1_PULLED 0xF1   //  (puszczenie FUN1 czyli koniec wcisniecia przycisku FUN2)
#define CT_BUTTON_CODE_SW2_PULLED 0xF2
#define CT_BUTTON_CODE_SW3_PULLED 0xF3


#define CT_BUTTON_CODE_FUN1_PUSHED 0x44  //  (wcisniecie FUN1 czyli koniec puszczenia przycisku FUN1)
#define CT_BUTTON_CODE_FUN2_PUSHED 0x45
#define CT_BUTTON_CODE_FUN3_PUSHED 0x46
#define CT_BUTTON_CODE_FUN4_PUSHED 0x47
#define CT_BUTTON_CODE_FUN5_PUSHED 0x48

#define CT_BUTTON_CODE_FUN1_PULLED 0xF4
#define CT_BUTTON_CODE_FUN2_PULLED 0xF5
#define CT_BUTTON_CODE_FUN3_PULLED 0xF6
#define CT_BUTTON_CODE_FUN4_PULLED 0xF7
#define CT_BUTTON_CODE_FUN5_PULLED 0xF8

#define CT_BUTTON_CODE_PULLMASK 0xF0
#define CT_BUTTON_CODE_IDXMASK  0x0F



#define CT_BUTTON_CODE()  (BYTE0(CT_ButtonCode))



uint8_t CT_Toogleled	(uint32_t aDivider);

void CT_ScanButtons_10ms	(void);
void CT_ScanButtons_100ms	(void);


extern uint8_t  CT_ButtonFilters[CT_TOTAL_BUTTONS];  // FF wcisniety, 00 puszczony
extern uint32_t CT_ButtonState  [CT_TOTAL_BUTTONS];  // BYTE[3:1] czas w 10*milisekundy trwania, BYTE[0] - 0 puszczony, 0xFF wcisniety
extern uint32_t CT_ButtonChanged[CT_TOTAL_BUTTONS];  // BYTE[3:1] czas w 10*milisekundy zakonczenia zdarzenia, BYTE[0] - typ zakonczenia(!) zdarzenia: 0 puszczony, 0xFF wcisniety, po wykorzystaniu zerowac!
extern uint32_t CT_ButtonCode; // jw = nr przycisku



void CT_RunStop(uint8_t aRun);



#endif /* CT_CONTROL_H_ */
