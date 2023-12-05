/*
 * CT_Control.c
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */
#include "stdint.h"
#include "stm32f4xx.h"

#include "main.h"
#include "CT_Control.h"

#include "OLED_Driver.h"
#include "OLED_GUI.h"

#include "SIN_Sinus.h"
#include "VIEW_Code.h"
#include "HW_Hardware.h"



#define BYTE0(d) (*(uint8_t*)&d)
//-------------------------------------------------------------------------------
/**
 * uint8_t CT_Toogleled(uint32_t aDivider)
 * przelacza LED na stan preciwny
 * zwraca 1 gdy przelaczone
 * @param aDivider - dzielnik wywolan do przelaczenia diody
 */

uint8_t CT_Toogleled(uint32_t aDivider)
{
static uint32_t div;
	if(++div<aDivider) return 0;
	div=0;
	//LL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	return 1;

}

uint8_t  CT_ButtonFilters[CT_TOTAL_BUTTONS];  // FF wcisniety, 00 puszczony
uint32_t CT_ButtonState  [CT_TOTAL_BUTTONS];  // BYTE[3:1] czas w 10*milisekundy trwania, BYTE[0] - 0 puszczony, 0xFF wcisniety
uint32_t CT_ButtonChanged[CT_TOTAL_BUTTONS];  // BYTE[3:1] czas w 10*milisekundy zakonczenia zdarzenia, BYTE[0] - typ zakonczenia(!) zdarzenia: 0 puszczony, 0xFF wcisniety, po wykorzystaniu zerowac!
uint32_t CT_ButtonCode; // jw = nr przycisku


//-------------------------------------------------------------------------
void CT_ScanButtons_10ms(void)
{
	CT_ButtonFilters[CT_BUTTON_SW0]<<=1;
	if(USER_BUTTON_DOWN()) CT_ButtonFilters[CT_BUTTON_SW0]|=0x01;
	/*
	CT_ButtonFilters[CT_BUTTON_SW0]<<=1;
	if(!(SW0_GPIO_Port->IDR & SW0_Pin)) CT_ButtonFilters[CT_BUTTON_SW0]|=0x01;

	CT_ButtonFilters[CT_BUTTON_SW1]<<=1;
	if(!(SW1_GPIO_Port->IDR & SW1_Pin)) CT_ButtonFilters[CT_BUTTON_SW1]|=0x01;

	CT_ButtonFilters[CT_BUTTON_SW2]<<=1;
	if(!(SW2_GPIO_Port->IDR & SW2_Pin)) CT_ButtonFilters[CT_BUTTON_SW2]|=0x01;

	CT_ButtonFilters[CT_BUTTON_SW3]<<=1;
	if(!(SW3_GPIO_Port->IDR & SW3_Pin)) CT_ButtonFilters[CT_BUTTON_SW3]|=0x01;

	CT_ButtonFilters[CT_BUTTON_FUN1]<<=1;
	if(!(FUN1_GPIO_Port->IDR & FUN1_Pin)) CT_ButtonFilters[CT_BUTTON_FUN1]|=0x01;

	CT_ButtonFilters[CT_BUTTON_FUN2]<<=1;
	if(!(FUN2_GPIO_Port->IDR & FUN2_Pin)) CT_ButtonFilters[CT_BUTTON_FUN2]|=0x01;

	CT_ButtonFilters[CT_BUTTON_FUN3]<<=1;
	if(!(FUN3_GPIO_Port->IDR & FUN3_Pin)) CT_ButtonFilters[CT_BUTTON_FUN3]|=0x01;

	CT_ButtonFilters[CT_BUTTON_FUN4]<<=1;
	if(!(FUN4_GPIO_Port->IDR & FUN4_Pin)) CT_ButtonFilters[CT_BUTTON_FUN4]|=0x01;

	CT_ButtonFilters[CT_BUTTON_FUN5]<<=1;
	if(!(FUN5_GPIO_Port->IDR & FUN5_Pin)) CT_ButtonFilters[CT_BUTTON_FUN5]|=0x01;
	*/
}	
//-------------------------------------------------------------------------
void CT_ScanButtons_100ms(void)
{


  for(uint8_t i=0;i<CT_TOTAL_BUTTONS;i++)
  {
	CT_ButtonState[i]+=0x100;	

	if((uint8_t)((CT_ButtonFilters[i]+1))>1) continue; // zaklocenia
	if(!(uint8_t)((CT_ButtonState[i]^CT_ButtonFilters[i]))) continue; // brak zmiany
		// zmiana stanu
	CT_ButtonChanged[i] = CT_ButtonState[i]|0x40; // 00 nop, 0x40 wcisniecie, 0xFF puszczenie
	CT_ButtonCode = ((CT_ButtonChanged[i]&0xFFFFFFF0) | i);
	CT_ButtonState[i] = CT_ButtonFilters[i];
  }	 
}	
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
/** uint8_t CT_GetKey(void)
 *  @return - zwraca kod wcisnietego przycisku, zero jesli nic nie wcisniete
 */

uint32_t CT_KeyCode;
uint8_t CT_GetKey(void)
{
uint8_t code;
	code = CT_BUTTON_CODE();
	if(!code) return 0;
	CT_KeyCode = CT_ButtonChanged[0];
	CT_ButtonCode = 0;
	return code; 
}
//-------------------------------------------------------------------------------
void CT_ClearChanged(void)
{
	for(uint8_t i=0;i<CT_TOTAL_BUTTONS;i++) CT_ButtonChanged[i]=0;   
}
//-----------------------------------
/**
 *  uint8_t CT_KeyLoop(void)
 *  obsluga funkcji przyciskow
 *  @return 1 gdy parametr funkcyjny zmieniony
 */
uint8_t CT_KeyLoop(void)
{
uint8_t key,idx;
int16_t val;
uint8_t mode=0;
char txt[40];

	key = CT_GetKey();
	mode = VIEW_LEd12Switch;

	if(!key)
	{
		//if(((CT_ButtonState[CT_BUTTON_SW0]&0xFF)==0xFF) && (CT_ButtonState[CT_BUTTON_SW0]>0x2FFF)) goto SW0_Pushed;
		//if(((CT_ButtonState[CT_BUTTON_SW1]&0x0FF)==0x0FF) && (CT_ButtonState[CT_BUTTON_SW1]>0x6FF)) goto SW1_Pushed;
		//if(((CT_ButtonState[CT_BUTTON_SW2]&0x0FF)==0x0FF) && (CT_ButtonState[CT_BUTTON_SW2]>0x6FF)) goto SW2_Pushed;
		//if(((CT_ButtonState[CT_BUTTON_SW3]&0x0FF)==0x0FF) && (CT_ButtonState[CT_BUTTON_SW3]>0x6FF)) goto SW3_Pushed;
		//if(((CT_ButtonState[CT_BUTTON_FUN5]&0x0FF)==0x0FF) && (CT_ButtonState[CT_BUTTON_FUN5]>0x6FF)) goto FUN5_Pushed;
	//	if(((CT_ButtonState[CT_BUTTON_FUN2]&0x0FF)==0x0FF) && (CT_ButtonState[CT_BUTTON_FUN2]>0x6FF)) goto FUN2_Pushed;
		return 0;
	}
//	if(CT_BUTTON_FUN2_DOWN()) mode=1;
	switch(key)
	{
	/*
		case CT_BUTTON_CODE_SW3_PUSHED:
							SW3_Pushed: if(!mode) return;
										if(mode>2)
										{
											mode=(mode-3)&0x01;
											VIEW_Amplifier[mode]=(VIEW_Amplifier[mode]+1)|0x80;
											VIEW_Save=1;
											return 1;
										}
										mode--;
										val = VIEW_Offset01p[mode]+10;
										if((val+(VIEW_Sinus01pp[mode]/2)) > 999)  val=999-(VIEW_Sinus01pp[mode]/2);//return 0;
										if((val-(VIEW_Sinus01pp[mode]/2)) <0)     val=(VIEW_Sinus01pp[mode]/2);//return 0;
										VIEW_Offset01p[mode] = val;
										VIEW_Params|=0x80;
										VIEW_Save=1;
										return 1;

		case CT_BUTTON_CODE_SW0_PUSHED:
							SW0_Pushed:
										if(!mode) return;
										if(mode>2)
										{
											mode=(mode-3)&0x01;
											VIEW_Amplifier[mode]=(VIEW_Amplifier[mode]-1)|0x80;
											VIEW_Save=1;
											return 1;
										}
										mode--;
										val = VIEW_Offset01p[mode]-10;
										if(val<1) val=1;
										if((val+(VIEW_Sinus01pp[mode]/2)) > 999) val=999-(VIEW_Sinus01pp[mode]/2);//return 0;
										if((val-(VIEW_Sinus01pp[mode]/2)) <0) 	  val=(VIEW_Sinus01pp[mode]/2);//return 0;
										VIEW_Offset01p[mode] = val;
										VIEW_Params|=0x80;
										VIEW_Save=1;
										return 1;
		case CT_BUTTON_CODE_SW1_PUSHED:
							SW1_Pushed:
										if(!mode) return;
										if(mode>2)
										{
											mode=(mode-3)&0x01;
											VIEW_Amplifier[mode]=(VIEW_Amplifier[mode]-5)|0x80;
											VIEW_Save=1;
											return 1;

										}
										mode--;
										val = VIEW_Sinus01pp[mode]-10;
										if(val<1) val=1;
										if((VIEW_Offset01p[mode]-(val/2)) <0) 	 val=2*(VIEW_Offset01p[mode]);//return 0;
										if((VIEW_Offset01p[mode]+(val/2)) > 999) val=2*(999-(VIEW_Offset01p[mode]));//return 0;
										VIEW_Sinus01pp[mode] = val;
										VIEW_Params|=0x80;
										VIEW_Save=1;
										return 1;
		case CT_BUTTON_CODE_SW2_PUSHED:
							SW2_Pushed:
										if(!mode) return;
										if(mode>2)
										{
											mode=(mode-3)&0x01;
											VIEW_Amplifier[mode]=(VIEW_Amplifier[mode]+5)|0x80;
											VIEW_Save=1;
											return 1;
										}
										mode--;
										val = VIEW_Sinus01pp[mode]+10;
										if((VIEW_Offset01p[mode]-(val/2)) <0) 	 val=2*(VIEW_Offset01p[mode]);//return 0;
										if((VIEW_Offset01p[mode]+(val/2)) > 999) val=2*(999-(VIEW_Offset01p[mode]));//return 0;
										VIEW_Sinus01pp[mode] = val;
										VIEW_Params|=0x80;
										VIEW_Save=1;
										return 1;
		case CT_BUTTON_CODE_FUN1_PUSHED:HW_StoreConfig();
										VIEW_Noice=0x80;
										VIEW_Save=0;
										return 1;
		case CT_BUTTON_CODE_FUN3_PUSHED:
										VIEW_OffMode=(VIEW_OffMode&0x7F)+1;
										if(VIEW_OffMode>3) VIEW_OffMode=0;
										VIEW_OffMode|=0x80;
										VIEW_Params|=0x80;
										CT_RunStop(1);
										VIEW_Save=1;
										return 1;
		case CT_BUTTON_CODE_FUN2_PUSHED:
										VIEW_LEd12Switch=(VIEW_LEd12Switch+1);
										if(VIEW_LEd12Switch>4) VIEW_LEd12Switch=0;
						FUN2_Pushed:
										return 1;

	SW0_Pushed:
		case CT_BUTTON_CODE_SW0_PUSHED:
										VIEW_Det=(VIEW_Det+1)&0x7F;
										if(VIEW_Det>2) VIEW_Det=0;
										VIEW_Det|=0x80;
										CT_RunStop(1);

										VIEW_Save=1;
										VIEW_Params|=0x80;
										return 1;
		case CT_BUTTON_CODE_FUN5_PUSHED:
						FUN5_Pushed: */
	/*SW0_Pushed:
				CT_ButtonState[CT_BUTTON_SW0]=0;
				SIN_SquaredSinus=!SIN_SquaredSinus;
				CT_RunStop((VIEW_Run&0x01));
				return 1;
				*/
		case CT_BUTTON_CODE_SW0_PULLED:
		//case CT_BUTTON_CODE_SW0_PUSHED:
										AutoPowerOffTimer_100ms=0;
										sprintf(txt,"\n\r%d\n\r",CT_KeyCode>>8);
										DAT_OutSendText(txt);
										if((CT_KeyCode>>8)>30)
										{
											// POWER oFF
											PowerOff("POWER off");
										}
										else
										{
											CT_RunStop(!(VIEW_Run&0x01));
										}
										//VIEW_DisplayOff=((VIEW_DisplayOff+1)&0x03)|0x80;
										return 1;


		default: return 0;
	}
	return 0;
}
//-------------------------------------------
void CT_RunStop(uint8_t aRun)
{
	if(aRun && !(VIEW_Run&0x01))
  	{
 	  	  	  	  VIEW_Params|=0x80;
 	  	  	  	  VIEW_Run=1|0x80;
	}
	else
    if(!aRun && (VIEW_Run&0x01))
	{
		VIEW_Params|=0x80;
		VIEW_Run=0|0x80;
	}
}

		/*
 *          SW3
 * 		SW1    SW2
 * 	        SW0
 */
