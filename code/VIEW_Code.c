/*
 * VIEW_Code.c
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */

#include <string.h>

#include "SIN_Sinus.h"
#include "CT_Control.h"
#include "DAT_Buffer.h"

#include "OLED_Driver.h"
#include "OLED_GUI.h"


#include "VIEW_Code.h"

extern volatile uint32_t Timer_1ms;


uint8_t VIEW_OutSinMode,VIEW_OffMode,VIEW_SlowF,VIEW_Save,VIEW_Params,VIEW_Noice,VIEW_Det,VIEW_Run;
uint8_t VIEW_AvMulti;
uint16_t VIEW_PhaseOffset;
// phase offset w

uint16_t VIEW_Offset01p[2];
uint16_t VIEW_Sinus01pp[2];
uint16_t VIEW_HalfSinus[2]; // w jednostkach DAC
uint8_t VIEW_Amplifier[2];
uint8_t VIEW_DisplayOff;

uint8_t VIEW_LEd12Switch;

extern uint8_t PotError[];



static const char TxtProgress[4]={'|','/','-','\\'};
//static const char * TxtSwitchMode[]={"LED1","LED2","P1","P2"};

/**
 * void VIEW_ShowOutSinus(void)
 *  wyswietla glowny ekran
 */

void VIEW_ShowLockin(char * aText, uint8_t aX, uint8_t aY, LCK_LOCKINDATA * aLin, sFONT * aFont, char * aFormat, uint8_t aColor, uint8_t aBackground);

//--------------------------------------
void VIEW_ShowOutSinus(void)
{
char tmp[50];

uint8_t pushed;
sFONT * font;


	 if(VIEW_DisplayOff==3) return;

	  OLED_Clear(OLED_BACKGROUND);


	  if((VIEW_DisplayOff&0x7F)==3)
	  {
		  VIEW_DisplayOff&=0x7F;
		  goto update;
	  }
	  VIEW_DisplayOff&=0x7F;

	  if(VIEW_DisplayOff==1)
	  {
		  sprintf(tmp,"Detector 1    %c%d",TxtProgress[(Timer_1ms/100)&0x03],Timer_1ms/1000);
		  GUI_DisString_EN(10,1, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);

		  if((VIEW_Det&0x7F)==1)
		        VIEW_ShowLockin("1",1,40,&LCK_AcuData2[0],&Font24,"%s~%d.%03d1=%d.%03d",0xFF,0x01);
		  else	VIEW_ShowLockin("1",1,40,&LCK_AcuData[0], &Font24,"%s~%d.%03d1=%d.%03d",0xFF,0xFF);


		  OLED_Display();
		  return;

	  }

	  if(VIEW_DisplayOff==2)
	  {
		  sprintf(tmp,"Detector 1    %c%d",TxtProgress[(Timer_1ms/100)&0x03],Timer_1ms/1000);
		  GUI_DisString_EN(10,1, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);

		  if((VIEW_Det&0x7F)==1)
			  	VIEW_ShowLockin("1",1, 20,&LCK_AcuData2[0],&Font24,"%s~%d.%03d1=%d.%03d",0xFF,0x01);
		  else  VIEW_ShowLockin("1",1, 20,&LCK_AcuData [0],&Font24,"%s~%d.%03d1=%d.%03d",0xFF,0xFF);

		  if((VIEW_Det&0x7F)==1)
			  	VIEW_ShowLockin("2",1, 80,&LCK_AcuData2[1],&Font24,"%s~%d.%03d2=%d.%03d",0xFF,0x01);
		  else  VIEW_ShowLockin("2",1, 80,&LCK_AcuData[1], &Font24,"%s~%d.%03d2=%d.%03d",0xFF,0xFF);

		  OLED_Display();
		  return;

	  }

	  sprintf(tmp,"L1 =%u.%02u  ~%d.%02u",(33*SIN_Offset01p[0])/10000,((33*SIN_Offset01p[0])/100)%100,(33*SIN_Sinus01pp[0])/10000,((33*SIN_Sinus01pp[0])/100)%100);
	  GUI_DisString_EN(5,5, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);

	  sprintf(tmp,"L2 =%u.%02u  ~%d.%02u",(33*SIN_Offset01p[1])/10000,((33*SIN_Offset01p[1])/100)%100,(33*SIN_Sinus01pp[1])/10000,((33*SIN_Sinus01pp[1])/100)%100);
	  GUI_DisString_EN(5,15, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);

//	  sprintf(tmp,"P1%c=%03d  P2%c=%03d  *%c%d",PotError[0]?'!':' ',VIEW_Amplifier[0]&0x7F,PotError[1]?'!':' ',VIEW_Amplifier[1]&0x7F,TxtProgress[(Timer_1ms/100)&0x03],Timer_1ms/1000);
//	  GUI_DisString_EN(5,26, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);
	  sprintf(tmp,"P1%c=%03d           *%c%d",PotError[0]?'!':' ',VIEW_Amplifier[0]&0x7F,TxtProgress[(Timer_1ms/100)&0x03],Timer_1ms/1000);
	  GUI_DisString_EN(5,25, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);
	  sprintf(tmp,"P2%c=%03d      %s",PotError[1]?'!':' ',VIEW_Amplifier[1]&0x7F,(SIN_Run && (VIEW_OffMode&0x7F))?"R U N":"STOPPED !");
	  GUI_DisString_EN(5,35, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);

#define BCOLOR 0xF
      pushed = (CT_ButtonState[CT_BUTTON_FUN2]&0x0FF);

      switch(VIEW_LEd12Switch&0x7F)
      {
      case 1:
		  GUI_DisString_EN(5,5,"L1",&Font8,1,BCOLOR);//pushed?1:BCOLOR,pushed?BCOLOR:1);
		  break;
      case 2:
		  GUI_DisString_EN(5,15,"L2",&Font8,1,BCOLOR);//pushed?1:BCOLOR,pushed?BCOLOR:1);
		  break;
      case 3:
		  GUI_DisString_EN(5,25,"P1",&Font8,1,BCOLOR);//pushed?1:BCOLOR,pushed?BCOLOR:1);
		  break;
      case 4:
		  GUI_DisString_EN(5,35,"P2",&Font8,1,BCOLOR);//1:BCOLOR);
		  break;
      }


	  if(!DAT_UsbLock)
		   sprintf(tmp,"BL%d",DAT_InBytes);
	  else strcpy(tmp,"USB");
      GUI_DisString_EN(99,10, (const char*)tmp, &Font8, FONT_BACKGROUND, WHITE);

      uint8_t viewdet=VIEW_Det&0x7F;

      if(viewdet<2)
      {
    	   // jeden detektor
    	  if(!viewdet)
    	  {
    		  GUI_DisString_EN(43,50, "DET 1", &Font12, FONT_BACKGROUND, WHITE);

    		  // DET nr 1
    		  if(VIEW_OffMode&0x01)
    			  VIEW_ShowLockin("1",10,62,&LCK_AcuData[0],&Font12,NULL,0xFF,0xFF);
    		  if(VIEW_OffMode&0x02)
    			  VIEW_ShowLockin("2",10,74,&LCK_AcuData[1],&Font12,NULL,0xFF,0xFF);
    	  }
    	  else
    	  {
    		  GUI_DisString_EN(43,50, "DET 2", &Font12, 0x01, WHITE);
    		  //DET nr 2
    		  if(VIEW_OffMode&0x01)
    			  VIEW_ShowLockin("1",10,62,&LCK_AcuData2[0],&Font12,NULL,0xFF,0x01);
    		  if(VIEW_OffMode&0x02)
    			  VIEW_ShowLockin("2",10,74,&LCK_AcuData2[1],&Font12,NULL,0xFF,0x01);
    	  }
      }
      else
      {
   	   // dwa detektory
		  if(VIEW_OffMode&0x01)
			  VIEW_ShowLockin("1",10,50,&LCK_AcuData[0],&Font12,NULL,0xFF,0xFF);
		  if(VIEW_OffMode&0x02)
			  VIEW_ShowLockin("2",10,62,&LCK_AcuData[1],&Font12,NULL,0xFF,0xFF);

		  if(VIEW_OffMode&0x01)
			  VIEW_ShowLockin("1",10,78,&LCK_AcuData2[0],&Font12,NULL,0xFF,1);
		  if(VIEW_OffMode&0x02)
			  VIEW_ShowLockin("2",10,90,&LCK_AcuData2[1],&Font12,NULL,0xFF,1);
      }



	  GUI_DisString_EN(1,114,"LCHG",&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);
//	  GUI_DisString_EN(0,114,VIEW_OutSinMode&0x01?" Sin ":" Lck ",&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);

	 // GUI_DrawRectangle(0,60,12,90,	WHITE,	DRAW_FULL,	DOT_PIXEL_1X1);
	  if(VIEW_Save)
	  {
		  pushed = (CT_ButtonState[CT_BUTTON_FUN1]&0x0FF);
		  GUI_DisString_EN(1,54,"S",&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);
		  GUI_DisString_EN(1,66,"a",&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);
		  GUI_DisString_EN(1,78,"v",&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);
	  }
	  pushed = (CT_ButtonState[CT_BUTTON_FUN3]&0x0FF);
	  switch(VIEW_OffMode&0x7F)
	  {
	  	  case 0: strcpy(tmp," LED- ");break;
	  	  case 1: strcpy(tmp," LED1 " );break;
	  	  case 2: strcpy(tmp," LED2" );break;
	  	  case 3: strcpy(tmp," L1+2 " );break;
	  	  default:strcpy(tmp," ??  ");break;
	  }
	  GUI_DisString_EN(35,114,tmp,&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);

	  pushed = (CT_ButtonState[CT_BUTTON_FUN4]&0x0FF);
	  switch(viewdet)
	  {
	  	  case 0: strcpy(tmp," DET1 "); break;
	  	  case 1: strcpy(tmp," DET2 "); break;
	  	  case 2: strcpy(tmp," D1+2 "); break;
	  	  default:strcpy(tmp,"  ?? "); break;
	  }
	  GUI_DisString_EN(82,114,tmp,&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);

	  /*
	  pushed = (CT_ButtonState[CT_BUTTON_FUN4]&0x0FF);
	  lcknow[0]=VIEW_SlowF&0x7F;
	  if(lcknow[0])
		   sprintf(tmp,"F%dk%d",lcknow[0]/10,lcknow[0]%10);
	  else strcpy(tmp,"F1Hz");
      GUI_DisString_EN(90,114,tmp,&Font12,pushed?BLACK:BCOLOR,pushed?BCOLOR:BLACK);
*/
	  //pushed = (CT_ButtonState[CT_BUTTON_FUN5]&0x0FF);
	  //GUI_DisString_EN(108,114," F5 ",&Font12,pushed?BLACK:WHITE,pushed?WHITE:BLACK);
      update:

	  OLED_Display();





}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
    	  // 6,59
    	  //6,71,
void VIEW_ShowLockin(char * aText, uint8_t aX, uint8_t aY, LCK_LOCKINDATA * aLin, sFONT * aFont, char * aFormat, uint8_t aForeColor, uint8_t aBackColor)
{
uint16_t lcknow,lckav,lck;
uint32_t vol[2];
char  tmp[70];

		if(aForeColor==0xFF) aForeColor=WHITE;
		if(aBackColor==0xFF) aBackColor=FONT_BACKGROUND;

		if(!aFormat) aFormat = "%s~%01d.%03dV =%01d.%03dV";

		lcknow=(aLin->ValueAmp2>>4);
		lckav=(aLin->AvVal2>>4);

	    vol[0]=lcknow*3300L*2;
	    vol[0]>>=12;

	    vol[1]=lckav*3300L*2;
	    vol[1]>>=12;

	   // if(vol[0]>3300) vol[0]=3300;
	   // if(vol[1]>3300) vol[1]=3300;

	    sprintf(tmp,aFormat, aText, vol[0]/1000,vol[0]%1000,vol[1]/1000,vol[1]%1000);
	    GUI_DisString_EN(aX,aY, (const char*)tmp, aFont, aBackColor, aForeColor);
}
