/*
 * Parser.c
 *
 *  Created on: 15.01.2019
 *      Author: Adam
 */

#include "stdint.h"
#include "stm32f4xx.h"
#include <string.h>

#include "main.h"

#include "VIEW_Code.h"
#include "HW_Hardware.h"
#include "DAT_Buffer.h"
#include "SIN_Sinus.h"


uint8_t PAR_State;
#define PAR_INBUFFER_SIZE 250
uint8_t PAR_InBuffer	[PAR_INBUFFER_SIZE+2];
uint8_t	PAR_InBufferIdx;


#define PAR_OUTBUFFER_SIZE 250
uint8_t PAR_OutBuffer	[PAR_OUTBUFFER_SIZE+2];
uint8_t	PAR_OutBufferIdx;

/*
 * [0] - 'X', w odpowiedzi 'x'
 * [1] - index od 0x40 : 64 znaki
 * [2] - cmd (duze litery, w odpowiedzi male)
 * [3] - dlugosc samych danych + ' ' (max 94)
 * [4] - dane
 * ...
 * [5] - crc na razie '.'
 * [6] - 0x0A lub 0x0D
 */
enum PAR_FRAME {PAR_FRAME_BEG=0, PAR_FRAME_IDX, PAR_FRAME_CMD, PAR_FRAME_LEN, PAR_FRAME_DATA};

#define PAR_FRAME_MAXDATALEN 240

uint8_t  PAR_DataIn			(char aChar);
uint16_t PAR_FormatOutput	(uint8_t aLen, uint8_t aCmd, uint8_t aIndex, char * aBuffer);
uint16_t PAR_ParseCommand	(char * aBuffer, char aCmd, uint8_t aIndex);

uint8_t PAR_ReadParams	(char * aParams);
uint8_t PAR_PutParam	(char ** aParams);
uint8_t PAR_WriteParams	(char * aParams);
uint8_t PAR_GetParam	(char ** aParams);

uint16_t PAR_FormatOutput (uint8_t aLen, uint8_t aCmd, uint8_t aIndex, char * aBuffer);
int8_t   PAR_GetDecimal   (uint32_t aMin, uint32_t aMax, uint32_t * aVal, char * aString);


uint8_t PAR_LastIndex;
//-----------------------------------------------------------------
void PAR_DataReceived(char aZnak)
{
	if(!PAR_DataIn(aZnak)) return;
	if(PAR_ParseCommand((char*)PAR_InBuffer+PAR_FRAME_DATA, PAR_InBuffer[PAR_FRAME_CMD], PAR_InBuffer[PAR_FRAME_IDX]))
	   PAR_OutData((char*)PAR_OutBuffer);
	PAR_OutBuffer[0]=0;
	PAR_State=0;
}
//-----------------------------------------------------------------

uint8_t PAR_DataIn(char aChar)
{
uint8_t len;
	 PAR_InBuffer[PAR_State]=aChar;
	 if(!PAR_State)
	 {
		 if(aChar!='X') return 0;
		 PAR_State++;
		 return 0;
	 }

     switch(aChar)
     {
     	 case 0x0D:
     	 case 0x0A:
     		 	     if(PAR_State<4) goto resetstate;
     		 	     len = PAR_InBuffer[PAR_FRAME_LEN];
     		 	     len-=0x20;
     		 	     if(len>PAR_FRAME_MAXDATALEN) goto resetstate;
     		 	     if(len && (PAR_State!=(len+PAR_FRAME_DATA+1))) goto resetstate;
     		 	     PAR_State--;
     		 	     PAR_InBuffer[PAR_State]=0;
     		 	     return PAR_State;
     	 default:    if(aChar<' ') goto resetstate;
     	 	 	 	 break;
     }
     if(PAR_State >= (PAR_INBUFFER_SIZE-1))
     {
    	  resetstate:
		  PAR_State = PAR_FRAME_BEG;
		  return 0;
     }
     PAR_State++;
     return 0;
}
//-----------------------------------------------------------------
void PAR_FormatName(char * aDestPtr)
{

   sprintf(aDestPtr,"PLEFIND.H%02d.S%03d",HW_VERSION,SIN_VERSION);
}
//-----------------------------------------------------------------
uint16_t PAR_ParseCommand(char * aBuffer, char aCmd, uint8_t aIndex)
{
char tmp[20];
	 AutoPowerOffTimer_100ms = 0;
	 switch(aCmd)
	 {
 	 	 case '?': // echo
 	 		 	 	PAR_FormatName(tmp);
 		 	        return PAR_FormatOutput(0, aCmd, aIndex, tmp);
	 	 case 'E': // echo
	 		 	 	if(!aBuffer) break;
	 		 	    return PAR_FormatOutput(0, aCmd, aIndex, aBuffer);
	 	 case 'O':
	 		 	 	if(!aBuffer) break;
	 		 	 	VIEW_DisplayOff=((aBuffer[0]-'0')&0x03)|0x80;
	 		 	 	PAR_FormatOutput(0, aCmd, aIndex, "OK");
	 		 	 	return 1;
	 	 case 'L': // STOP/L1/L2/L1+2
	 		 	 	if(!aBuffer) break;
	 		 	    VIEW_OffMode=0x80+((aBuffer[0]-'0')&0x03);
					VIEW_Params|=0x80;
					VIEW_Save=1;
					return PAR_FormatOutput(0, aCmd, aIndex, NULL);
	 	  case 'P': // zapisz parametry
	 		  	  	if(!aBuffer) break;
	 		 	 	PAR_WriteParams(aBuffer);
	 		  	    return PAR_FormatOutput(strlen((char*)PAR_OutBuffer), aCmd, aIndex, NULL);

	 	  case 'G': // odczyt parametrow
	 		  	  	if(!aBuffer) break;
	 		  	    PAR_ReadParams(aBuffer);
	 		  	    return PAR_FormatOutput(strlen((char*)PAR_OutBuffer+4), aCmd, aIndex, NULL);
	 	  case 'S': // store memory
	 		  	  	  HW_StoreConfig();
	 		 		  VIEW_Noice=0x80;
	 		 		  VIEW_Save=0;
	 		 		  return 1;

	 	  case 'X':  CT_RunStop(1);
	 	  	  	  	  return PAR_FormatOutput(0, aCmd, aIndex, "OK");

	 	  case 'Z':   CT_RunStop(0);
	 	  	  	  	  return PAR_FormatOutput(0, aCmd, aIndex, "OK");

	 }
	 return 0;
}
//-----------------------------------------------------------------
void PAR_TrigCommand(char * aBuffer, char aCmd)
{
	if(PAR_ParseCommand(aBuffer,aCmd,255))
		PAR_OutData((char*)PAR_OutBuffer);
}
//-----------------------------------------------------------------
void PAR_OutData(char * aBuffer)
{
		if(DAT_UsbLock)
			 DAT_OutSendText(aBuffer);
		else DAT_OutSendText2(aBuffer);
}
//------------------------------------------------------------------

//-----------------------------------------------------------------
uint8_t PAR_ReadParams(char * aParams)
{
uint8_t cnt=0;
uint8_t retval;

	PAR_OutBufferIdx=PAR_FRAME_DATA;
    while(1)
    {
    	retval=PAR_PutParam(&aParams);
    	if(retval<=0) return cnt;
    	cnt++;
    	if(!*aParams) break;
    	if(*aParams!=',') return cnt;
    	aParams++;
    	if(PAR_OutBufferIdx>(PAR_FRAME_MAXDATALEN-10)) return cnt;
    }
    PAR_OutBuffer[--PAR_OutBufferIdx]=0;
    return cnt;
}
//-----------------------------------------------------------------
uint8_t PAR_PutParam(char ** aParams)
{
uint32_t val;
	char * param = *aParams;
	if(!memcmp(param,"S1OF",4)) val=VIEW_Offset01p[0];
	else
	if(!memcmp(param,"S1MO",4)) val=VIEW_Sinus01pp[0];
	else
	if(!memcmp(param,"S2OF",4)) val=VIEW_Offset01p[1];
	else
	if(!memcmp(param,"S2MO",4)) val=VIEW_Sinus01pp[1];
	else
	if(!memcmp(param,"AMP1",4)) val=VIEW_Amplifier[0]&0x7F;
	else
	if(!memcmp(param,"AMP2",4)) val=VIEW_Amplifier[1]&0x7F;
	else
	if(!memcmp(param,"FILT",4)) val=VIEW_AvMulti&0x7F;
	else
	if(!memcmp(param,"FREQ",4)) val=VIEW_SlowF&0x7F;
	else
	if(!memcmp(param,"SZUM",4)) val=VIEW_Noice&0x7F;
	else
	if(!memcmp(param,"MODE",4)) val=SIN_Mode;
	else
	if(!memcmp(param,"STAT",4)) val=VIEW_OffMode+(10*VIEW_Save);
	else return 0;
	memcpy(PAR_OutBuffer+PAR_OutBufferIdx,param,4);
	PAR_OutBufferIdx+=4;
	sprintf((char*)PAR_OutBuffer+PAR_OutBufferIdx,"=%u,",val);
	PAR_OutBufferIdx+=strlen((char*)PAR_OutBuffer+PAR_OutBufferIdx);
	param+=4;
	*aParams=param;
	return 1;
}
//-----------------------------------------------------------------
uint8_t PAR_WriteParams(char * aParams)
{
uint8_t cnt=0;
uint8_t retval;
	PAR_OutBufferIdx=PAR_FRAME_DATA;
    while(1)
    {
    	retval=PAR_GetParam(&aParams);
    	if(retval<=0) return cnt;
    	cnt++;
    	if(!*aParams) break;
    	if(*aParams!=',') return cnt;
    	aParams++;
    }
    PAR_OutBuffer[PAR_OutBufferIdx-1]=0;
    return cnt;
}
//-----------------------------------------------------------------
uint8_t PAR_GetParam(char ** aParams)
{
uint32_t val;
uint8_t ret;
	char * param = *aParams;
    memcpy(PAR_OutBuffer+PAR_OutBufferIdx,param,4);
	PAR_OutBufferIdx+=4;
	PAR_OutBuffer[PAR_OutBufferIdx]='=';
	PAR_OutBuffer[PAR_OutBufferIdx+1]=0;
	PAR_OutBufferIdx++;

	if(!memcmp(param,"S1OF=",5))
	{
		  ret=PAR_GetDecimal(0,1000,&val,param+5);
		  if(ret>0)
		  {
			  //if((val+(VIEW_Sinus01pp[0]/2)) > 1000) return 0;
			  //if((val-(VIEW_Sinus01pp[0]/2)) <0) return 0;
			  VIEW_Offset01p[0] = val;
			  VIEW_Params|=0x80;
			  VIEW_Save=1;
			  *aParams+=ret+5;
		  }
		  return ret;
	}
	if(!memcmp(param,"S1MO=",5))
	{
		  ret=PAR_GetDecimal(0,1000,&val,param+5);
		  if(ret>0)
		  {
			  //if((VIEW_Offset01p[0]-(val/2)) <0) return 0;
			  //if((VIEW_Offset01p[0]+(val/2)) > 1000) return 0;
			  VIEW_Sinus01pp[0] = val;
			  VIEW_Params|=0x80;
			  VIEW_Save=1;
			  *aParams+=ret+5;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"S2OF=",5))
	{
		  ret=PAR_GetDecimal(0,1000,&val,param+5);
		  if(ret>0)
		  {
			  //if((val+(VIEW_Sinus01pp[1]/2)) > 1000) return 0;
			  //if((val-(VIEW_Sinus01pp[1]/2)) <0) return 0;
			  VIEW_Offset01p[1] = val;
			  VIEW_Params|=0x80;
			  VIEW_Save=1;
			  *aParams+=ret+5;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"S2MO=",5))
	{
		  ret=PAR_GetDecimal(0,1000,&val,param+5);
		  if(ret>0)
		  {
			  //if((VIEW_Offset01p[1]-(val/2)) <0) return 0;
			  //if((VIEW_Offset01p[1]+(val/2)) > 1000) return 0;
			  VIEW_Sinus01pp[1] = val;
			  VIEW_Params|=0x80;
			  VIEW_Save=1;
			  *aParams+=ret+5;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"AMP1=",5))
	{
		  ret=PAR_GetDecimal(0,127,&val,param+5);
		  if(ret>0)
		  {
				VIEW_Amplifier[0]=val|0x80;
				VIEW_Save=1;
				*aParams+=ret+5;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"AMP1+=",6))
	{
		  ret=PAR_GetDecimal(0,99,&val,param+6);
		  if(ret>0)
		  {
			  	val+=VIEW_Amplifier[0]&0x7F;
			  	if(val>127) val=127;
				VIEW_Amplifier[0]=val|0x80;
				VIEW_Save=1;
				*aParams+=ret+6;
		  }
		  return ret+6;
	}
	if(!memcmp(param,"AMP1-=",6))
	{
		  ret=PAR_GetDecimal(0,99,&val,param+6);
		  if(ret>0)
		  {
			  	val=(VIEW_Amplifier[0]&0x7F)-val;
			  	if(val>127) val=0;
				VIEW_Amplifier[0]=val|0x80;
				VIEW_Save=1;
				*aParams+=ret+6;
		  }
		  return ret+6;
	}
	if(!memcmp(param,"AMP2=",5))
	{
		  ret=PAR_GetDecimal(0,127,&val,param+5);
		  if(ret>0)
		  {
				VIEW_Amplifier[1]=val|0x80;
				VIEW_Save=1;
				*aParams+=ret+5;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"AMP2+=",6))
	{
		  ret=PAR_GetDecimal(0,99,&val,param+6);
		  if(ret>0)
		  {
			  	val+=VIEW_Amplifier[1]&0x7F;
			  	if(val>127) val=127;
				VIEW_Amplifier[1]=val|0x80;
				VIEW_Save=1;
				*aParams+=ret+6;
		  }
		  return ret+6;
	}
	if(!memcmp(param,"AMP2-=",6))
	{
		  ret=PAR_GetDecimal(0,99,&val,param+6);
		  if(ret>0)
		  {
			  	val=(VIEW_Amplifier[1]&0x7F)-val;
			  	if(val>127) val=0;
				VIEW_Amplifier[1]=val|0x80;
				VIEW_Save=1;
				*aParams+=ret+6;
		  }
		  return ret+6;
	}
	if(!memcmp(param,"FREQ=",5))
	{
		  ret=PAR_GetDecimal(0,19,&val,param+5);
		  if(ret>0)
		  {
			  VIEW_SlowF=val|0x80;
			  VIEW_Save=1;
			  VIEW_Params|=0x80;
			  *aParams+=ret+5;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"FILT=",5))
	{
		  ret=PAR_GetDecimal(2,127,&val,param+5);
		  if(ret>0)
		  {
			  VIEW_AvMulti=val|0x80;
			  VIEW_Save=1;
			  VIEW_Params|=0x80;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"MODE=",5))
	{
		  ret=PAR_GetDecimal(0,33,&val,param+5);
		  if(ret>0)
		  {
			  VIEW_OffMode=((val%10)&0x03)|0x80;
			  VIEW_Det=((((val/10)-1))&0x03)|0x80;
			  VIEW_Save=1;
			  VIEW_Params|=0x80;
		  }
		  return ret+5;
	}
	if(!memcmp(param,"SZUM=",5))
	{
		  ret=PAR_GetDecimal(0,99,&val,param+5);
		  if(ret>0)
		  {
			  VIEW_Noice=val|0x80;
			  VIEW_Save=1;
			  VIEW_Params|=0x80;
		  }
		  return ret+5;
	}
	PAR_OutBufferIdx-=5;
	strcat((char*)PAR_OutBuffer+PAR_OutBufferIdx,"?");
	PAR_OutBufferIdx++;
	return 0;
}
//--------------------------------------------
int8_t PAR_GetDecimal(uint32_t aMin, uint32_t aMax, uint32_t * aVal, char * aString)
{
uint32_t val=0;
char * start;
char znak;
	if(!*aString) return -1;
	start=aString;
	while(1)
	{
		znak=*aString++;
		if(!znak || znak==',') break;
		if(znak<'0' || znak>'9') return -2;
		val*=10;
		val+=znak-'0';
	}
	if(val<aMin || val>aMax) return 0;
	*aVal=val;
	return aString-start-1;
}
/*
uint8_t PAR_GetParam(char ** aParams)
{
uint32_t val;
char znak;
	char * param = *aParam;
	val=0;
	for(uint8_t i=0;i<3;i++)
	{
		znak=*param++;
		if(znak==0) return -1;
		val<<=8;
		val|=znak;
	}

	if(param[4]!='=') return -1;
	char * next;
	next=strchr(param+4,',');
	if()

}
*/
//-----------------------------------------------------------------
uint16_t PAR_FormatOutput(uint8_t aLen, uint8_t aCmd, uint8_t aIndex, char * aBuffer)
{
    PAR_OutBuffer[PAR_FRAME_BEG]='x';
    if(aLen>PAR_FRAME_MAXDATALEN) aLen=0;
	if(aBuffer)
	{
        if(aLen==0) aLen=strlen(aBuffer);
        if(aLen>PAR_FRAME_MAXDATALEN) aLen=0;
		memcpy(PAR_OutBuffer+PAR_FRAME_DATA,aBuffer,aLen);
	}
	PAR_OutBuffer[PAR_FRAME_CMD]=aCmd+0x20;
	PAR_OutBuffer[PAR_FRAME_IDX]=0x40+(aIndex&0x3F);
	PAR_OutBuffer[PAR_FRAME_LEN]=0x20;//+aLen;
	PAR_OutBuffer[PAR_FRAME_DATA+aLen]='.'; // crc
	PAR_OutBuffer[PAR_FRAME_DATA+aLen+1]=0x0D; // 0x0A
	PAR_OutBuffer[PAR_FRAME_DATA+aLen+2]=0x0A; // 0x0D
	PAR_OutBuffer[PAR_FRAME_DATA+aLen+3]=0;
	return aLen+PAR_FRAME_DATA+3;
}
//-----------------------------------------------------------------
//-----------------------------------------------------------------
