/*
 * SIN_sinus.c
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */




#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "string.h"

#include "CT_Control.h"
#include "SIN_Sinus.h"
#include "VIEW_Code.h"
#include "HW_Hardware.h"

uint8_t SIN_SquaredSinus;

//--------------------------------------------

#define SIN_SWAP_SAMPLES_DELAY 50 // ilosc pustych sampli przy przelaczaniu diod  w jednostkach pomiaru, czyli 10uS * 50 = 0.5ms
uint8_t SIN_EmptySamples;

uint8_t SIN_Mode; // tryb pracy hex (LSN - led, MSNibble - det)
/***
 * SIN_SinusTable
 * tablica sinusa w 32 bitiwych wartosciach 0-0xFFFFFFFF
 */

static const uint32_t SIN_SinusTable[SIN_SINUS_TAB_SIZE] =
{
2147483648,
2252855675,
2357973852,
2462584941,
2566436923,
2669279609,
2770865244,
2870949098,
2969290059,
3065651218,
3159800430,
3251510883,
3340561637,
3426738161,
3509832850,
3589645520,
3665983896,
3738664071,
3807510954,
3872358686,
3933051042,
3989441810,
4041395140,
4088785871,
4131499834,
4169434129,
4202497369,
4230609900,
4253703997,
4271724026,
4284626573,
4292380555,
4294967294,
4292380555,
4284626573,
4271724026,
4253703997,
4230609900,
4202497369,
4169434129,
4131499834,
4088785871,
4041395140,
3989441810,
3933051042,
3872358686,
3807510954,
3738664071,
3665983896,
3589645520,
3509832850,
3426738161,
3340561637,
3251510883,
3159800430,
3065651218,
2969290059,
2870949098,
2770865244,
2669279609,
2566436923,
2462584941,
2357973852,
2252855675,
2147483647,
2042111618,
1936993441,
1832382352,
1728530370,
1625687684,
1524102049,
1424018195,
1325677234,
1229316075,
1135166863,
1043456410,
954405656,
868229132,
785134443,
705321773,
628983397,
556303222,
487456339,
422608607,
361916251,
305525483,
253572153,
206181422,
163467459,
125533164,
92469924,
64357393,
41263296,
23243267,
10340720,
2586738,
0,
2586738,
10340720,
23243267,
41263296,
64357393,
92469924,
125533164,
163467459,
206181422,
253572153,
305525483,
361916251,
422608607,
487456339,
556303222,
628983397,
705321773,
785134443,
868229132,
954405656,
1043456410,
1135166863,
1229316075,
1325677234,
1424018195,
1524102049,
1625687684,
1728530370,
1832382352,
1936993441,
2042111618
};

//----------------------------------------------

#define LCK_PERIODS 		64L // ilosc okresow liczenia lockina i pamieci probek jednoczesnie
#define LCK_TOTALSAMPLES 	(LCK_PERIODS * SIN_SINUS_TAB_SIZE) // ilosc probek w pamieci z ktorych jest liczony lockin

//int32_t LCK_InBuffer	[LCK_TOTALSAMPLES]; // bufor wyniku mnozenia przez ref sinus (wartosc 32 bit ze znakiem)
//int32_t LCK_InBuffer2	[LCK_TOTALSAMPLES]; // bufor wyniku mnozenia przez ref cosinus (wartosc 32 bit ze znakiem)

uint32_t LCK_PutIdx; // numer probki trafiajacej do bufora

/*
int32_t  LCK_ValueSin,      LCK_ValueCos;		// obliczony lockin w 16 bitach ze znakiem: z sinusa i cosinusa
uint32_t LCK_ValueSin_Snap, LCK_ValueCos_Snap;  // lockin w 16 bitach ze znakiem
uint32_t LCK_ValueAmp1,LCK_ValueAmp2;
uint16_t LCK_AvVal2;
*/

uint16_t SIN_AddNoise(uint16_t aValue);

uint32_t SIN_NoiceLevel;

int64_t LCK_SumSin,LCK_SumCos; // sumu z usredniania LCK_PERIODS okresów
int64_t LCK_SumSin2,LCK_SumCos2; // sumu z usredniania LCK_PERIODS okresów

//#define PHASE_OFFSET_DEG 0

//uint16_t SIN_PhaseOffset = PHASE_OFFSET_DEG ;
uint16_t SIN_PhaseProbeoffset; // wyliczona wartosc offsetu probki

#define LCK_SKIP_PERDIOS 2

uint16_t SIN_ShowOutVal;
uint8_t  SIN_SwapState,SIN_SwapIt;
//----------------------------------------------
uint16_t SIN_SinusDacTable  [2][SIN_SINUS_TAB_SIZE];	      //wygeneowana tablica sinusa wyjsciowego do wrzucenia na DAC, zaleznie od ustawionych parametrow offsetu i modulacji
//----------------------------------------------

void LCK_Acumulate		(uint32_t aLockin, LCK_LOCKINDATA * aAkuData);
void LCK_ClearAcumulate	(void);

uint8_t LCK_Run_V1		(uint16_t aIndex, uint32_t aControlSinus, uint32_t aControlCosinus, int32_t aMeasuredSinus1, int32_t aMeasuredSinus2, uint8_t aTab);
void 	LCK_Init_V1		(void) ;

uint16_t randrand(void);
uint32_t SIN_sqrt32(uint32_t aValue);

void LCK_OutLockData(uint8_t aFullFrame);


//----------------------------------------------
/*
	aAmplitude01p - amplituda sinusoidy w 0.1 procenta ca³ego zakresu 0 -FFF (po³ówka sinusoidy!)
	aConsant01p   - sta³a sinusoidy w 0.1 procenta ca³ego zakresu 0 -3FF
			aAmplitude + aConstant nie moze byc wieksza niz 100%
			aAmplitude nie moze byc wieksze niz aConstant

	zwraca 0 jak paramtery nie przesterowane
*/

typedef struct SIN_SinusParamsStr
{
	uint16_t Offset01p; // offset w 0.1%
	uint16_t Sinus01pp; // modulacja pik2pik w 0.1%
	uint16_t HalfSinus;
} SIN_SinusParamsStr;


uint8_t SIN_GenerateDacTable(uint16_t aConstant01p, uint16_t aSinPikToPik01p, uint8_t aTab)
{
uint16_t halfSinus; // w jednostkach DAC
uint64_t val64;
uint16_t const16,val16;
uint8_t ret = 0;
#define MAXSIN 0xFFFFFFFFL

uint16_t * sinusdactable;
sinusdactable=&SIN_SinusDacTable[aTab];


	if(!aConstant01p || !aSinPikToPik01p) 	ret |= 0x01;
	if((aSinPikToPik01p>1000)  || (aConstant01p> 1000)) ret |= 0x02;
	if(((aSinPikToPik01p>>1) + aConstant01p)> 1000) ret |= 0x04;

	SIN_Offset01p[aTab] = aConstant01p;
	SIN_Sinus01pp[aTab] = aSinPikToPik01p;
	SIN_ShowOutVal = (aConstant01p/20) | ((aSinPikToPik01p/20)<<8);
	switch(VIEW_Det&0x7F)
	{
		case 0: SIN_Det=0x01;break;
		case 1: SIN_Det=0x02;break;
		case 2: SIN_Det=0x03;break;
	}

	const16 = (uint16_t)((aConstant01p * (uint32_t)0xFFFL) / 1000);
	halfSinus = (((uint32_t)0xFFFL) * aSinPikToPik01p) / 2000;
	for(int  i=0;i<SIN_SINUS_TAB_SIZE; i++)
	{
		val64 = SIN_SinusTable[i];
		if(SIN_SquaredSinus)
		{
			if(val64&0x80000000L) val64=0xFFFFFFFFL; else val64=0;
		}
		val64 = ((val64 * 0xFFF * aSinPikToPik01p) / MAXSIN)/ 1000;
		val16 = const16 + (uint16_t)val64;
		if(val16 < halfSinus) val16=0; else val16-=halfSinus;
		if(val16 > 0xFFF)
		{
			val16 = 0xFFF;
			ret |= 0x10;
		}
		sinusdactable[i] =val16;
	}
	SIN_PhaseProbeoffset = (((VIEW_PhaseOffset&0x1FF)%360) * SIN_SINUS_TAB_SIZE) / 360;
	//SIN_SwapIt = (VIEW_OffMode&0x7F)==2;
	SIN_SwapIt=VIEW_OffMode&0x7F;

	return ret;
}
//----------------------------------------------
uint8_t SIN_GenerateDacTable_Constant(uint16_t aConstant01p, uint8_t aTab)
{
	return SIN_GenerateDacTable(aConstant01p, (1000-aConstant01p)>>1, aTab);
}
//----------------------------------------------
uint8_t SIN_GenerateDacTable_SinPP(uint16_t aPikToPik01p, uint8_t aTab)
{
	aPikToPik01p>>=1;
	return SIN_GenerateDacTable(1000-aPikToPik01p, aPikToPik01p, aTab);
}
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------



// ---------------------------------------------------------------

// ---------------------------------------------------------------

#include "DAT_Buffer.h"

uint16_t  SIN_DacIndex;
uint16_t  SIN_DividerSet, SIN_Divider; // 1 /2 /4

uint8_t   SIN_Run;

uint32_t  SIN_TotalSamples;
uint8_t   SIN_State;
uint32_t  SIN_OutVoltage, SIN_OutVoltage2; // aktualna wartosc sinusa i cosinusa wyjsciowego z tablicy
uint16_t  SIN_InVoltage,SIN_InVoltage2;
uint16_t  SIN_DacOut;

uint16_t SIN_Offset01p[2]; // offset w 0.1%
uint16_t SIN_Sinus01pp[2]; // modulacja pik2pik w 0.1%

uint8_t SIN_Det;
//uint16_t SIN_HalfSinus[2]; // wartosc amplitudy sinusa wyjsciowego w jednostkach DAC

/*
 * Format danych
 *  sinus: bajt 1 bit 7 poczatek ramki gdy ==1
 *  	   bajt 1 bit 6 poczatek sinusa gdy ==1
 *  	   bajt 1 bity [6:0 ] 6 mlodszych bitow danych
 *
 * 		   bajt 2 bit 7 . =0
 *  	   bajt 2 bit 6 = 0 - typ danych - SINUS
 *  	   bajt 2 bity [6:0 ] 6 starszych bitow danych
 *  	   *
 *  ext  : bajt 1 bit 7 poczatek ramki gdy ==1
 *  	   bajt 1 bit 6 gdy ==1 to wartosc lockina
 *  	   				gdy ==0 to parametry sinusa na 12 bitach (w jednostkach 2%, skladowa stala i skladowa zmienna)
 *  	   bajt 1 bity [5:0 ] 6 mlodszych bitow danych
 *
 * 		   bajt 2 bit 7 . =0
 *  	   bajt 2 bit 6 = 1(!) - typ danych EXT
 *  	   bajt 2 bity [6:0 ] 6 starszych bitow danych
 *  	    *  	   					jesli wysylana wartosc lockina to w dwoch najstaszych bitach zakodowany typ lockina
 *  	    							bity [5:4] == 00 to lockin2 na bitach[9:0] w 3.3mV
 *  	   									       == 01 to lockin1 na bitach[9:0] w 3.3mV
 *  	   									       == 10 to lockin2 na bitach[9:0] usrednione z 200ms
 *  	   									       == 11
 *
 *  	   *
 *
 *
 *  Komendy: format na 2 bajtach
 *  	bajt 1
 *
 */

uint8_t SIN_LedUsed;

void SIN_SampleStep(void) // timer co 10us
{
uint32_t newidx;
uint8_t  newled;
	if(!SIN_State) return;

	if(SIN_EmptySamples)
	{
		SIN_EmptySamples--;
		return;
	}

	LL_ADC_ClearFlag_EOCS(ADC1);
	LL_ADC_REG_StartConversionSWStart(ADC1);

#ifdef ADC2_ENABLED
	LL_ADC_ClearFlag_EOCS(ADC2);
	LL_ADC_REG_StartConversionSWStart(ADC2);
#endif

	if(++SIN_DacIndex >= SIN_SINUS_TAB_SIZE) 	SIN_DacIndex=0;

	SIN_OutVoltage  = SIN_SinusTable[SIN_DacIndex & 0x7F];
	SIN_OutVoltage2 = SIN_SinusTable[(SIN_DacIndex + 32) & 0x7F];

	newidx=(LCK_PutIdx+1)&(LCK_TOTALSAMPLES-1);
	if(!newidx && (SIN_SwapIt==3))
	{
		 newled = !SIN_LedUsed;
		 SIN_EmptySamples=SIN_SWAP_SAMPLES_DELAY;
	}
	else newled =  SIN_LedUsed;

    SIN_DacOut = SIN_SinusDacTable[newled][SIN_DacIndex];

	while(!LL_ADC_IsActiveFlag_EOCS(ADC1));
	SIN_InVoltage = LL_ADC_REG_ReadConversionData12(ADC1);
#ifdef ADC2_ENABLED

	while(!LL_ADC_IsActiveFlag_EOCS(ADC2));
	SIN_InVoltage2 = LL_ADC_REG_ReadConversionData12(ADC2);
#endif

	LCK_Run_V1(LCK_PutIdx,(int32_t)SIN_OutVoltage, (int32_t)SIN_OutVoltage2, (int32_t)SIN_InVoltage, (int32_t)SIN_InVoltage2, SIN_LedUsed);

	LCK_PutIdx  = newidx;
	SIN_LedUsed = newled;
	if(!newidx)
	{
		if(SIN_LedUsed)
		{
			LL_DAC1_OUT(0);
			//LL_GPIO_ResetOutputPin (OUT_LED2_GPIO_Port,OUT_LED2_Pin); //!!
			//LL_GPIO_SetOutputPin   (OUT_LED1_GPIO_Port,OUT_LED1_Pin); //!!
		}
		else
		{
			LL_DAC2_OUT(0);
			//LL_GPIO_SetOutputPin     (OUT_LED2_GPIO_Port,OUT_LED2_Pin); //!!
			//LL_GPIO_ResetOutputPin   (OUT_LED1_GPIO_Port,OUT_LED1_Pin); //!!
			SIN_EmptySamples=5;
		}
	}
	if(SIN_LedUsed)
		 LL_DAC2_OUT(SIN_DacOut);
	else LL_DAC1_OUT(SIN_DacOut);

}
// ---------------------------------------------------------------
void SIN_Start(uint32_t aSamples, uint8_t aDivider)
{
	SIN_SinusEnable();

	SIN_State = 0;
	DAT_OutBufferClear();
	LCK_Init_V1();

	SIN_OutVoltage = SIN_SinusTable[(0 + SIN_PhaseProbeoffset) & 0x7F];
	SIN_OutVoltage2 = SIN_SinusTable[(0 + SIN_PhaseProbeoffset + 32) & 0x7F];

	LCK_ClearAcumulate();
	LCK_AcuData[0].AcuSample = VIEW_AvMulti&0x7F;
	LCK_AcuData[1].AcuSample = VIEW_AvMulti&0x7F;
	LCK_AcuData2[0].AcuSample = VIEW_AvMulti&0x7F;
	LCK_AcuData2[1].AcuSample = VIEW_AvMulti&0x7F;

	if(SIN_SwapIt==2) SIN_LedUsed=1; else SIN_LedUsed=0;

	SIN_InVoltage = 0;

	if(SIN_LedUsed)
	{
		 //LL_GPIO_ResetOutputPin (OUT_LED2_GPIO_Port,OUT_LED2_Pin);//!!
		 //LL_GPIO_SetOutputPin   (OUT_LED1_GPIO_Port,OUT_LED1_Pin);//!!
		 LL_DAC2_OUT(SIN_SinusDacTable[1][0]);
		 LL_DAC1_OUT(0);
	}
	else
	{
		//LL_GPIO_SetOutputPin     (OUT_LED2_GPIO_Port,OUT_LED2_Pin);//!!
		//LL_GPIO_ResetOutputPin   (OUT_LED1_GPIO_Port,OUT_LED1_Pin);//!!
		LL_DAC1_OUT(SIN_SinusDacTable[0][0]);
		LL_DAC2_OUT(0);
	}

	SIN_DacIndex = 0;
//	SIN_TotalSamples = aSamples;
//	if(aDivider) SIN_DividerSet=aDivider-1; else SIN_DividerSet=0;
//	SIN_Divider = 0;
	HAL_Delay(2);
	SIN_EmptySamples = SIN_SWAP_SAMPLES_DELAY;
	SIN_State = 1;

}
// ---------------------------------------------------------------
void SIN_Stop(void)
{
	LL_DAC1_OUT(0);
	LL_DAC2_OUT(0);

	SIN_SinusDisable();

	SIN_State = 0;
    //LL_GPIO_ResetOutputPin(OUT_LED1_GPIO_Port,OUT_LED1_Pin);//!!
	//LL_GPIO_ResetOutputPin(OUT_LED2_GPIO_Port,OUT_LED2_Pin);//!!

}
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
/*
  (A+B*sin(x)) * (C+D*sin(x+d)) = A*C + A*D*sin(x+d) + B*C*sin(x) + B*D*sin(x)*sin(x+d) =
= A*C + A*D*sin(x+d) + B*C*sin(x) + 0.5*B*D*cos(x-x-d) - 0.5*B*D*cos(x+x+d) =
= A*C + A*D*sin(x+d) + B*C*sin(x) + 0.5*B*D*cos(-d) - 0.5*B*D*cos(2x+d) =

	po odcieciu skladowej stalej:

 = A*C + 0.5*B*D*cos(-d)

dla A=0 i faza = 0
 = 0.5*B*D

 dla B=1 = 0.5*D *** <- tu uzyte

 dla A=0.5 i B=1 = 0.5*(C+D)

 */

// ---------------------------------------------------------------------------------------------------------------------------




//----------------------------------------------
void LCK_Init_V1(void)
{
	//memset(LCK_InBuffer,0,sizeof(LCK_InBuffer));
	//memset(LCK_InBuffer2,0,sizeof(LCK_InBuffer2));
	LCK_SumSin  = LCK_SumCos  = 0;
	LCK_SumSin2 = LCK_SumCos2 = 0;
	LCK_PutIdx = 0;
}

//----------------------------------------------

uint8_t LCK_Run_V1(uint16_t aIndex, uint32_t aControlSinus, uint32_t aControlCosinus, int32_t aMeasuredSinus1, int32_t aMeasuredSinus2, uint8_t aTab) // wartosc usredniona ze 128 okresów
{
int32_t ival, mul1,val, mul2;
uint64_t tmp;


	aControlSinus >>= 16; // 16bitow: zakres 0 - 0xFFFF
	aControlCosinus >>= 16; // 16bitow: zakres 0 - 0xFFFF

		// ---------- * sinus ---------------
	ival = (aControlSinus - 0x8000); // 16bit pp

	mul1 = ival * (int32_t)aMeasuredSinus1;
	LCK_SumSin  += mul1;

#ifdef ADC2_ENABLED
	mul2 = ival * (int32_t)aMeasuredSinus2;
	LCK_SumSin2 += mul2;
#endif

		// -------- * cosinus ---------
	ival = (aControlCosinus - 0x8000); // 16bit pp

	mul1 = ival * (int32_t)aMeasuredSinus1;
	LCK_SumCos  += mul1;

#ifdef ADC2_ENABLED
	mul2 = ival * (int32_t)aMeasuredSinus2;
	LCK_SumCos2 += mul2;
#endif

	if(aIndex>=(LCK_TOTALSAMPLES-1))
	{
			if(SIN_Det&0x01)
			{
				LCK_LOCKINDATA * ld = &LCK_AcuData[aTab];

				WORD0(ld->ValueSin) = WORD1(LCK_SumSin);
				WORD1(ld->ValueSin) = WORD2(LCK_SumSin);

				WORD0(ld->ValueCos) = WORD1(LCK_SumCos);
				WORD1(ld->ValueCos) = WORD2(LCK_SumCos);  // 16bit x 12bit x

				if(ld->ValueSin<0) ld->ValueSin=-ld->ValueSin;
				if(ld->ValueCos<0) ld->ValueCos=-ld->ValueCos;

				ld->ValueSin>>=8; // 16 bitow bez znaku =
				ld->ValueCos>>=8; // 16 bitow bez znaku = 1/2 amplitudy

				if(ld->ValueSin_Snap==0xFFFFFFFF)
				{
					ld->ValueSin_Snap = (uint32_t)ld->ValueSin;
					ld->ValueCos_Snap = (uint32_t)ld->ValueCos;

				}
			}

			LCK_SumSin=0;
			LCK_SumCos=0;


#ifdef ADC2_ENABLED

			// detector 2

			if(SIN_Det&0x02)
			{

				LCK_LOCKINDATA * ld = &LCK_AcuData2[aTab];

				WORD0(ld->ValueSin) = WORD1(LCK_SumSin2);
				WORD1(ld->ValueSin) = WORD2(LCK_SumSin2);

				WORD0(ld->ValueCos) = WORD1(LCK_SumCos2);
				WORD1(ld->ValueCos) = WORD2(LCK_SumCos2);  // 16bit x 12bit x

				if(ld->ValueSin<0) ld->ValueSin=-ld->ValueSin;
				if(ld->ValueCos<0) ld->ValueCos=-ld->ValueCos;

				ld->ValueSin>>=8; // 16 bitow bez znaku =
				ld->ValueCos>>=8; // 16 bitow bez znaku = 1/2 amplitudy

				if(ld->ValueSin_Snap==0xFFFFFFFF)
				{
					ld->ValueSin_Snap = (uint32_t)ld->ValueSin;
					ld->ValueCos_Snap = (uint32_t)ld->ValueCos;
				}

				LCK_SumSin2=0;
				LCK_SumCos2=0;
			}
#endif
			return !aTab;
	}
	return aTab;
}
//--------------------------------------------------------------
uint16_t randrand(void)
{
static uint16_t randval=0x3543;
	randval<<=2;
    randval*=7;
    randval^=LL_TIM_GetCounterMode(TIM7);
    randval^=SIN_InVoltage&0xF;
    randval+=LL_TIM_GetCounterMode(TIM5);
    return randval;
}

//---------------------------------------------------------------
uint16_t SIN_AddNoise(uint16_t aValue)
{
int32_t rndc;
uint16_t rndval = randrand()&0xFFF;
	rndval= (rndval*(uint32_t)SIN_NoiceLevel)/0x1000;
	rndc = rndval-(SIN_NoiceLevel>>1);
	rndc+=aValue;
	if(rndc<0) rndc=0;
	if(rndc>0xFFF) rndc=0xFFF;
	return rndc;
}
//-----------------------------------------------------------------
typedef struct SIN_STATUS
{
	uint16_t  Header; // ==0xFFFF
	uint8_t   Offset;
	uint8_t   Amplitude;
	uint16_t  LockInValue100ms;
	uint16_t  LockInValue1s;
	uint16_t  LockInValue10s;
	uint8_t   Status;
	uint8_t   nu0;
	uint16_t  nu16[2];
} SIN_STATUS;
SIN_STATUS SIN_Status;
//----------------------------------------------
void SIN_InitStatus(uint8_t aOffset, uint8_t aAmplitude)
{
	memset(&SIN_Status,0,sizeof(SIN_Status));
	SIN_Status.Header = 0xFFFF;
	SIN_Status.Offset = aOffset;
	SIN_Status.Amplitude = aAmplitude;
}
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

uint16_t SIN_sqrt(uint32_t aLiczba)
{
uint32_t 	x,p=6 ;
uint32_t    blad = 1;

	if(aLiczba == 0) return 1;

	while(1)
	{
		x=p;
		p = (x+aLiczba/x)>>1;

		if(p>=x)
		{
			if((p-x) <= blad) break;
		}
		else
		{
			if((x-p) <= blad) break;
		}
	}

	return p;
}

//-----------------------------------------------------------------------
uint32_t SIN_sqrt32(uint32_t aValue)
{
uint32_t c = 0x8000;
uint32_t g = 0x8000;

	for(;;)
	{
		if(g*g > aValue) 	g ^= c;
		c >>= 1;
		if(c == 0)   return g;
		g |= c;
    }
}
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
LCK_LOCKINDATA LCK_AcuData [2];
LCK_LOCKINDATA LCK_AcuData2[2];
//-----------------------------------------------------------------------
void LCK_ClearAcumulate(void)
{
	memset(LCK_AcuData2,0,sizeof(LCK_AcuData2));
	memset(LCK_AcuData,0,sizeof(LCK_AcuData));
}
//-----------------------------------------------------------------------
uint8_t LCK_UpdateSums(LCK_LOCKINDATA * ld)
{
uint32_t vsin,vcos,val32;
//uint64_t val64;

	//LCK_LOCKINDATA * ld = &LCK_AcuData[aTab];


	if(ld->ValueSin_Snap != 0xFFFFFFFF)
	{
		vsin  = ld->ValueSin_Snap>>1;
		vcos =  ld->ValueCos_Snap>>1;

		ld->ValueSin_Snap = 0xFFFFFFFF;

		val32 = (vsin*vsin) + (vcos*vcos);

		val32 = SIN_sqrt32(val32);
		val32>>=1;

		ld->ValueAmp2 = val32<<3;
		ld->ValueAmp1 = vsin<<2;

		LCK_Acumulate(ld->ValueAmp2, ld);
		return 1;
	}
	return 0;
}
//-----------------------------------------------------------------------
void LCK_Acumulate( uint32_t aLockIn, LCK_LOCKINDATA * ld)
{
	//LCK_LOCKINDATA * ld = &LCK_AcuData[aTab];

	ld->AcumulateBuffer[ld->AkumulateIdx] = aLockIn;

	ld->AkumulateSum2 += aLockIn;
	ld->AkumulateSum2 -= ld->AcumulateBuffer[(uint8_t)(ld->AkumulateIdx-ld->AcuSample)];
	ld->AkumulateIdx++;
	ld->AcuSamplesUsed++;
	if(ld->AcuSamplesUsed > ld->AcuSample) ld->AcuSamplesUsed = ld->AcuSample;
	ld->AvVal2 = ld->AkumulateSum2 / ld->AcuSamplesUsed;
}

//-------------------------------------------------------------------------------


uint8_t HW_OutBuffer[256];
uint8_t HW_OutIdx;

extern uint8_t PotError[2];
void LCK_OutLockData(uint8_t aFullFrame)
{
char lck1[30]={"-1"};
char lck2[30]={"-1"};

int32_t v1,v2,v3,v4;

	v1 = ((LCK_AcuData[0].ValueAmp2>>4)*3300L*2)>>12;
	v2 = ((LCK_AcuData[1].ValueAmp2>>4)*3300L*2)>>12;

	v3 = ((LCK_AcuData2[0].ValueAmp2>>4)*3300L*2)>>12;
	v4 = ((LCK_AcuData2[1].ValueAmp2>>4)*3300L*2)>>12;


	if(!(SIN_SwapIt&0x01))	 {v1=0;v3=0;}//=-1;sprintf(lck1,"%d", v1 );
	if(!(SIN_SwapIt&0x02))	 {v2=0,v4=0;}//sprintf(lck2,"%d", v2 );

	if(!(SIN_Det&0x01)) {v1=0;v2=0;}
	if(!(SIN_Det&0x02)) {v3=0;v4=0;}

	uint8_t mode = SIN_SwapIt+(SIN_Det*10);

////	if(!LCK_AcuData[0].TotalIdx)
//		sprintf(params,",%d,%d,%d,%d",SIN_Offset01p[0]/10,SIN_Sinus01pp[0]/10,SIN_Offset01p[1]/10,SIN_Sinus01pp[1]/10);

	if(!(LCK_AcuData[0].TotalIdx&0xF) || aFullFrame)
		  sprintf(HW_OutBuffer,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\x0D\x0A",LCK_AcuData[0].TotalIdx++,mode,v1,v2,v3,v4,SIN_Offset01p[0]/10,SIN_Sinus01pp[0]/10,SIN_Offset01p[1]/10,SIN_Sinus01pp[1]/10,VIEW_Amplifier[0]&0x7F,VIEW_Amplifier[1]&0x7F);//,PotError[0],PotError[1]);
	else  sprintf(HW_OutBuffer,"%d,%d,%d,%d,%d,%d\x0D\x0A",LCK_AcuData[0].TotalIdx++,mode,v1,v2,v3,v4);//,PotError[0],PotError[1]);

	//LCK_OutData();

	//sprintf(HW_OutBuffer,"%d,2,3,4,5\x0D\x0A",HW_OutIdx++);
	//DAT_OutSendText2(HW_OutBuffer);
//	if(!(LCK_AcuData[0].TotalIdx&0x01)) return;
	DAT_OutSendText(HW_OutBuffer);
	DAT_OutSendText2(HW_OutBuffer);
	sprintf(HW_OutBuffer,"POT1=%d,POT2=%d\x0D\x0A",PotError[0],PotError[1]);
	DAT_OutSendText(HW_OutBuffer);
	//HW_OutSendText((uint8_t*)HW_OutBuffer,strlen(HW_OutBuffer));


}
void LCK_OutData(void)
{
	sprintf(HW_OutBuffer,"%d,2,3,4,5\x0D\x0A",HW_OutIdx++);
	//DAT_OutSendText(HW_OutBuffer);
	DAT_OutSendText2(HW_OutBuffer);

}
//---------------------------------------------
void LCK_OutInfoData(void)
{
	sprintf(HW_OutBuffer, "\x0D\x0APLEFIND %02d.%02d\x0D\x0A");
	DAT_OutSendText(HW_OutBuffer);
	DAT_OutSendText2(HW_OutBuffer);
}
//-------------------------------------------------------------
void SIN_SinusDisable(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  LL_DAC_Disable(DAC1,  LL_DAC_CHANNEL_1);
	  LL_DAC_Disable(DAC1,  LL_DAC_CHANNEL_2);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(SINUS_OUT1_GPIO_Port, SINUS_OUT1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SINUS_OUT2_GPIO_Port, SINUS_OUT2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : LED1_BLUE_Pin KEYPAD_CS_Pin BUZZER_OUT_Pin DATACMD_OLED_Pin
	                           OLED_CS_Pin */
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	  GPIO_InitStruct.Pin = SINUS_OUT1_Pin;
	  HAL_GPIO_Init(SINUS_OUT1_GPIO_Port, &GPIO_InitStruct);
	  GPIO_InitStruct.Pin = SINUS_OUT2_Pin;
	  HAL_GPIO_Init(SINUS_OUT2_GPIO_Port, &GPIO_InitStruct);

	  HAL_GPIO_WritePin(SINUS_OUT1_GPIO_Port, SINUS_OUT1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SINUS_OUT2_GPIO_Port, SINUS_OUT2_Pin, GPIO_PIN_RESET);
}
//-------------------------------------------------------------
void SIN_SinusEnable(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  LL_DAC_Enable(DAC1,  LL_DAC_CHANNEL_1);
	  LL_DAC_Enable(DAC1,  LL_DAC_CHANNEL_2);


	  /*Configure GPIO pins : LED1_BLUE_Pin KEYPAD_CS_Pin BUZZER_OUT_Pin DATACMD_OLED_Pin
	                           OLED_CS_Pin */
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	  GPIO_InitStruct.Pin = SINUS_OUT1_Pin;
	  HAL_GPIO_Init(SINUS_OUT1_GPIO_Port, &GPIO_InitStruct);
	  GPIO_InitStruct.Pin = SINUS_OUT2_Pin;
	  HAL_GPIO_Init(SINUS_OUT2_GPIO_Port, &GPIO_InitStruct);

	  LL_DAC_Enable(DAC1,  LL_DAC_CHANNEL_1);
	  LL_DAC_Enable(DAC1,  LL_DAC_CHANNEL_2);
}
