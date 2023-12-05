/*
 * SIN_Sinus.h
 *
 *  Created on: 27.08.2018
 *      Author: Adam
 */

#ifndef SIN_SINUS_H_
#define SIN_SINUS_H_


#define ADC2_ENABLED

#include "stdint.h"
#include "main.h"

 
#define SIN_VERSION 454
// v4.54 - Wylaczenie DAC gdy bez pomiaru
//		 - przycisk on/off
//		 - auto off
//		 - buzzer z generatorem
// v4.52 - uruchomione interfejsy,
//		   kwardatowy sinus
// v4.50 - dziala nina, pot, przycisk
// v4.31 - adresowalny pot. cyfrowy I2c1 adr1, I2c2 adr2
// v4.30 - zmiana pot cyfrowego
// v4.23 - rozszerzone bufory in/out
// v4.22 - usunieta blokada zmiany parametru
// v4.20 - zmiana struktury programu w pamieci
// v4.14 - poprawka dla buzz
// v4.11 - sterowanie przyciskiem start/stop + buzzer
// v4.00 - dodatki do sterowania pod apke
// v3.96 - komenda wyswietlacza OLED
// v3.93 - rozbudowa protokolu  o inkrementacje
// v3.92 - dodatki do wysylania diagnostyki na uarty
// v3.91 - poprawka inicjalizcji zmiennych konfiguracji
// v3.81, 82 poprawka usredniania
// v3.73 dodane ADC2 i I2C2
// v3.72 display LED12 i DET12
// v3.70 dodanawany ADC2
// v3.61 inwersja sterowania DAC
// v3.60 podwojny DAC
// v3.52 zomm lockinow
// v3.51 transmisja USB/CAN dualna
// v3.50 przebudowany lockin
//		 stabline BLe wysylane gdy nie ma obslugi OLED
//		 zmienione odswiezanie OLED
// v3.44 dziala odczyt/zapis/sterowanie po uart
// v3.41 dodawana dwustronna komunkacja i protokol
// v3.40 kontrola potencjomentru z GUI
// v3.30 dodany potencjometr
// v3.22 final release v.3 -
// v3.18 wysylanie lockina i parametrow na uart 115100
// v3.17 obsluga parametrow 2xLED z klawiatury
// v3.16 parametryzowane 2xLED i liczone 2xlockiny
// v3.13 ON/OFF led
// v3.12 inwersja DAC1
// v3.10 tryb swap pod dwa LED na jednym DAC
// v3.08 dodana regulacja fsin
// v3.05 regulacja usredniania
// v3.04 wyswietlanie lockina w rozdz. 1mV
// v2.24 dodany lockin 2 rzêdu
// v2.23 wprowadzony staly offset


#define SIN_SINUS_TAB_SIZE 128 // ilosc probek sinusa

// inwersja DAC1
#define LL_DAC1_OUT(d) LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1,d) //0xFFF-d)
#define LL_DAC2_OUT(d) LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2,d) //0xFFF-d)

uint8_t 	SIN_GenerateDacTable			(uint16_t aConstant01p, uint16_t aSinPikToPik01p, uint8_t aTab);
uint8_t 	SIN_GenerateDacTable_Constant	(uint16_t aConstant01p, uint8_t aTab);
uint8_t 	SIN_GenerateDacTable_SinPP		(uint16_t aPikToPik01p, uint8_t aTab);

extern uint16_t SIN_Offset01p[];
extern uint16_t SIN_Sinus01pp[];


void SIN_Start	(uint32_t aSamples, uint8_t aDivider);
void SIN_Stop	(void);

extern uint8_t   SIN_State;
extern int32_t   LCK_ValueSin,LCK_ValueCos;

extern uint32_t LCK_ValueAmp1,LCK_ValueAmp2;

extern uint32_t SIN_NoiceLevel;

extern uint16_t LCK_AvVal1,LCK_AvVal2;

extern uint8_t LCK_AcuSample;

extern uint8_t SIN_Det;

extern uint8_t   SIN_Run;


#define WORD0(d)  *(0+(uint16_t*)&d)
#define WORD1(d)  *(1+(uint16_t*)&d)
#define WORD2(d)  *(2+(uint16_t*)&d)
#define WORD3(d)  *(3+(uint16_t*)&d)


typedef struct LCK_LOCKINDATA
{

	//int64_t LCK_SumSin;
	//int64_t LCK_SumCos; // sumu z usredniania LCK_PERIODS okresów

	int32_t  ValueSin;
	int32_t  ValueCos;		// obliczony lockin w 16 bitach ze znakiem: z sinusa i cosinusa

	uint32_t ValueSin_Snap;
	uint32_t ValueCos_Snap;  // lockin w 16 bitach ze znakiem
	uint32_t ValueAmp1;
	uint32_t ValueAmp2;
	uint16_t AvVal2;

	uint16_t  AcumulateBuffer[256];
	uint8_t   AkumulateIdx;
	uint8_t   AcuSamplesUsed;
	uint32_t  AkumulateSum1;
	uint32_t  AkumulateSum2;
	uint8_t   AcuSample; // z ilu probek usredniany lockin, max 255
	uint8_t   TotalIdx;
} LCK_LOCKINDATA;


extern LCK_LOCKINDATA LCK_AcuData [];
extern LCK_LOCKINDATA LCK_AcuData2[];



extern uint16_t SIN_SinusDacTable  [][SIN_SINUS_TAB_SIZE];	      //wygeneowana tablica sinusa wyjsciowego do wrzucenia na DAC, zaleznie od ustawionych parametrow offsetu i modulacji
extern uint16_t SIN_Sinus2DacTable [];	  //wygeneowana tablica sinusa wyjsciowego dla diody 2 do wrzucenia na DAC, zaleznie od ustawionych parametrow offsetu i modulacji


uint8_t LCK_UpdateSums(LCK_LOCKINDATA * aAkuData);

void LCK_ChangeAmplifier (uint8_t aValue);

void LCK_OutData(void);

extern uint8_t SIN_Mode; // tryb pracy hex (LSN - led, MSNibble - det)

extern uint8_t  SIN_SwapIt;

void LCK_OutInfoData(void);

extern uint8_t SIN_SquaredSinus;

void SIN_SinusDisable(void);
void SIN_SinusEnable(void);


#endif /* SIN_SINUS_H_ */
