/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2012, 2021 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/F14 V2.03.07.02 [08 Nov 2021]
* Device(s)    : R5F10PPJ
* Tool-Chain   : CCRL
* Description  : This file implements main function.
* Creation Date: 28-08-2023
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_adc.h"
#include "r_cg_timer.h"
/* Start user code for include. Do not edit comment generated here */
#include "math.h"

/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */

#define ADC_BITS    10
#define ADC_COUNTS  (1<<ADC_BITS)
#define False    (uint8_t)0
#define True	(uint8_t)1
#define Timeout  (uint32_t)2000
#define Crossings (uint16_t)20
#define VDD (double)5000.00
#define Vmeas (double)255.00
#define Imeas (double)37.87
#define PhSmeas (double)1.7

extern unsigned long long milliseconds;

uint8_t state;
uint8_t ADC_FLAG = 0;
uint16_t ADC_TEMP = 0;
uint16_t Vadc_Value = 0;
uint16_t CrossingCounts = 0;
uint32_t timeout = Timeout;
unsigned long long TimeStamp, startTimeStamp, currentTimeStamp, tbdelay, tadelay;
double SupplyVoltage = VDD;
double lastFilteredV, filteredV, offsetV, phaseShiftedV, sqV, sumV, Vrms, V_RATIO;
double SampleV, SampleI, filteredI, offsetI, sqI, sumI, Irms, I_RATIO, sumP, instP, realPower, apparentPower, powerFactor, currentTime, energyConsumed;
uint16_t crossCount, crossings, NumberofSamples;
uint8_t lastVCross, checkVCross;
double VCAL, ICAL, PHASECAL;

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
void ZeroCrossingDetection(void);
void R_MAIN_UserInit(void);
void delay_ms(unsigned long long delay);
uint16_t Get_ADCStepValue(uint8_t channel);
unsigned long long Get_TimeStamp(void);
/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
    delay_ms(100);

    while (1)
    {
	ZeroCrossingDetection();
	delay_ms(1000);
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    DI();
    /* Set periperal I/O redirection */
    PIOR0 = 0x00U;
    PIOR1 = 0x00U;
    PIOR2 = 0x00U;
    PIOR3 = 0x00U;
    PIOR4 = 0x00U;
    PIOR5 = 0x00U;
    PIOR6 = 0x00U;
    PIOR7 = 0x00U;
    PIOR8 = 0x00U;
    R_CGC_Get_ResetSource();
    R_CGC_Create();
    R_ADC_Create();
    R_TAU0_Create();

    /* Set invalid memory access detection control */
    IAWCTL = 0x00U;
    
    EI();
    
    R_TAU0_Channel0_Start();
    R_ADC_Set_OperationOn();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */

void CalcVI(void){
	
	crossCount = 0;
	crossings = Crossings;
	NumberofSamples = 0;
	startTimeStamp = Get_TimeStamp();
	lastVCross = False;
	checkVCross = False;
	VCAL = Vmeas;
	ICAL = Imeas;
	PHASECAL = PhSmeas;
	
	offsetV = 512.00;
	offsetI = ADC_COUNTS>>1;
	V_RATIO = VCAL *((SupplyVoltage/1000.0) / ((double)ADC_COUNTS));
	I_RATIO = ICAL *((SupplyVoltage/1000.0) / ((double)ADC_COUNTS));

	while ((crossCount < crossings) && ((Get_TimeStamp()-startTimeStamp)<timeout)){
		
		NumberofSamples++;
		
		lastFilteredV = filteredV;
		
		SampleV = (double)Get_ADCStepValue(25);
		SampleI = (double)Get_ADCStepValue(24);
		
		//offsetV = offsetV + ((SampleV-offsetV)/1024);
		filteredV = SampleV - offsetV;
		
		//offsetI = offsetI + ((SampleI-offsetI)/1024);
    		filteredI = SampleI - offsetI;
		
		sqV = filteredV * filteredV;
		sumV += sqV;
		sqI = filteredI * filteredI;
    		sumI += sqI;
		
		//phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);
		
		//instP = phaseShiftedV * filteredI;          //Instantaneous Power
    		//sumP += instP;                               //Sum
		
		lastVCross = checkVCross;
		
    		if (SampleV > Vadc_Value){
			checkVCross = True;
		}
                else {
		checkVCross = False;
		}
		
    		if (NumberofSamples==1) {
			lastVCross = checkVCross;
		}
		
		if (lastVCross != checkVCross){
			crossCount++;
		}
		
	}
	
  	Vrms = sqrt(sumV / (double)NumberofSamples);
	Vrms = V_RATIO * Vrms;
	
  	Irms = sqrt(sumI / (double)NumberofSamples);
	Irms = I_RATIO * Irms;

  	//realPower = V_RATIO * I_RATIO * sumP / (double)NumberofSamples;
  	//apparentPower = Vrms * Irms;
  	//powerFactor = realPower / apparentPower;
	//currentTimeStamp = Get_TimeStamp();
	//currentTime = (double)currentTimeStamp/3600000.00; 
	//energyConsumed = apparentPower * powerFactor * currentTime;
	
	sumV = 0;
	sumI = 0;
 	//sumP = 0;
	
}

void ZeroCrossingDetection(void)
{
	state = False;
	startTimeStamp = Get_TimeStamp();
	
	while(state == False){
		Vadc_Value = Get_ADCStepValue(25);
		delay_ms(5);
		
		if ((Vadc_Value < (564)) && (Vadc_Value > (461))){
			state = True; 
		}
		if ((Get_TimeStamp()-startTimeStamp)>timeout){
			state = True;
		}
	}

	CalcVI();
}

void delay_ms(unsigned long long delay)
{
	tbdelay = 0;
	tadelay = 0;
	tbdelay = Get_TimeStamp();
	tadelay = Get_TimeStamp();
	while((tadelay - tbdelay) < delay){
		tadelay = Get_TimeStamp();
	}
	tbdelay = 0;
	tadelay = 0;
}

uint16_t Get_ADCStepValue(uint8_t channel)
{
	R_ADC_Start();
	ADS = channel;
	if(ADC_FLAG == 1){
		ADC_FLAG = 0;  
		R_ADC_Get_Result(&ADC_TEMP);
		delay_ms(2);
		R_ADC_Stop();
	}
	
	return ADC_TEMP;
}

unsigned long long Get_TimeStamp(void){
	TimeStamp = milliseconds;
	return TimeStamp;
}
/* End user code. Do not edit comment generated here */
