// Copyright (c) 2014, Joe Krachey
// All rights reserved.
//
// Redistribution and use in binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "adc.h"
#include "driver_defines.h"

/******************************************************************************
 * Initializes ADC to use Sample Sequencer #3, triggered by the processor,
 * no IRQs
 *****************************************************************************/
bool initialize_adc(  uint32_t adc_base )
{
  ADC0_Type  *myADC;
  uint32_t rcgc_adc_mask;
  uint32_t pr_mask;
  

  // examine the adc_base.  Verify that it is either ADC0 or ADC1
  // Set the rcgc_adc_mask and pr_mask  
  switch (adc_base) 
  {
    case ADC0_BASE :
    {
      
      // ADD CODE
      // set rcgc_adc_mask
				 rcgc_adc_mask = SYSCTL_RCGCADC_R0;
			
      // ADD CODE
      // Set pr_mask 
				pr_mask = SYSCTL_PRADC_R0;
			
      break;
    }
    case ADC1_BASE :
    {
      // ADD CODE
      // set rcgc_adc_mask
				 rcgc_adc_mask = SYSCTL_RCGCADC_R1;

      // ADD CODE
      // Set pr_mask 
				pr_mask = SYSCTL_PRADC_R1;

			
      break;
    }
    
    default:
      return false;
  }
  
  // Turn on the ADC Clock
  SYSCTL->RCGCADC |= rcgc_adc_mask;
  
  // Wait for ADCx to become ready
  while( (pr_mask & SYSCTL->PRADC) != pr_mask){}
    
  // Type Cast adc_base and set it to myADC
  myADC = (ADC0_Type *)adc_base;
  
  // ADD CODE
  // disable sample sequencer #3 by writing a 0 to the 
  // corresponding ASENn bit in the ADCACTSS register 
	myADC->ACTSS &= ~ADC_ACTSS_ASEN3;	

  // ADD CODE
  // Set the event multiplexer to trigger conversion on a processor trigger
  // for sample sequencer #3.
	myADC->EMUX &= ~ADC_EMUX_EM3_M;	
	myADC->EMUX |= ADC_EMUX_EM3_PROCESSOR;
		
  // ADD CODE
  // Set IE0 and END0 in SSCTL3
	myADC->SSCTL3 = ADC_SSCTL3_END0 | ADC_SSCTL3_IE0; 
  
		
  return true;
}


/******************************************************************************
 * Reads SSMUX3 for the given ADC.  Busy waits until completion
 *****************************************************************************/
uint32_t get_adc_value( uint32_t adc_base, uint8_t channel)
{
  ADC0_Type  *myADC;
  uint32_t result;
  
  if( adc_base == 0)
  {
    return false;
  }
  
  myADC = (ADC0_Type *)adc_base;
  
  myADC->SSMUX3 = channel;          // Set the Channel
  
  myADC->ACTSS |= ADC_ACTSS_ASEN3;  // Enable SS3
  
  myADC->PSSI =   ADC_PSSI_SS3;     // Start SS3
  
  while( (myADC->RIS & ADC_RIS_INR3)  == 0)
  {
    // wait
  }
  
  result = myADC->SSFIFO3 & 0xFFF;    // Read 12-bit data
  
  myADC->ISC  = ADC_ISC_IN3;          // Ack the conversion
  
  return result;
}

/******************************************************************************
 * Mark and Tanvi Version
 *****************************************************************************/
//******************************************************************************
 bool initialize_adcHW(  uint32_t adc_base,uint8_t channel1, uint8_t channel2 )
 {
  ADC0_Type  *myADC;
  uint32_t rcgc_adc_mask;
  uint32_t pr_mask;
  

  // examine the adc_base.  Verify that it is either ADC0 or ADC1
  // Set the rcgc_adc_mask and pr_mask  
  switch (adc_base) 
  {
    case ADC0_BASE :
    {
      
      // ADD CODE
      // set rcgc_adc_mask
				 rcgc_adc_mask = SYSCTL_RCGCADC_R0;
			
      // ADD CODE
      // Set pr_mask 
				pr_mask = SYSCTL_PRADC_R0;
			
      break;
    }
    case ADC1_BASE :
    {
      // ADD CODE
      // set rcgc_adc_mask
				 rcgc_adc_mask = SYSCTL_RCGCADC_R1;

      // ADD CODE
      // Set pr_mask 
				pr_mask = SYSCTL_PRADC_R1;

			
      break;
    }
    
    default:
      return false;
  }
  
  // Turn on the ADC Clock
  SYSCTL->RCGCADC |= rcgc_adc_mask;
  
  // Wait for ADCx to become ready
  while( (pr_mask & SYSCTL->PRADC) != pr_mask){}
    
  // Type Cast adc_base and set it to myADC
  myADC = (ADC0_Type *)adc_base;
  
  // ADD CODE
  // disable sample sequencer #3 by writing a 0 to the 
  // corresponding ASENn bit in the ADCACTSS register 
	myADC->ACTSS &= ~ADC_ACTSS_ASEN2;	

  // ADD CODE
  // Set the event multiplexer to trigger conversion on a processor trigger
  // for sample sequencer #3.
	myADC->EMUX &= ~ADC_EMUX_EM2_M;	
	myADC->EMUX |= ADC_EMUX_EM2_PROCESSOR;
		
  // ADD CODE
	//clear here
	//myADC->SSCTL2 &= ~ADC_SSMUX2_MUX3_M;
  // Set IE0 and END0 in SSCTL2
	myADC->SSCTL2 &= 	~ADC_SSCTL2_END0;
	myADC->SSCTL2 &= 	~ADC_SSCTL2_END2;
	myADC->SSCTL2 &= 	~ADC_SSCTL2_END3;	
	
	myADC->SSCTL2 &= 	~ADC_SSCTL2_IE2;	
	myADC->SSCTL2 &= 	~ADC_SSCTL2_IE3;	
		
	myADC->SSCTL2 = ADC_SSCTL2_END1| ADC_SSCTL2_IE0 | ADC_SSCTL2_IE1; 
  
	myADC->IM &= ~ADC_IM_MASK0;
	myADC->IM &= ~ADC_IM_MASK1;
	myADC->IM &= ~ADC_IM_MASK3;
	
	myADC->IM |= ADC_IM_MASK2;
	//clear the four channels
	myADC->SSMUX2 &= ~ADC_SSMUX2_MUX3_M;
	myADC->SSMUX2 &= ~ADC_SSMUX2_MUX2_M;
	myADC->SSMUX2 &= ~ADC_SSMUX2_MUX1_M;
	myADC->SSMUX2 &= ~ADC_SSMUX2_MUX0_M;
	
	//Set the channels to some values
	myADC->SSMUX2 = channel1|(channel2<<4);
	NVIC_SetPriority(ADC0SS2_IRQn, 1);
		
  // Enable the Interrupt in the NVIC
  NVIC_EnableIRQ(ADC0SS2_IRQn);
	
  myADC->ACTSS |= ADC_ACTSS_ASEN2;  // Enable SS2
  return true;
}
 

//Kick off the conversion
void kickoff( uint32_t adc_base){
	ADC0_Type  *myADC;
   myADC = (ADC0_Type *)adc_base;
  
  myADC->PSSI =   ADC_PSSI_SS2;     // Start SS2
	//myADC->PSSI |= ADC_PSSI_SS1; 
}


uint32_t numberGetter(uint32_t adc_base){
	ADC0_Type  *myADC;
	uint32_t result;
  myADC = (ADC0_Type *)adc_base;
  result = myADC->SSFIFO2 & 0xFFFF;    // Read 12-bit data
	result = result; 
  myADC->ISC  = ADC_ISC_IN2;          // Ack the conversion
	return result;
}
