//This file pertains to the Quadrature Encoded Dial

#include "quadraturedial.h"
#include "adc.h"
#include "driver_defines.h"




/*******************************************************************************
* Function Name: initialize_adc_gpio_pins
********************************************************************************
* Used to initialize the PS2 joystick GPIO pins for both the analog inputs
*
* Configuration Info
*		Fill out relevant information in ps2.h.  ps2.h defines 
*		how various peripherals are physically connected to the board.
*******************************************************************************/
void initialize_dial(void)
{
	  
    //uint32_t rcgc_mask = SYSCTL_RCGC1_QEI0 ;
    //uint32_t pr_mask = SYSCTL_RCGC1_QEI0;
    //rcgc mask should be 1 same with pr mask
	  uint32_t rcgc_mask = 0x00000001 ;
    uint32_t pr_mask = 0x00000001;
	  QEI0_Type *qei = (QEI0_Type *)(QUADRATUREDIAL_QEI_BASE);
	  gpio_enable_port(QUADRATUREDIAL_GPIO_BASE);
    
		gpio_config_alternate_function(QUADRATUREDIAL_GPIO_BASE, QUADRATUREDIAL_PhA_MASK);
	  gpio_config_alternate_function(QUADRATUREDIAL_GPIO_BASE, QUADRATUREDIAL_PhB_MASK);
	
	  gpio_config_port_control(QUADRATUREDIAL_GPIO_BASE, GPIO_PCTL_PD7_M, GPIO_PCTL_PD7_PHB0);
	  gpio_config_port_control(QUADRATUREDIAL_GPIO_BASE, GPIO_PCTL_PD6_M, GPIO_PCTL_PD6_PHA0);
	
	  //gpio_config_enable_input(QUADRATUREDIAL_GPIO_BASE, QUADRATUREDIAL_PhA);
	  //gpio_config_enable_input(QUADRATUREDIAL_GPIO_BASE, QUADRATUREDIAL_PhB);
	
	  gpio_config_digital_enable(QUADRATUREDIAL_GPIO_BASE, QUADRATUREDIAL_PhA_MASK);
	  gpio_config_digital_enable(QUADRATUREDIAL_GPIO_BASE, QUADRATUREDIAL_PhB_MASK);
	
	  //gpio_config_enable_pullup(uint32_t baseAddr, uint8_t pins)
	  //Enable the QEI clock using the RCGCQEI register in the System Control module
	  SYSCTL->RCGCQEI |= rcgc_mask;
	  //while(!(SYSCTL->PRQEI & pr_mask)); 
		while( (pr_mask & SYSCTL->PRQEI) != pr_mask){}
			
    
		
		qei->CTL    |= 0x000F2008;
    qei->MAXPOS |= 0x00000060;	
			
		//Enable the quadrature encoder
		qei->CTL    |= 0x00000001;
		
}

/*******************************************************************************
* Function Name: quadraturedial_getpos
********************************************************************************
*Returns the most current reading of the X direction  Only the lower 12-bits
* contain data.
********************************************************************************/
uint32_t quadraturedial_getpos(uint32_t qei_base)
{
	
  uint32_t rot_val;
	QEI0_Type *qei = (QEI0_Type *)(QUADRATUREDIAL_QEI_BASE);
  rot_val = qei->POS;
  return rot_val;
}


