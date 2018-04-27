//This is the header file for the Quadrature Dial 

#ifndef __QUADRATUREDIAL_H__
#define __QUADRATUREDIAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "gpio_port.h"
#include "adc.h"
#include "driver_defines.h"


// ADD CODE
// Define the base addresses for the GPIO port that is connected to the 
#define   QUADRATUREDIAL_GPIO_BASE    GPIOD_BASE
#define   QUADRATUREDIAL_QEI_BASE			QEI0_BASE

// ADD CODE
// Define which bit number ( 0 through 7) each signal is connected to on the GPIO port.
// This bit number is NOT the value listed in the Pin Number column of the datasheet.  
// As an example, if the analog input was connected to pin PD2, you would set the bit
// number to 2.
#define   QUADRATUREDIAL_PhA      6
#define   QUADRATUREDIAL_PhB      7

// Macros passed to the gpio functions that are used to configure the GPIO port.
#define   QUADRATUREDIAL_PhA_MASK     (1 << QUADRATUREDIAL_PhA)
#define   QUADRATUREDIAL_PhB_MASK     (1 << QUADRATUREDIAL_PhB)

// ADD CODE
// Define the base addresses of the the ADC you are going to use to either ADC0 or ADC1.
// Either will work.


// ADD CODE
// Set the analog channel for each direction.  Each analog channel is associated with a
// single GPIO pin.  Make sure to see table 13-1 to see how the GPIO pin maps to an 
// analog channel.  Specifically, look at the column Pin Name.


/*******************************************************************************
* Function Name: ps2_initialize
********************************************************************************
* Initializes the GPIO pins connected to the PS2 Joystick.  It also configures
* ADC0 to use Sample Sequencer #3 to convert a programmable channel number.
*******************************************************************************/
void initialize_dial(void);

/*******************************************************************************
* Function Name: quadraturedial_getpos
********************************************************************************
*Returns the most current reading of the X direction  Only the lower 12-bits
* contain data.
********************************************************************************/
uint32_t quadraturedial_getpos(uint32_t qei_base);


#endif

