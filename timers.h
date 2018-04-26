#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <stdbool.h>
#include <stdint.h>

#include "driver_defines.h"


//*****************************************************************************
// Configure a general purpose timer to be a 32-bit timer.  
//
// Paramters
//  base_address          The base address of a general purpose timer
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  count_up              When true, the timer counts up.  When false, it counts
//                        down
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_32(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts);


//*****************************************************************************
// Waits for 'ticks' number of clock cycles and then returns.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_wait(uint32_t base_addr, uint32_t ticks);


//*****************************************************************************
// Configure a general purpose timer to be a 16-bit timer.  
//
// Paramters
//  base_address          The base address of a general purpose timer
//
//  up_dwn_n                         set to true to count up, and false to count down
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  timer                                   boolean to select which timer true => A, false =>B
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_16(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts, uint32_t ticks);

//*****************************************************************************
// Set ILR and starts timer
// timer = true => timerA, timer = false => timerB
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_start_16(uint32_t base_addr, bool timer, uint8_t prescale, uint16_t TILR);



void clearTimer0A(uint32_t base_addr);
void clearTimer0B(uint32_t base_addr);

//PWM
bool gp_timer_config_16PWM(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts, uint32_t ticks);




#endif
