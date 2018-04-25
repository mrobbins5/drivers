#include "timers.h"


//*****************************************************************************
// Verifies that the base address is a valid GPIO base address
//*****************************************************************************
static bool verify_base_addr(uint32_t base_addr)
{
   switch( base_addr )
   {
     case TIMER0_BASE:
     case TIMER1_BASE:
     case TIMER2_BASE:
     case TIMER3_BASE:
     case TIMER4_BASE:
     case TIMER5_BASE:
     {
       return true;
     }
     default:
     {
       return false;
     }
   }
}

//*****************************************************************************
// Returns the RCGC and PR masks for a given TIMER base address
//*****************************************************************************
static bool get_clock_masks(uint32_t base_addr, uint32_t *timer_rcgc_mask, uint32_t *timer_pr_mask)
{
  // Set the timer_rcgc_mask and timer_pr_mask using the appropriate
  // #defines in ../include/sysctrl.h
  switch(base_addr)
  {
    case TIMER0_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R0;
      *timer_pr_mask = SYSCTL_PRTIMER_R0;
      break;
    }
    case TIMER1_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R1;
      *timer_pr_mask = SYSCTL_PRTIMER_R1;
      break;
    }
    case TIMER2_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R2;
      *timer_pr_mask = SYSCTL_PRTIMER_R2;
      break;
    }
    case TIMER3_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R3;
      *timer_pr_mask = SYSCTL_PRTIMER_R3;
      break;
    }
    case TIMER4_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R4;
      *timer_pr_mask = SYSCTL_PRTIMER_R4;
      break;
    }
    case TIMER5_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R5;
      *timer_pr_mask = SYSCTL_PRTIMER_R5;
      break;
    }
    default:
    {
      return false;
    }
  }
  return true;
}


//*****************************************************************************
// Waits for 'ticks' number of clock cycles and then returns.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_wait(uint32_t base_addr, uint32_t ticks)
{
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }

  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;

  //*********************    
  // ADD CODE
  //*********************
	gp_timer -> CTL &= ~TIMER_CTL_TAEN;
	gp_timer -> CTL &= ~TIMER_CTL_TBEN;
	
	gp_timer -> TAILR = ticks;
	
	gp_timer -> ICR |= TIMER_ICR_TATOCINT;
	
	gp_timer -> CTL |= TIMER_CTL_TAEN;
	
	while(!(gp_timer -> RIS & TIMER_RIS_TATORIS)){};
  
  return true;
}


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
bool gp_timer_config_32(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts)
{
  uint32_t timer_rcgc_mask;
  uint32_t timer_pr_mask;
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // get the correct RCGC and PR masks for the base address
  get_clock_masks(base_addr, &timer_rcgc_mask, &timer_pr_mask);
  
  // Turn on the clock for the timer
  SYSCTL->RCGCTIMER |= timer_rcgc_mask;

  // Wait for the timer to turn on
  while( (SYSCTL->PRTIMER & timer_pr_mask) == 0) {};
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;
    
  //*********************    
  // ADD CODE
  //*********************
	gp_timer -> CTL &= ~TIMER_CTL_TAEN;
	gp_timer -> CTL &= ~TIMER_CTL_TBEN;
		
	gp_timer -> CFG |= TIMER_CFG_32_BIT_TIMER;
	
	gp_timer -> TAMR &= ~TIMER_TAMR_TAMR_M;
	gp_timer -> TAMR |= (mode);
		
	if (count_up){
		gp_timer -> TAMR |= ~TIMER_TAMR_TACDIR;
	}
	else{
		gp_timer -> TAMR &= ~TIMER_TAMR_TACDIR;
	}
	
	if (enable_interrupts){
		gp_timer -> IMR |= TIMER_IMR_TATOIM;
	}
	else{
		gp_timer -> IMR &= ~TIMER_IMR_TATOIM;
	}

    
  return true;  
}


//*****************************************************************************
// Configure a general purpose timer to be a 16-bit timer.  
//
// Paramters
//  base_address          The base address of a general purpose timer
//
//  up_dwn_n						 set to true to count up, and false to count down
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  timer									boolean to select which timer true => A, false =>B
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_16(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts, uint32_t ticks)
{
  uint32_t timer_rcgc_mask;
  uint32_t timer_pr_mask;
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // get the correct RCGC and PR masks for the base address
  get_clock_masks(base_addr, &timer_rcgc_mask, &timer_pr_mask);
  
  // Turn on the clock for the timer
  SYSCTL->RCGCTIMER |= timer_rcgc_mask;

  // Wait for the timer to turn on
  while( (SYSCTL->PRTIMER & timer_pr_mask) == 0) {};
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;
    
  //*********************    
  // ADD CODE
  //*********************
		gp_timer->CTL &= ~TIMER_CTL_TAEN; 
		gp_timer->CTL &= ~TIMER_CTL_TBEN;
    
		gp_timer->CFG = TIMER_CFG_16_BIT; 
		
		//First clear the mode bits
		gp_timer->TAMR &= ~TIMER_TAMR_TAMR_M; 
		gp_timer->TBMR &= ~TIMER_TBMR_TBMR_M; 

		
		
    //set the timer's mode based on the 'mode' parameter passed to the function
		gp_timer->TAMR |= mode;
		gp_timer->TBMR |= mode;
		
		//Set direction to count up if requested
		if(count_up){
			gp_timer->TAMR |= TIMER_TAMR_TACDIR;
			gp_timer->TBMR |= TIMER_TBMR_TBCDIR;
		} else{
			gp_timer->TAMR &= ~TIMER_TAMR_TACDIR; 
			gp_timer->TBMR &= ~TIMER_TBMR_TBCDIR; 
		}
		
		if(enable_interrupts){
			gp_timer->IMR |= TIMER_IMR_TATOIM; 
			gp_timer->IMR |= TIMER_IMR_TBTOIM;
		} else {
			gp_timer->IMR &= ~TIMER_IMR_TATOIM;
			gp_timer->IMR &= ~TIMER_IMR_TBTOIM;
		}
		// Set the Priority /////***added by mark
  NVIC_SetPriority(TIMER0A_IRQn, 1);
  NVIC_SetPriority(TIMER0B_IRQn, 1);
		
  // Enable the Interrupt in the NVIC
  NVIC_EnableIRQ(TIMER0A_IRQn);
	NVIC_EnableIRQ(TIMER0B_IRQn);
	
		///****** ^^^added by mark
		
		gp_timer->TAILR |= ticks; 
		gp_timer->TBILR |= ticks; 
		
		gp_timer->TAPR &= ~TIMER_TAPR_TAPSR_M;
		gp_timer->TAPR |= 7; 
		
		gp_timer->TBPR &= ~TIMER_TBPR_TBPSR_M;
		gp_timer->TBPR |= 15; 
		
		//Renable Timers
		gp_timer->CTL |= TIMER_CTL_TAEN | TIMER_CTL_TBEN; 


		//**WRite a 1 to interrupt clear register
  return true;  
}

//*****************************************************************************
// Set ILR and starts timer
// timer = true => timerA, timer = false => timerB
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_start_16(uint32_t base_addr, bool timer, uint8_t prescale, uint16_t TILR)
{
  TIMER0_Type *gp_timer;
	
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;
	
	if (timer) {								// if timerA
		gp_timer->TAILR = TILR;
		gp_timer->TAPR = prescale;
		gp_timer->CTL |= TIMER_CTL_TAEN;
	}
	else {											// if timerB
		gp_timer->TBILR = TILR;
		gp_timer->TBPR = prescale;
		gp_timer->CTL |= TIMER_CTL_TBEN;
	}		
    
  return true;  
}


void clearTimer0A(uint32_t base_addr){
	
	 TIMER0_Type *gp_timer;
	 gp_timer = (TIMER0_Type *)base_addr;
	
	 gp_timer->ICR |= TIMER_ICR_TATOCINT; 
	
}
void clearTimer0B(uint32_t base_addr){
	
	TIMER0_Type *gp_timer;
	gp_timer = (TIMER0_Type *)base_addr;
	
	gp_timer->ICR |= TIMER_ICR_TBTOCINT; 
	
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
//New timer to set up in PWM mode
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool gp_timer_config_16PWM(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts, uint32_t ticks)
{
  uint32_t timer_rcgc_mask;
  uint32_t timer_pr_mask;
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // get the correct RCGC and PR masks for the base address
  get_clock_masks(base_addr, &timer_rcgc_mask, &timer_pr_mask);
  
  // Turn on the clock for the timer
  SYSCTL->RCGCTIMER |= timer_rcgc_mask;

  // Wait for the timer to turn on
  while( (SYSCTL->PRTIMER & timer_pr_mask) == 0) {};
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;
    
  //*********************    
  // ADD CODE
  //*********************
		gp_timer->CTL &= ~TIMER_CTL_TAEN; 
		gp_timer->CTL &= ~TIMER_CTL_TBEN;
    
		gp_timer->CFG = TIMER_CFG_16_BIT; 
		
		//First clear the mode bits
		gp_timer->TAMR &= ~TIMER_TAMR_TAMR_M; 

    //set the timer's mode based on the 'mode' parameter passed to the function
		gp_timer->TAMR |= mode;
		
		gp_timer ->TAMR &= ~TIMER_TAMR_TACMR; //Clear the TACMR bit. 
		gp_timer ->TAMR |= TIMER_TAMR_TAAMS; //Set bit to enable PWM mode
		
		//Set direction to count up if requested
		if(count_up){
			gp_timer->TAMR |= TIMER_TAMR_TACDIR;
		} else{
			gp_timer->TAMR &= ~TIMER_TAMR_TACDIR; 

		}
		
		if(enable_interrupts){
			gp_timer->IMR |= TIMER_IMR_TATOIM; 
		} else {
			gp_timer->IMR &= ~TIMER_IMR_TATOIM;
		}
		// Set the Priority /////***added by mark
  NVIC_SetPriority(TIMER0A_IRQn, 1);
		
  // Enable the Interrupt in the NVIC
  NVIC_EnableIRQ(TIMER0A_IRQn);
	
		///****** ^^^added by mark
		
		gp_timer->TAILR |= ticks; 
		
		gp_timer->TAPR &= ~TIMER_TAPR_TAPSR_M;
		gp_timer->TAPR |= 7; 
		
		
		//Renable Timers
		gp_timer->CTL |= TIMER_CTL_TAEN; 

  return true;  
}