//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include "../../workspace_v10/DMX512Project/timer1.h"

#include <stdint.h>
#include "tm4c123gh6pm.h"




//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initTimer1(uint32_t time_in_us)
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    SYSCTL_GPIOHBCTL_R = 0;             //system use APB


    // Enable clocks
    SYSCTL_RCGCTIMER_R |=SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);


    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for one shot mode (count down)

    TIMER1_TAILR_R = 40 * time_in_us;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}


void timer1ISR()
{
    if(phase == 0)
    {
        phase = 1;
        D_PIN = 1;
        initTimer1(12);
    }

    if (phase == 1)
    {
        phase = 2;
        sendByteUart1(0x00);
    }
}
