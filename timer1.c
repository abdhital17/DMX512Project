//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include "timer1.h"
#include <stdint.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"



//defines
#define D_PIN      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))   //port B1


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
extern void sendByteUart1(uint8_t data);

// Initialize UART0
void initTimer1(uint32_t time)
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

    TIMER1_TAILR_R = 40 * time;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}


void timer1ISR()
{

    if(phase == 0)
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;     // turn-off timer
        TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;   // turn-off interrupts
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;

        D_PIN = 1;
        initTimer1(12);
    }

    if (phase == 1)
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;     // turn-off timer
        TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;   // turn-off interrupts
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;


        GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX;
        UART1_IM_R  |= 0x20;                 //enable the TX interrupt
        sendByteUart1(0x00);
    }
    phase++;

}
