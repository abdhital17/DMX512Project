#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8



//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// PWM Interface:
//   M1PWM5 (PF1), M1PWM6 (PF2) and M1PWM7 (PF3) are connected to the 2nd controller

/*                     PWM1
 * Generator 0 ->  A            M1-PWM0
 * Generator 0 ->  B            M1-PWM1

 * Generator 1 ->  A            M1-PWM2
 * Generator 1 ->  B            M1-PWM3

 * Generator 2 ->  A            M1-PWM4
 * Generator 2 ->  B            M1-PWM5

 * Generator 3 ->  A            M1-PWM6
 * Generator 3 ->  B            M1-PWM7
 */

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize PWM for on-board LED's
void initLEDPWM()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three on-board LEDs
    GPIO_PORTF_DIR_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;                       // make bit 1,2,3 GPIO outputs
    GPIO_PORTF_DR2R_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;                      // set drive strength to 2mA
    GPIO_PORTF_DEN_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;                       // enable digital
    GPIO_PORTF_AFSEL_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;                     // select auxilary function
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M |GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PWM module 1 to drive on-board RGB LEDs
    // RED   on M1PWM5 (PF1), M1PWM2B
    // BLUE  on M0PWM6 (PF2), M1PWM3A
    // GREEN on M0PWM7 (PF3), M1PWM3B

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2
    PWM1_3_CTL_R = 0;                                // turn-off PWM0 generator 3
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM1_2_LOAD_R = 256;                            // set period to 40 MHz sys clock / 2 / 255 = 78.43137 kHz
    PWM1_3_LOAD_R = 256;
    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
                                                     // invert outputs so duty cycle increases with increasing compare values
    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPA_R = 0;                               // blue off
    PWM1_3_CMPB_R = 0;                               // green off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}


//uint8_t rgb = 1 for RedLED, rgb = 2 for Blue LED, rgb =3 for Green LED
void setLEDPWM(uint8_t rgb, uint8_t cmpb)
{
    if (rgb == 1)
        PWM1_2_CMPB_R = cmpb;

    else if (rgb == 2)
        PWM1_3_CMPA_R = cmpb;

    else if (rgb == 3)
        PWM1_3_CMPB_R = cmpb;
}

void uninitPWM()
{
    GPIO_PORTF_AFSEL_R &= ~(RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK);                     // use GPIO to control the LEDs
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M |GPIO_PCTL_PF3_M);
}
