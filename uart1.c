//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U1TX (PB1) and U1RX (PB0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "uart1.h"
#include "PWM.h"
#include <stdio.h>
#include "wait.h"


// PortB masks
#define R_MASK 1
#define D_MASK 2

#define DE_PIN     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))   //port C7
#define D_PIN      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))   //port B1 -> UART1 TX

#define RED_LED   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  //PF1
#define BLUE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))  //PF2
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  //PF3

//LED counts
extern uint8_t LED_OFF_TIMEOUT;
extern uint8_t LED_ON_TIMEOUT;

uint16_t Rxphase;


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//function that takes a 32 bit integer as argument
//and then prints out the corresponding Hex value for the 32 bit integer


// Initialize UART1
void initUart1()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTB_DIR_R |= D_MASK;                   // enable output on UART1 TX pin
    GPIO_PORTB_DIR_R &= ~R_MASK;                   // enable input on UART1 RX pin
    GPIO_PORTB_DR2R_R |= D_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= D_MASK | R_MASK;          // enable digital on UART1 pins
    GPIO_PORTB_AFSEL_R &= ~(D_MASK | R_MASK);     // DO *NOT* use peripheral to drive PB0, PB1; USE GPIO
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M); // clear bits 0-7

    // Configure UART0 to 115200 baud, 8N1 format
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART1_IBRD_R = 10;                                  // r = 40 MHz / (Nx250kHz), set floor(r)=10, where N=16
    UART1_FBRD_R = 0;                                  // round(fract(r)*64)=0
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;

   UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT;
                                                        // enable TX, RX, and module
   NVIC_EN0_R |= 1 << (INT_UART1-16);                  // turn-on interrupt 22 (UART1)

}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART1_IBRD_R = divisorTimes128 >> 7;                 // set integer value to floor(r)
    UART1_FBRD_R = ((divisorTimes128 + 1)) >> 1 & 63;    // set fractional value to round(fract(r)*64)
}


//function to send a byte of data through the UART1 TX
void sendByteUart1(uint8_t data)
{
    while (UART1_FR_R & UART_FR_TXFF);                   // wait if uart1 tx fifo full
    UART1_DR_R = data;                                  // write character to fifo
}


//Interrupt Service Routine to handle either RX or TX interrupt for the UART1 module
void uart1ISR()
{
   if (UART1_MIS_R & 0x20)           //if tx interrupt triggered the isr
   {
       if((phase - 2) < max)
       {
           if(pollMode)         //if the controller is polling the device, send 1 if the pollIndex = phase -2; otherwise send a 0; pollIndex keeps on increasing to 512
           {
               if ((phase - 2) == pollIndex)
               {
                   sendByteUart1(0x1);
               }
               else
                   sendByteUart1(0);
           }

           else     //if not polling mode
               sendByteUart1(dataTable[phase - 2]);

           phase++;
        }

       else                 //if phase - 2 >= max
       {
           UART1_IM_R  &= ~0x20;                 //disable the TX interrupt for UART1

           GPIO_PORTB_AFSEL_R &= ~(D_MASK);  // *DO NOT* use peripheral to drive PA1 (UART1 TX)
           GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M); // clear bits 4-7

           if (ON)
           {
               while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
               DE_PIN = 0;
               phase = 0;
               startDMX_TX();
           }

           if(pollMode)
           {
//               displayUart0("device waiting for ack\n\r");
               //enter code on what the controller does between when it finishes requesting ACK at an address and before it starts requesting ack at (address + 1)
               checkBreak = true;


               UART1_ECR_R = 0;
               UART1_ICR_R |= 0x280;
               //prepare the controller to start receiving
//               while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
               UART1_IM_R &= ~0x20;                //disable the UART1 TX interrupt
               UART1_ICR_R |= 0x20;                 //clear the interrupt flags

               GPIO_PORTB_AFSEL_R &= ~(D_MASK);  // *DO NOT* use peripheral to drive PA1 (UART1 TX)
               GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M); // clear bits 4-7

               GPIO_PORTB_AFSEL_R |= R_MASK;          // use peripheral to drive PA1 (UART1 TX)
               GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M); // clear bits 0-3
               GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB0_U1RX); // set bits 0-3 for UART1 RX control

               UART1_IM_R  |= 0x10;                 //enable the UART1 RX interrupt/ start receiving ack From the device at address = pollIndex

           }

       }



   }

   else if (UART1_MIS_R & 0x10)             //if rx interrupt triggered the isr
   {                                         //0xABCDEF is the controller flag; determines that the mode is in controller mode
       if (pollMode && checkBreak && MODE == 0xABCDEF)               //if on polling mode and it is at the phase where the controller tries to receive an ACK
       {
//           displayUart0("check4\r\n");
           DE_PIN = 0;
           waitMicrosecond(300);
           uint16_t data = UART1_DR_R;
           if ((data & 0x400) )
           {
               if(UART1_RIS_R & 0x280)
               {
                   pollFound[pollIndex] = 1;
                   GREEN_LED = 1;
                   LED_OFF_TIMEOUT = 2;
               }

               else
                   pollFound[pollIndex] = 0;
           }

           checkBreak = false;

           //        //preparing the controller to send data again
           while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
           GPIO_PORTB_AFSEL_R &= ~(R_MASK | D_MASK);  // *DO NOT* use peripheral to drive PB0 (UART1 TX and RX)
           GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M); // clear bits 0-3

           UART1_IM_R  &= ~0x30;                 //disable the UART1 RX interrupt
           UART1_ICR_R |= 0x30;

           poll();
       }

       else             //if NOT in the controller mode where (controller is looking for ACK), resume normal functioning
                        //only device mode reaches to this part
       {
           //displayUart0("check3\r\n");
           uint16_t data = UART1_DR_R;

           if (data & 0x400)            //if break error occured in data register
           {
               initLEDPWM();
               Rxphase = 0;
               setLEDPWM(2, dataTable[devAddr]);
               GREEN_LED = 0;
           }

           else                         //if not a break
           {
              if(Rxphase <= 515)
               {
                   dataTable[Rxphase] = (data & 0xFF);
                   GREEN_LED = 1;


                   if (Rxphase == 0 && (dataTable[Rxphase] & 0xFF) == 0xF7)              //since while receiving, the value at index 0 is always the start code,
                       pollMode = true;                                                     //check at index 0 to see whether the controller is sending a break

                           if((pollMode) && (Rxphase == 512))
                           {
                               if(dataTable[devAddr] == 1)
                               {
                                   while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
                                   UART1_IM_R  &= ~0x30;                              //disable the UART1 RX/TX interrupt/ stop receiving; prepare to acknowledge
                                   GPIO_PORTB_AFSEL_R &= ~(R_MASK | D_MASK);          // do not peripheral to drive PA1 (UART1 TX)
                                   GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M); // clear bits 0-3


                                   ///-----------------------code and discard-------------------------------------------
                                   waitMicrosecond(12);

                                   DE_PIN = 1;
                                   D_PIN  = 0;         //pull D pin low to signal a break(ACK in this case)

                                   waitMicrosecond(176);

                                   D_PIN = 0;       //ACK

                                   RED_LED = 1;
                                   LED_OFF_TIMEOUT = 2;

                                   DE_PIN = 0;
                                   pollMode = false;               //this happens in receiver mode since its responsibility to send an ACK is completed
                                   MODE = 0xFFFFFFFF;

                                   while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
                                   GPIO_PORTB_AFSEL_R |= R_MASK;          // use peripheral to drive PA1 (UART1 TX)
                                   GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M); // clear bits 0-3
                                   GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB0_U1RX); // set bits 0-3 for UART1 RX control

                                   UART1_IM_R  |= 0x10;                 //enable the UART1 RX interrupt for normal device mode functioning

                               }
                               pollIndex++;
                               if(pollIndex == 513)
                                   pollMode = false;
                           }

                   Rxphase++;
               }

           }
       }

   }
}
