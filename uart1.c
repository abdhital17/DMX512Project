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


// PortB masks
#define R_MASK 1
#define D_MASK 2

#define DE_PIN     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))   //port C7

uint16_t Rxphase;


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


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
                   sendByteUart1(1);
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
               displayUart0("device waiting for ack\n\r");
               //enter code on what the controller does between when it finishes requesting ACK at an address and before it starts requesting ack at (address + 1)
               DE_PIN = 0;
               checkBreak = true;

               //prepare the controller to start receiving
               while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
               UART1_IM_R &= ~0x20;                //disable the UART1 TX interrupt (if enabled)
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
   {                                        //0xABCDEF is the controller flag; determines that the mode is in controller mode
       if (pollMode && checkBreak && MODE == 0xABCDEF)               //if on polling mode and it is at the phase where the controller tries to receive an ACK
       {
           uint16_t data = UART1_DR_R;
           if (data & 0x400)  //if the device sent a break
           {
              displayUart0("found at ");
           }

           char text[20];
           sprintf(text, "address: %d \n\r", pollIndex);
           displayUart0(text);

           checkBreak = false;
           pollIndex++;

           //        //preparing the controller to send data again
           while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
           UART1_IM_R  &= ~0x10;                 //disable the UART1 RX interrupt

           GPIO_PORTB_AFSEL_R &= ~(R_MASK | D_MASK);  // *DO NOT* use peripheral to drive PB0 (UART1 TX and RX)
           GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M); // clear bits 0-3

           poll();
       }

       else             //if NOT in the controller mode where (controller is looking for ACK), resume normal functioning
                        //only device mode reaches to this part
       {
           uint16_t data = UART1_DR_R;

           if (data & 0x400)            //if break error occured in data register
           {
               initLEDPWM();
               Rxphase = 0;
               setLEDPWM(2, dataTable[devAddr]);
           }

           else                         //if not a break
           {
               dataTable[Rxphase] = (data & 0xFF);


               if (Rxphase == 0 && (dataTable[Rxphase] & 0xFF) == 0xF7)              //since while receiving, the value at index 0 is always the start code,
                   pollMode = true;                                                     //check at index 0 to see whether the controller is sending a break


               if(pollMode && (Rxphase == devAddr))//&& dataTable[devAddr] == 1)         //if the controller is on polling mode, then this receiver sees if it has received an ACK request from the controller
               {
//                              char text[50];
//                              sprintf(text, "address: %d, %d, %d\n\r",dataTable[devAddr-1], dataTable[devAddr], dataTable[devAddr+1] );
//                              displayUart0(text);


                   UART1_IM_R  &= ~0x10;                 //disable the UART1 RX interrupt/ stop receiving; prepare to acknowledge

                   phase = 0;           //transmit phase
                   initTimer1(16);
               }

               Rxphase++;

           }
       }

   }
}
