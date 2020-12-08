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

#ifndef UART1_H_
#define UART1_H_

extern uint16_t max;              //variable to hold the number of max devices on the bus; default value = 512
extern uint8_t dataTable[];          //table that holds the data to be sent to devices on the bus
extern uint16_t phase;
extern uint16_t devAddr;
extern bool ON;
extern uint32_t MODE;

extern uint16_t pollIndex;
extern bool pollMode;
extern bool checkBreak;

extern uint16_t pollFound[512];
extern void startDMX_TX();


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
extern void poll();

void initUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void sendByteUart1(uint8_t data);
uint8_t receiveByteUart1();

#endif
