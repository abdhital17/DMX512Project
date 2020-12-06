//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef TIMER1_H_
#define TIMER1_H_

#include <stdint.h>
#include <stdbool.h>
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

extern uint16_t phase;
extern uint8_t startCode;
extern bool pollMode;
extern uint32_t MODE;
void initTimer1(uint32_t time);

#endif
