//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// PWM Interface:
//   M1PWM5 (PF1), M1PWM6 (PF2) and M1PWM7 (PF3) are connected to the 2nd controller

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef PWM_H_
#define PWM_H_


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initLEDPWM();
void setLEDPWM(uint8_t rgb, uint8_t cmpb);


#endif
