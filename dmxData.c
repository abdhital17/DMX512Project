// CSE 4342 Project
// Abhishek Dhital

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


//__________________________________________________Includes___________________________________________________________________________


#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
#include "eeprom.h"
#include "uart1.h"
#include "timer1.h"



//__________________________________________________defines______________________________________________________________________________

#define MAX_CHARS 80
#define MAX_FIELDS 8
#define CONTROLLER_FLAG 0xABCDEF        //flag to specify that the system is running in controller mode
                                        //this flag is stored at EEPROM address 0x0  in controller mode

#define MODE_ADDRESS   0x0              // address in eeprom where the mode is specified to either controller/device
                                        // if the data in this address is 0xFFFFFFFF, device mode is configured
                                        //if data in this address is 0xABCDEF, controller mode is configured

#define DEVICE_ADDRESS_LOCATION 0x1       //address in eeprom where the device address is stored in device mode

//bitbanded aliases
#define D_PIN      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))   //port B1
#define DE_PIN     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))   //port C7


//Port C mask
#define DE_MASK 128

// PortB masks
#define R_MASK 1
#define D_MASK 2



//__________________________________________________Global Variables_______________________________________________________________________

 uint16_t max = 513;                //variable to hold the number of max devices on the bus; default value = 512
 uint8_t dataTable[513];            //table that holds the data to be sent to devices on the bus
 uint8_t hr, min, sec;            //variables to hold the hour, min and sec of the day
 uint8_t mth, day;                //variables to hold the month and day
 bool ON = false;                 //run boolean to specify whether DMX transmit is ON/OFF; set by the ON/OFF commands on UART0
 uint16_t phase;                  //variable to hold the value of phase ranging from 0 (break condition) to 514
 uint16_t devAddr = 1;            //variable to hold the address of the device when in Device Mode; Default value is set to 1
 uint32_t MODE;




//data structure to hold the incoming string through the UART0 Rx
typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;



//__________________________________________________subroutines_________________________________________________________________________________

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure GPIO for D pin in port B0 and DE pin in port C7
    GPIO_PORTB_DIR_R  |= D_MASK;
    GPIO_PORTB_DR2R_R |= D_MASK;
    GPIO_PORTB_DEN_R  |= D_MASK;

    GPIO_PORTC_DIR_R  |= DE_MASK;
    GPIO_PORTC_DR2R_R |= DE_MASK;
    GPIO_PORTC_DEN_R  |= DE_MASK;

}

void getsUart0(USER_DATA* d)
{
  uint8_t c=0; //counter variable
  char ch;
  while (1)  //loop starts
  {

    ch=getcUart0();
    if ((ch==8 || ch==127) && c>0) c--;

    else if (ch==13)
        {
         d->buffer[c]=0;
         return;
        }
    else if (ch>=32)
     {
        d->buffer[c]=ch;
        c++;
        if (c==MAX_CHARS)
        {
            d->buffer[c]='\0';
            return;
        }
     }
     else continue;
  }
}

void parseFields(USER_DATA* d)
{
    uint8_t i=0;
    char prev=0;
    d->fieldCount=0;
    while(d->buffer[i]!='\0')
    {
        if((d->fieldCount)>=MAX_FIELDS)
        {
            break;
        }

        char temp=d->buffer[i];

        if(((temp>=97 && temp<=122) || (temp>=65&&temp<=90)) && prev!='a' )
        {
            prev='a';
            d->fieldType[(d->fieldCount)]='a';
            d->fieldPosition[(d->fieldCount)]=i;
            d->fieldCount+=1;
        }

        else if ((temp>=48 && temp<=57) && prev!='n')
           {
                prev='n';
                d->fieldType[d->fieldCount]='n';
                d->fieldPosition[d->fieldCount]=i;
                d->fieldCount+=1;
            }
        else if(!((temp>=97 && temp<=122) || (temp>=65&&temp<=90)) && !(temp>=48 && temp<=57) )
           {
             prev=0;
             d->buffer[i]='\0';
           }
        i++;
   }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
  if(fieldNumber<=data->fieldCount)
      {
        return &(data->buffer[data->fieldPosition[fieldNumber]]);
      }
  else
      return NULL;
}

int32_t alphabetToInteger(char* numStr)
{
    int32_t num=0;
    while (*numStr != 0)
      {
        if(*numStr >= 48 && *numStr <= 57)
        {
              num = num*10 + ((*numStr) - 48);
              numStr++;
        }

      }
    return num;
}

bool stringCompare(const char* str1,const char* str2)
{
   bool equal = true;
   while(*str1 != 0 || *str2 != 0)
   {
       if((*str1 == 0 && *str2 != 0) || (*str1 != 0 && *str2 ==0))
           return false;

       if(!(*str1 == *str2 || (*str1 + 32) == *str2 || *str1 == (*str2+32) || (*str1 - 32) == *str2 || *str1 == (*str2 - 32)))
       {
           equal = false;
           break;
       }

       str1++;
       str2++;
   }
   return equal;
}


int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber<=data->fieldCount && data->fieldType[fieldNumber]=='n')
    {
        return alphabetToInteger(getFieldString(data, fieldNumber));
    }
    else
        return 0;
}


bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
 if(stringCompare(strCommand,getFieldString(data,0)) && (data->fieldCount) == (minArguments + 1))
     return true;
 return false;
}

void setData(uint16_t add, uint8_t data)
{
  dataTable[add] = data;
}

void getData(uint16_t add)
{

}

void setTime(uint8_t h, uint8_t m, uint8_t s)
{

}

void getTime(uint8_t* h, uint8_t* m, uint8_t* s)
{

}

void setDate(uint8_t m, uint8_t d)
{

}

void getDate(uint8_t* m, uint8_t* d)
{

}

void addTask(uint16_t a, uint8_t v, uint8_t h, uint8_t m, uint8_t s, uint8_t month, uint8_t d)
{

}

void controllerMode()
{
    writeEeprom(MODE_ADDRESS, CONTROLLER_FLAG);
    MODE = readEeprom(MODE_ADDRESS);

    ON = true;


    GPIO_PORTB_AFSEL_R &= ~(R_MASK);  // *DO NOT* use peripheral to drive PA1 (UART1 TX)
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M); // clear bits 0-3

    UART1_IM_R  &= ~0x10;                 //disable the UART1 RX interrupt

    startDMX_TX();
}

void deviceMode(uint16_t address)
{
    writeEeprom(MODE_ADDRESS, 0xFFFFFFFF);
    writeEeprom(DEVICE_ADDRESS_LOCATION, address);

    MODE = 0xFFFFFFFF;
    devAddr = address;

    GPIO_PORTB_AFSEL_R |= R_MASK;          // use peripheral to drive PA1 (UART1 TX)
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M); // clear bits 0-3
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB0_U1RX); // set bits 0-3 for UART1 RX control

    UART1_IM_R &= ~0x20;                //disable the UART1 TX interrupt (if enabled)
    UART1_IM_R  |= 0x10;                 //enable the UART1 RX interrupt

    ON = false;
}

void clear()
{
    uint16_t i =0;
    for (i=0; i < max; i++)
    {
        dataTable[i] = 0;
    }

}

void startDMX_TX()
{
    DE_PIN = 1;
    D_PIN  = 0;
    initTimer1(176);
    phase = 0;
}


int main(void)
{
    dataTable[0] = 0;               //start Code

    initHw();
    initUart0();
    initUart1();
    initEeprom();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    setUart1BaudRate(250000, 40e6);

//    displayUart0("\nABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\n\r");
//    displayUart0("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\n\r");

    MODE = readEeprom(MODE_ADDRESS);
    if ( MODE == 0xFFFFFFFF)
    {
        devAddr = readEeprom(DEVICE_ADDRESS_LOCATION);

        if(devAddr == 0xFFFFFFFF)
            devAddr = 1;
        GPIO_PORTB_AFSEL_R |= R_MASK;          // use peripheral to drive PA1 (UART1 TX)
        GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M); // clear bits 0-3
        GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB0_U1RX); // set bits 0-3 for UART1 RX control

        UART1_IM_R  |= 0x10;                 //enable the UART1 RX interrupt
    }

    else
    {
        ON = true;
        startDMX_TX();
    }


    USER_DATA data;


    while(1)
    {
   // putsUart0("\r");
    getsUart0(&data);
    displayUart0(data.buffer);
    displayUart0("\n\r");
    parseFields(&data);
    bool valid=false;

    if(isCommand(&data,"set", 2))
    {
       setData(getFieldInteger(&data, 1), getFieldInteger(&data, 2));

       valid = true;
    }

    else if (isCommand(&data, "get", 1))
    {
        getData(getFieldInteger(&data, 1));
        valid = true;
    }

    else if (isCommand(&data, "clear", 0))
    {
        clear();
        valid = true;
    }

    else if (isCommand(&data, "max", 1))
    {
        max = getFieldInteger(&data, 1);
        valid = true;
    }

    else if (isCommand(&data, "on", 0))
    {
        ON = true;
        startDMX_TX();

        valid = true;
    }

    else if(isCommand(&data, "off", 0))
    {
        ON = false;
        valid = true;
    }

    else if(isCommand(&data, "poll", 0))
    {
        valid = true;
    }

    else if(isCommand(&data, "time", 3))
    {
        setTime(getFieldInteger(&data, 1), getFieldInteger(&data, 2), getFieldInteger(&data, 3));
        valid = true;
    }

    else if(isCommand(&data, "time", 0))
    {
        char text[50];
        getTime(&hr, &min, &sec);

        sprintf(text, "time %d:%d:%d\n\r", hr, min, sec);
        displayUart0(text);
        valid = true;
    }

    else if(isCommand(&data, "date", 2))
    {
        setDate(getFieldInteger(&data, 1), getFieldInteger(&data, 2));
        valid = true;
    }

    else if(isCommand(&data, "date", 0))
    {
        char text[50];

        getDate(&mth, &day);

        sprintf(text, "month: %d day: %d\n\r", mth, day);
        displayUart0(text);
        valid = true;
    }

    else if(isCommand(&data, "setat", 7))
    {
        addTask(getFieldInteger(&data, 1), getFieldInteger(&data, 2), getFieldInteger(&data, 3), getFieldInteger(&data, 4),
                getFieldInteger(&data, 5), getFieldInteger(&data, 6), getFieldInteger(&data, 7));

        valid = true;
    }

    else if(isCommand(&data, "controller", 0))
    {
        controllerMode();
        valid = true;
    }

    else if(isCommand(&data, "device", 1))
    {
        deviceMode(getFieldInteger(&data, 1));
        valid = true;

    }


    if(!valid)
        displayUart0("Invalid command\n\r");

    }

    return 0;
}
