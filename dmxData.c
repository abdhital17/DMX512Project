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



//on board LED
#define RED_LED   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  //PF1
#define BLUE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))  //PF2
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  //PF3

//LED MASKS
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

//LED counts
uint8_t RED_TIMEOUT_OFF;


//Port C mask
#define DE_MASK 128

// PortB masks
#define R_MASK 1
#define D_MASK 2



//__________________________________________________Global Variables_______________________________________________________________________

 uint16_t max = 514;                //variable to hold the number of max devices on the bus; default value = 512
 uint8_t dataTable[513];            //table that holds the data to be sent to devices on the bus;
                                    //(receive mode contains MAB on index 0) so need an extra index to hold the last value transmitted

// uint8_t hr, min, sec;              //variables to hold the hour, min and sec of the day
// uint8_t mth, day;                  //variables to hold the month and day
 bool ON = false;                   //run boolean to specify whether DMX transmit is ON/OFF; set by the ON/OFF commands on UART0
 uint16_t phase;                    //variable to hold the value of phase ranging from 0 (break condition) to 514
 uint16_t devAddr = 1;              //variable to hold the address of the device when in Device Mode; Default value is set to 1
 uint32_t MODE;


 //poll variables
bool pollMode = false;
uint8_t startCode = 0;              //0x00 for transmission mode; 0xF7 for polling mode
uint16_t pollIndex = 0;
bool checkBreak = false;            //true when the controller releases the bus to receive an ack from the devices while polling

//data structure to hold the incoming string through the UART0 Rx
typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;


#define MAX_SET 5
uint32_t setAtTable [MAX_SET][3] = {0};

//__________________________________________________subroutines_________________________________________________________________________________

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCTIMER_R |=SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    //configure the on board LEDs
    GPIO_PORTF_DIR_R  |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DR2R_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DEN_R  |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    // Configure GPIO for D pin in port B0 and DE pin in port C7
    GPIO_PORTB_DIR_R  |= D_MASK;
    GPIO_PORTB_DR2R_R |= D_MASK;
    GPIO_PORTB_DEN_R  |= D_MASK;

    GPIO_PORTC_DIR_R  |= DE_MASK;
    GPIO_PORTC_DR2R_R |= DE_MASK;
    GPIO_PORTC_DEN_R  |= DE_MASK;


    // Configure Timer 0 as the time base
//    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
//    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
//    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for one shot mode (count down)
//
//    TIMER0_TAILR_R = 4000000;
//    TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
//    NVIC_EN0_R |= 1 << (INT_TIMER0A-16);             // turn-on interrupt 37 (TIMER1A)
//    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void HIB_INIT()
{
    //Configure the Hibernation module

     while (!(HIB_CTL_R & 0x80000000));
     HIB_CTL_R &= ~0x1;


     while (!(HIB_CTL_R & 0x80000000));
     HIB_RTCM0_R = 0xFFFFFFFF;

     while (!(HIB_CTL_R & 0x80000000));
     HIB_RTCLD_R = 0;

     while (!(HIB_CTL_R & 0x80000000));
     HIB_IM_R |= 0x1;
     NVIC_EN1_R |= 1 << (INT_HIBERNATE - 16 - 32);

     while (!(HIB_CTL_R & 0x80000000));
     HIB_CTL_R |= 0x00000041;

}

void alarmISR()
{
    while (!(HIB_CTL_R & 0x80000000));
    HIB_IC_R |= 0x1;

    setData(setAtTable[0][1], setAtTable[0][2]);

    uint8_t i = 1;

    while(i < MAX_SET)
    {
      setAtTable[i - 1][0] = setAtTable[i][0];
      setAtTable[i - 1][1] = setAtTable[i][1];
      setAtTable[i - 1][2] = setAtTable[i][2];
      i++;
    }

    while (!(HIB_CTL_R & 0x80000000));
    HIB_RTCM0_R = setAtTable[0][0];
}

void timer0ISR()
{
    //if (RED_TIMEOUT)
//    if(RED_TIMEOUT)
//    {
//
//    }
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
        //return alphabetToInteger(getFieldString(data, fieldNumber));
        return atoi(getFieldString(data, fieldNumber));
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
    char text[50];
    sprintf(text,"%d\n\r",dataTable[add]);
    displayUart0(text);
}

void setTime(uint8_t h, uint8_t m, uint8_t s)
{
    uint32_t raw_time = h*3600 + m * 60 + s;

    HIB_RTCLD_R = raw_time;
}

void getTime(uint8_t* h, uint8_t* m, uint8_t* s)
{
    uint32_t rawTime = HIB_RTCC_R;

    rawTime = rawTime%(86400 * 30);
    rawTime = rawTime%(86400);

    *h = rawTime/3600;
    rawTime -= *h * 3600 ;

    *m = rawTime/60;
    rawTime -= *m * 60;

    *s = rawTime;
}

void setDate(uint8_t m, uint16_t d)
{
    uint32_t rawDate = 86400*(m*30 + d);
    HIB_RTCLD_R += rawDate;
}

void getDate(uint8_t* month, uint16_t* day)
{
    uint32_t rawDate = HIB_RTCC_R;

    *month = rawDate / (86400 * 30);

    *day = (rawDate % (86400 * 30)) / 86400;
}

void addTask(uint16_t a, uint8_t v, uint8_t h, uint8_t m, uint8_t s, uint8_t month, uint16_t d)
{
    //setAtTable
    uint32_t raw_time = s + m*60 + h*3600;
    raw_time = raw_time + (month * 30 + d) * 86400;

    uint8_t i = 0;
    bool found = false;
    for(i = 0; i<MAX_SET; i++)
    {
        if (setAtTable[0][0] == 0)
        {
           setAtTable[i][0] = raw_time;
           setAtTable[i][1] = a;
           setAtTable[i][2] = v;

           found = true;
           //HIB_RTCM0_R = raw_time;
           break;
        }

        if(raw_time < setAtTable[i][0])
        {
            uint32_t temp11, temp12, temp13;
            temp11 = setAtTable[i][0];
            temp12 = setAtTable[i][1];
            temp13 = setAtTable[i][2];

            setAtTable[i][0] = raw_time;
            setAtTable[i][1] = a;
            setAtTable[i][2] = v;

            i++;
            while(i < MAX_SET)
            {
                uint32_t temp21, temp22, temp23;

                temp21 = setAtTable[i][0];
                temp22 = setAtTable[i][1];
                temp23 = setAtTable[i][2];

                setAtTable[i][0] = temp11;
                setAtTable[i][1] = temp12;
                setAtTable[i][2] = temp13;

                temp11 = temp21;
                temp12 = temp22;
                temp13 = temp23;
                i++;
            }

            found = true;
            break;
        }

        else
        {
            if(setAtTable[i][0] == 0)
            {
                setAtTable[i][0] = raw_time;
                setAtTable[i][1] = a;
                setAtTable[i][2] = v;
                found = true;
                break;
            }

            else if (setAtTable[i][0] == raw_time)
            {
              setAtTable[i][0] = raw_time;
              setAtTable[i][1] = a;
              setAtTable[i][2] = v;
              found = true;
              break;
            }
        }
    }

    if(!found)
    {
        displayUart0("table is full \n\r");
    }


    while (!(HIB_CTL_R & 0x80000000));
    HIB_CTL_R &= ~0x1;
    while (!(HIB_CTL_R & 0x80000000));
    HIB_RTCM0_R = setAtTable[0][0];
    while (!(HIB_CTL_R & 0x80000000));
    HIB_CTL_R |= 0x00000041;
}

void controllerMode()
{
    ON = true;

    writeEeprom(MODE_ADDRESS, CONTROLLER_FLAG);
    MODE = readEeprom(MODE_ADDRESS);

    while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
    UART1_IM_R  &= ~0x10;                 //disable the UART1 RX interrupt

    GPIO_PORTB_AFSEL_R &= ~(R_MASK | D_MASK);  // *DO NOT* use peripheral to drive PB0 (UART1 TX)
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M); // clear bits 0-3

    startDMX_TX();
}

void deviceMode(uint16_t address)
{
    ON = false;

    writeEeprom(MODE_ADDRESS, 0xFFFFFFFF);
    writeEeprom(DEVICE_ADDRESS_LOCATION, address);
    MODE = 0xFFFFFFFF;
    devAddr = address;

    while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
    UART1_IM_R &= ~0x20;                //disable the UART1 TX interrupt (if enabled)
    GPIO_PORTB_AFSEL_R &= ~(D_MASK);  // *DO NOT* use peripheral to drive PA1 (UART1 TX)
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M); // clear bits 4-7

    GPIO_PORTB_AFSEL_R |= R_MASK;          // use peripheral to drive PA1 (UART1 TX)
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M); // clear bits 0-3
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB0_U1RX); // set bits 0-3 for UART1 RX control

    UART1_IM_R  |= 0x10;                 //enable the UART1 RX interrupt

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
    startCode = 0;
    RED_LED = 1;
    DE_PIN = 1;
    D_PIN  = 0;
    initTimer1(176);
    phase = 0;
}

void poll()
{
    if(pollIndex > 512)
    {
        displayUart0("polling devices completed\n\r");
        displayUart0("controller starting to send data again\n\rAlso remember to change the max value since it has been changed to 512 for polling\n\r");

//        //preparing the controller to send data again
        while (UART1_FR_R & UART_FR_BUSY);                  // wait if uart1 tx fifo busy
        UART1_IM_R  &= ~0x10;                 //disable the UART1 RX interrupt

        GPIO_PORTB_AFSEL_R &= ~(R_MASK | D_MASK);  // *DO NOT* use peripheral to drive PB0 (UART1 TX and RX)
        GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB0_M | GPIO_PCTL_PB1_M); // clear bits 0-3

        pollMode = false;
        checkBreak = false;
        ON = true;
        startDMX_TX();
    }

    else
    {
        ON = false;     //not transmitting untill poll is done
        pollMode = true;    //stays true until the last address is polled
        checkBreak = false; //true only when the controller is trying to receive an ack from the device

        phase = 0;
        startCode = 0xF7;
        DE_PIN = 1;
//        while(phase != 0);          //waiting for the last transmission to end
        initTimer1(176);
    }
}




int main(void)
{
    initHw();
    initUart0();
    initUart1();
    initEeprom();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    setUart1BaudRate(250000, 40e6);

    HIB_INIT();

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

    else if(isCommand(&data, "address", 0))
    {
       char text[20];
       sprintf(text, "address: %d\n\r", devAddr);
       displayUart0(text);
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
        max = getFieldInteger(&data, 1) + 2;
        valid = true;
    }

    else if (isCommand(&data, "on", 0))
    {
        if(MODE == CONTROLLER_FLAG)
        {
            ON = true;
            startDMX_TX();
        }

        else
        {
            displayUart0("running on device mode. Can not turn on transmission\n\r");
        }
        valid = true;
    }

    else if(isCommand(&data, "off", 0))
    {
        ON = false;
        RED_LED = 0;
        valid = true;
    }

    else if(isCommand(&data, "poll", 0))
    {
        max = 514;                  //need to poll at all addresses to complete the polling properly; so whatever teh
        displayUart0("Starting to poll devices on the bus\n\r");
        pollIndex = 0;
        pollMode = true;
        poll();
        valid = true;
    }

    else if(isCommand(&data, "time", 3))
    {
        setTime(getFieldInteger(&data, 1), getFieldInteger(&data, 2), getFieldInteger(&data, 3));
        valid = true;

        displayUart0("\n\rtime set!\n\rset date now..\n\r");
    }

    else if(isCommand(&data, "time", 0))
    {
        uint8_t hr, min, sec;              //variables to hold the hour, min and sec of the day


        getTime(&hr, &min, &sec);

        char text[50];
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
        uint8_t mth;                  //variables to hold the month and day
        uint16_t day;

        getDate(&mth, &day);

        sprintf(text, "month: %d day: %d\n\r", mth, day);
        displayUart0(text);
        valid = true;
    }

    else if(isCommand(&data, "setat", 7))
    {
        addTask(getFieldInteger(&data, 1), getFieldInteger(&data, 2), getFieldInteger(&data, 3), getFieldInteger(&data, 4),
                getFieldInteger(&data, 5), getFieldInteger(&data, 6), getFieldInteger(&data, 7));

        char text[50];
        uint8_t i = 0;
        for(i=0; i< MAX_SET; i++)
        {
            sprintf(text, "A:%d  B:%d C:%d \n\r", setAtTable[i][0], setAtTable[i][1], setAtTable[i][2]);
            displayUart0(text);
        }

        valid = true;
    }

    else if(isCommand(&data, "setat", 5))
    {
        addTask(getFieldInteger(&data, 1), getFieldInteger(&data, 2), getFieldInteger(&data, 3), getFieldInteger(&data, 4),
                getFieldInteger(&data, 5), 0 , 0);
        valid = true;
    }

    else if(isCommand(&data, "controller", 0))
    {
        if (MODE == 0xFFFFFFFF)
            controllerMode();

        else if (MODE == CONTROLLER_FLAG)
            displayUart0("Already in controller mode \n\r");

        valid = true;
    }

    else if(isCommand(&data, "device", 1))
    {
        if (MODE == 0xFFFFFFFF)
        {
            uint16_t tempAddr;
            tempAddr = getFieldInteger(&data,1);

            if (tempAddr <= 512 && tempAddr > 0)
            {
                devAddr = getFieldInteger(&data, 1);
                writeEeprom(DEVICE_ADDRESS_LOCATION, devAddr);
                displayUart0("Already in device mode. Only updating the address\n\r");
            }

            else
                displayUart0("Enter an address between 1 to 512.\n\r");
            valid = true;
        }


        else
        {
            deviceMode(getFieldInteger(&data, 1));
            displayUart0("Device mode activated \n\r");
        }

        valid = true;

    }


    if(!valid)
        displayUart0("Invalid command\n\r");

    }

    return 0;
}
