/*
* This program is essentially a copy of "kpdlcdtestPCA.c". I am attempting to
* make LCD keypad and computer keyboard be used in parallel for data input.
* Hopefully this will be implemented in Lab 4.
*
*
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h> // Include files. This file is available online in LMS
#include <i2c.h>        // Get from LMS, THIS MUST BE INCLUDED AFTER stdio.h
#define PCA_START 28672 // 28672 for exactly 20ms
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);   // Initialize ports for input and output
void Interrupt_Init(void);
void PCA_Init(void);
void SMB0_Init(void);
void PCA_ISR(void) __interrupt 9;
unsigned int parallel_input(void);
void wait(void);
void pause(void);
unsigned int pow(unsigned int a, unsigned char b);

// Global variables
unsigned int Counts, nCounts, nOverflows, printCount;
unsigned int finalNumber;

//*****************************************************************************
void main(void)
{
    Sys_Init();     // System Initialization - MUST BE 1st EXECUTABLE STATEMENT
    Port_Init();    // Initialize ports 2 and 3 - XBR0 set to 0x05, UART0 & SMB
    Interrupt_Init();   // You may want to change XBR0 to match your SMB wiring
    PCA_Init();
    SMB0_Init();
    putchar('\r');  // Dummy write to serial port
    printf("\nStart");

	printCount = 0;
    Counts = 0;
    while (Counts < 1); // Wait a long time (1s) for keypad & LCD to initialize

    lcd_clear();
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");
	
	wait();
	
    while (1)
    {	
		printf("\r\nSelect a desired heading by inputing 5 digits. Press # to confirm.\r\n");
		lcd_clear();
		lcd_print("Press 5 keys.\n");
		
		finalNumber = parallel_input();
		printf("\r\nYou selected %d as your heading", finalNumber);
		wait();		//Waits to allow the input number a chance to be displayed
		wait();
    }
}
//*****************************************************************************

//-----------------------------------------------------------------------------
// parallel_input
//-----------------------------------------------------------------------------
//
// Waits for a user to press 5 keys and confirm with a # key. The following 5 
// digit number is then returned.
//
unsigned int parallel_input(void)
{
	unsigned char keypad;
	unsigned char keyboard;
	unsigned char isPress = 0;
	unsigned char pressCheck = 0;
	unsigned int value = 0;
	
	while(1)
	{
		keyboard = getchar_nw();	//This constantly sets keyboard to whatever char is in the terminal
		keypad = read_keypad();		//This constantly sets the keypad to whatever char is on the LCD
		pause();					//Pause necessary to prevent overreading the keypad
		
		if (keyboard == '#' || keypad == '#') //# is a confirm key, so it will finish parallel_input()
			return value;
		
		if (isPress > pressCheck && keypad == 0xFF && keyboard == 0xFF)	//Only increments pressCheck if held key is released
			pressCheck++;
		

		if (pressCheck == 6)	//If a 6th key is pressed, then released
		{
			isPress = pressCheck = 0;	//Reset the flags
			value = 0;	//Reset return value
			lcd_print("\b\b\b\b\b\b");	//Clear value displayed on LCD, needs an extra \b for some reason?
			printf("\r      \r");	//Clear value displayed on terminal
			
		}

		
		if (isPress == pressCheck)	//pressCheck must be equal to isPress, only occurs if no key is held down
		{
			if (keypad != 0xFF)		//When an actual key is held down
			{
				lcd_print("%c",keypad);	//Adds pressed key to LCD screen
				printf("%c", keypad);	//Adds pressed key to computer terminal
				value = value + ((unsigned int)(keypad - '0')) * pow(10,4 - isPress);	//Essentially takes each pressed key and multiples by some power of 10
				isPress++;
			}
			if (keyboard != 0xFF)	//When an actual key is held down
			{
				lcd_print("%c",keyboard);	//Adds pressed key to LCD screen
				//printf("%c", keyboard); this line is not necessary as getchar_nw automatically executes a putchar()
				value = value + ((unsigned int)(keyboard - '0')) * pow(10,4 - isPress);	//Essentially takes each pressed key and multiples by some power of 10
				isPress++;	
			}
		}
	}
}




void Port_Init(void)	//0x05
{					// NOTE on NOTE: the crossbar is changed to 0x27 to allow
					// for compatibility with the Lab 3 circuit.
    XBR0 = 0x27;    // NOTE: Only UART0 & SMB enabled; SMB on P0.2 & P0.3
}                   // No CEXn are used; no ports to initialize

void Interrupt_Init(void)
{
    IE |= 0x02;
    EIE1 |= 0x08;
    EA = 1;
}

void PCA_Init(void)
{
    PCA0MD = 0x81;      // SYSCLK/12, enable CF interrupts, suspend when idle
//  PCA0CPMn = 0xC2;    // 16 bit, enable compare, enable PWM; NOT USED HERE
    PCA0CN |= 0x40;     // enable PCA
}

void SMB0_Init(void)    // This was at the top, moved it here to call wait()
{
    SMB0CR = 0x93;      // Set SCL to 100KHz
    ENSMB = 1;          // Enable SMBUS0
}

void PCA_ISR(void) __interrupt 9
{
    if (CF)
    {
        CF = 0;                     // clear the interrupt flag
        nOverflows++;               // continuous overflow counter
        nCounts++;
        PCA0L = PCA_START & 0xFF;   // low byte of start count
        PCA0H = PCA_START >> 8;     // high byte of start count
        if (nCounts > 50)
        {
            nCounts = 0;
            Counts++;               // seconds counter
        }
     }
     else PCA0CN &= 0xC0;           // clear all other 9-type interrupts
}

void pause(void)
{
    nCounts = 0;
    while (nCounts < 1);// 1 count -> (65536-PCA_START) x 12/22118400 = 20ms
}                       // 6 counts avoids most of the repeated hits

void wait(void)
{
    nCounts = 0;
    while (nCounts < 50);    // 50 counts -> 50 x 20ms = 1000ms
}
//-----------------------------------------------------------------------------
// pow
//-----------------------------------------------------------------------------
//
// Function that raises a to the power of b
//
unsigned int pow(unsigned int a, unsigned char b)
{
	unsigned char i;
	unsigned char base = a;

	if (b == 0) return 1;
	for(i = 1; i < b; i++)
		a = a*base;
	return a;
}