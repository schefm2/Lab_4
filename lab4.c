/* Sample code for speed control using PWM. */
#include <stdio.h>
#include <c8051_SDCC.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// 8051 Initialization Functions
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init(void);
void Interrupt_Init(void);
void ADC_Init();						//Initialize the ADC converter
void SMB_Init();

//other functions
void Min_Max(void);
void LED_Brightness(void);
unsigned int ReadRanger();
void PingRanger();

unsigned int PW_MIN =1000;
unsigned int PW_MAX =36700;
unsigned int PW_NEUT =18432;
unsigned char addr=0xE0; // the address of the ranger is 0xE0
unsigned char Data[2];
unsigned char r_count=0;
unsigned char r_check=0;

//sbits
__sbit __at 0xB5 SS; //Port 3.5 slideswitch

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
	// initialize board
	Sys_Init();
	putchar(' '); //the quotes in this line may not format correctly
	Port_Init();
	XBR0_Init();
	Interrupt_Init();
	PCA_Init();
	SMB_Init();
	ADC_Init();
}

void ADC_Init(void)
{
	REF0CN = 0x03;	//Sets V_ref as 2.4V
	ADC1CN = 0x80;	//Enables AD/C converter

	//Gives capacitors in A/D converter time to charge
	r_check=r_count; //makes sure r_count isn't altered while waiting 
	while(r_count=<(r_check+2));
	
	//Resets timer0 and overflow counter
	TMR0 = 0;
	T0_overflows = 0;

	//Sets gain to 1
	ADC1CF |= 0x01;
	ADC1CF &= 0xFD;
}

void Port_Init()
{
	P1MDOUT |= 0x03;//set output pin for CEX2 in push-pull mode
	P3MDOUT &= 0x20; //Pin 3.5 open drain
	P3 |= 0x20; //Pin 3.5 high impedance
}
//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Interrupt_Init(void)
{
	// IE and EIE1
	EA=1;
	EIE1 |= 0x08;
}
//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init(void)
{
	XBR0 = 0x27; //configure crossbar with UART, SPI, SMBus, and CEX channels as
	// in worksheet
}
//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
	// reference to the sample code in Example 4.5 - Pulse Width Modulation implemented using
	// Use a 16 bit counter with SYSCLK/12.
	PCA0MD = 0x81;
	PCA0CPM3 = 0xC2;
	PCA0CN = 0x40; //Enable PCA counter
}
//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9
{
	if(CF)
	{
		CF=0; //clear flag
		PCA0 = 28672;//determine period to 20 ms
		r_count++;
	}
	PCA0CN &= 0x40; //Handle other interupt sources
// reference to the sample code in Example 4.5 -Pulse Width Modulation implemented using
}
//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Set up the I2C Bus
//
void SMB_Init()
{
	SMB0CR = 0x93;	//Sets SCL to 100 kHz (actually ~94594 Hz)
	ENSMB = 1;		//Enables SMB
}
//-----------------------------------------------------------------------------