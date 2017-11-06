/* Sample code for speed control using PWM. */
#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

#define ON 1
#define OFF 0

#define RANGER_ADDR 0xE0
#define COMPASS_ADDR 0xC0
#define PING_CM 0x51

#define PCA_START 28672

#define SERVO_LEFT_PW 2395
#define SERVO_CENTER_PW 2825
#define SERVO_RIGHT_PW 3185

#define MOTOR_REVERSE_PW 2027 
#define MOTOR_NEUTRAL_PW 2765
#define MOTOR_FORWARD_PW 3502

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
void Read_Compass(void);
void Read_Ranger(void);
void Read_Compass(void);
void Read_Ranger(void);
void Set_Servo_PWM(void);
void Set_Motor_PWM(void);
void Pause(void);


void Car_Parameters(void);
void Set_Motion(void);
void Set_Neutral(void);
void Print_Data(void);

unsigned char addr=0xE0; // the address of the ranger is 0xE0
unsigned char Data[2];
unsigned char r_count=0;
unsigned char r_check=0;
unsigned int PCA_overflows, desired_heading, current_heading, heading_error, initial_speed, range, Servo_PW, Motor_PW
unsigned char nestedCount, readCount, compassFlag, rangerFlag, keyboard, keypad


//sbits
__sbit __at 0xB7 SS; //Port 3.5 slideswitch run/stop
__sbit __at 0x97 POT; //Port 1.7 potentiometer

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
/*
	Set PCA_overflows to 0
	Wait while PCA_overflows < 50
	
	Car_Parameters();
	
	Begin infinite loop
		Set_Motion();
		Set_Neutral();
		Print_Data();
	End infinite loop

	a main function that calls a read_compass()
	function and sets the PWM for the steering servo based on the present heading and a desired
	compass heading. 

	The main code also calls a ranger function and adjusts the desired heading
	and/or drive motor based on SecureCRT inputs when the measurement from the ultrasonic
	sensor to an obstacle is less than a set value. 

	

	Display relevant values or messages on the LCD display. Once the desired heading and gains are
	selected, the LCD should display the current heading, the current range and optionally the battery 
	voltage. Updating the display every 400 ms or longer is reasonable. Updating more frequently is not
	needed and should be avoided.



	Tabulate the data from the compass heading error & servo motor PW value, and
	transmit it to the SecureCRT terminal for filing and later plotting
*/

}
//HIGH LEVEL FUNCTIONS
//----------------------------------------------------------------------------
//Car_Parameters
//----------------------------------------------------------------------------
void Car_Parameters(void)
{
	/*
	Allow user to enter initial speed, desired heading, and other parameters.  This can be done by
	pressing keys on the keyboard or keypad. The system should allow the operator to enter a specific
	desired angle, or pick from a predefined list of 0°, 90°, 180°, 270°. If the user is to enter a
	desired angle, the SecureCRT or LCD screen should indicate so with a prompt. If the user is to 
	choose an angle from a predefined list, the screen should show the list along with the key press 
	needed to select that angle. The steering gain and drive gain (only for obstacle tracking, not used 
	here) must also be selectable. 
	Configure the A/D converter in the C8051 to read a potentiometer voltage. As mentioned previously, 
	the potentiometer is used to select the gain. This is set once with the other initializations. The
	final value must be displayed for the user to see, and allowing the user to make adjustments until
	a desired value is set is a nice feature.

	Set Servo_PW to SERVO_CENTER_PW
		Set Motor_PW to MOTOR_NEUTRAL_PW
		Print instructions to select cardinal direction with keypad or computer keyboard
		Begin infinite loop
			Set keyboard to getchar_nw()
			Set keypad to read_keypad()
			Pause()
			If keypad is '5'
				break
			if keyboard is 'a'
				break
			If keypad is not 0xFF
				print out selected keypad direction and ask for confirmation of direction
			If keyboard is not 0xFF
				print out selected keyboard direction and ask for confirmation of direction
		End infinite loop
		If keypad is '2' OR keyboard is 'n'
			Set desired_heading to 0
		If keypad is '6' OR keyboard is 'e'
			Set desired_heading to 900
		If keypad is '8' OR keyboard is 's'
			Set desired_heading to 1800
		If keypad is '4' OR keyboard is 'w'
			Set desired_heading to 2700
	*/
}
//----------------------------------------------------------------------------
//Set_Motion
//----------------------------------------------------------------------------
void Set_Motion(void)
{
	Read_Compass();
	Read_Ranger();
	Set_Servo_PWM();
	Set_Motor_PWM();
}
//----------------------------------------------------------------------------
//Set_Neutral
//----------------------------------------------------------------------------
void Set_Neutral(void)
{
	/*
	If SS is OFF
			Set Servo_PW to SERVO_CENTER_PW
			Set Motor_PW to MOTOR_NEUTRAL_PW
			Wait while (SS is OFF)
	*/
}
//----------------------------------------------------------------------------
//Print_Data
//----------------------------------------------------------------------------
void Print_Data(void)
{
	/*
	
	Tabulate the data from the compass heading error & servo motor PW value, and
	transmit it to the SecureCRT terminal for filing and later plotting

	If readCount > 25
			Set readCount to 0
			print heading_error and Motor_PW
	*/
}
//LOW LEVEL FUNCTIONS
//----------------------------------------------------------------------------
//Read_Compass
//----------------------------------------------------------------------------
void Read_Compass(void)
{
	/*
	If r_count mod 2
			i2c_read_data(COMPASS_ADDR, 2, Data, 2)
			Set current heading to Data with bit shifting
			Increment readCount
	*/
}
//----------------------------------------------------------------------------
//Read_Ranger
//----------------------------------------------------------------------------
void Read_Ranger(void)
{
	/*
	If r_count mod 4
			r_count=0
			i2c_read_data(RANGER_ADDR, 2, Data, 2)
			Set range to Data with bit shifting
			Set Data[0] to PING_CM
			i2c_write_data (RANGER_ADDR, 0, Data, 1)
			Set Data to 0
	*/
}
//----------------------------------------------------------------------------
//Set_Servo_PWM
//----------------------------------------------------------------------------
void Set_Servo_PWM(void)
{

}
//----------------------------------------------------------------------------
//Set_Motor_PWM
//----------------------------------------------------------------------------
void Set_Motor_PWM(void)
{

}
//----------------------------------------------------------------------------
//Pause
//----------------------------------------------------------------------------
void Pause(void)
{

}
//----------------------------------------------------------------------------
//ADC_Init
//----------------------------------------------------------------------------
void ADC_Init(void)
{
	REF0CN = 0x03;	//Sets V_ref as 2.4V
	ADC1CN = 0x80;	//Enables AD/C converter

	//Gives capacitors in A/D converter time to charge
	r_check=r_count; //makes sure r_count isn't altered while waiting 
	while(r_count=<(r_check+2));

	//Sets gain to 1
	ADC1CF |= 0x01;
	ADC1CF &= 0xFD;
}
//-----------------------------------------------------------------------------
//Port_Init
//-----------------------------------------------------------------------------
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