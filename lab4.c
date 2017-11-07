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
unsigned int PCA_overflows, desired_heading, current_heading, heading_error, initial_speed, range, Servo_PW, Motor_PW;
unsigned char r_count, r_check, keyboard, keypad;
float time;

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
	*/
	unsigned char isPress = 0;
	unsigned char pressCheck = 0;
	unsigned int finalValue = 0;

	Servo_PW = SERVO_CENTER_PW;		//Initialize car to straight steering and no movement
	Motor_PW = MOTOR_NEUTRAL_PW;
	
	lcd_clear();
	lcd_print("Compass direction\nWith even keys.");
	printf("Please select a cardinal direction. North is 8, East is 6, South is 2, West is 4.\r\n");
	while (1)
	{
		keyboard = getchar_nw();	//Equal to 0xFF when no key is pressed
		keypad = read_keypad();		//Equal to 0xFF when no key is pressed
		Pause();	//Pauses program to prevent rapid consecutive keypad reads
		
		if (keypad == '5' || keyboard == '5')	//5 key is used as the "enter" key
		{
			isPress = 0;
			break;
		}
		
		if (isPress == 1 && keypad == 0xFF && keyboard == 0xFF)	//When a key was previously held down but is currently released
			isPress = 0;
		
		if (keypad != 0xFF && isPress == 0)	//Checks to see if a key is currently pressed and no key is held down
		{
			isPress = 1;	//Sets isPress high when a key is held down
			finalValue = keypad;
			lcd_clear();
			lcd_print("You select %c\nIf sure press 5", keypad);
		}
		
		if (keyboard != 0xFF && isPress == 0)	//Checks to see if a key is currently pressed and no key is held down
		{
			isPress = 1;	//Sets isPress high when a key is held down
			finalValue = keyboard;
			printf("You selected %c as your desired heading. Press 5 to confirm.\r\n", keyboard);
		}
	}
	if (finalValue == '8')
		desired_heading = 0;	//Sets heading to North
	if (finalValue == '6')
		desired_heading = 900;	//Sets heading to East
	if (finalValue == '2')
		desired_heading = 1800;	//Sets heading to South
	if (finalValue == '4')
		desired_heading = 2700;	//Sets heading to West
	printf("Your desired heading has been set to %d\r\n", desired_heading);
	lcd_clear();
	lcd_print("You Selected %d\nFor heading", desired_heading);
	
	
	printf("Set initial speed by keying in 4 numbers.\r\n");
	lcd_clear();
	lcd_print("Key 4 nums for \ninitial speed\n");
	while(1)
	{
		keyboard = getchar_nw();	//Equal to 0xFF when no key is pressed
		keypad = read_keypad();		//Equal to 0xFF when no key is pressed
		Pause();	//Pauses program to prevent rapid consecutive keypad reads
		
		if (isPress > pressCheck && keypad == 0xFF && keyboard == 0xFF)
			pressCheck++;
		
		if (pressCheck == 4)
		{
			printf("\r\nYou have selected a speed of %d. Press * to enter", finalValue);
			lcd_clear();
			lcd_print("Speed is %d\nPress # confirm");
			isPress = pressCheck = 0;
		}
		
		if (keypad == '#' || keyboard == '#')
			break;
		
		if (keypad != 0xFF && isPress == pressCheck)
		{
			lcd_print("%c",keypad)
			finalValue = finalValue + (unsigned int)keypad * 10^isPress;
			isPress++;
		}
		if (keyboard != 0xFF && isPress == pressCheck)
		{
			printf("%c", keyboard);
			finalValue = finalValue + (unsigned int)keyboard * 10^isPress;
			isPress++;
		}
	}
	
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
	Once the desired heading and gains are
	selected, the LCD should display the current heading, the current range and optionally the battery 
	voltage. Updating the display every 400 ms or longer is reasonable. Updating more frequently is not
	needed and should be avoided.
	
	Tabulate the data from the compass heading error & servo PW & motor PW value & time, and
	transmit it to the SecureCRT terminal for filing and later plotting
	*/
	if(r_count%20)
	{
		time+=.4;
		r_counts=0;
		printf("\n%c,%c,%c", time, heading_error, Servo_PW, Motor_PW);
		lcd_clear();
		lcd_print("Heading is: %c\n", current_heading);
		lcd_print("Range is %c\n", range);
	}
}

//LOW LEVEL FUNCTIONS

//----------------------------------------------------------------------------
//Read_Compass
//----------------------------------------------------------------------------
void Read_Compass(void)
{
	i2c_read_data(COMPASS_ADDR, 2, Data, 2);	//Read two byte, starting at reg 2
	current_heading =(((unsigned int)Data[0] << 8) | Data[1]); //combine the two values
	//heading has units of 1/10 of a degree
}

//----------------------------------------------------------------------------
//Read_Ranger
//----------------------------------------------------------------------------
void Read_Ranger(void)
{
    if (!r_count % 4)
        //trigger every 80 ms
    {
        r_count = 0;
        i2c_read_data(RANGER_ADDR, 2, Data, 2);
        range = (unsigned int) Data[0] << 8 + (unsigned int) Data[1];
        //overwrites prev data and updates range
        Data[0] = PING_CM;
        i2c_write_data (RANGER_ADDR, 0, Data, 1 );
    }
}

//----------------------------------------------------------------------------
//Set_Servo_PWM
//----------------------------------------------------------------------------
void Set_Servo_PWM(void)
{
	heading_error = (signed int) desired_heading - current_heading;
    //Should allow error values between 3599 and -3599

	//If error greater than abs(180) degrees, then error is set to explementary angle of original error
    /*
	if (error > 1800)
		error = error - 3599;
	if (error < -1800)
		error = 3599 + error;
    */
    heading_error = (heading_error > 1800) ? (heading_error - 3599) : heading_error;
    heading_error = (heading_error < -1800) ? (heading_error + 3599) : heading_error;

	SERVO_PW = .416666*(heading_error) + SERVO_CENTER_PW;		//Limits the change from PW_CENTER to 750

	//Additional precaution: if SERVO_PW somehow exceeds the limits set in Lab 3-1,
	//then SERVO_PW is set to corresponding endpoint of PW range [PW_LEFT, PW_RIGHT]
    /*
	if (SERVO_PW > PW_RIGHT)
		SERVO_PW = PW_RIGHT;
	if (SERVO_PW < PW_LEFT)
		SERVO_PW = PW_LEFT;
    */
    SERVO_PW = (SERVO_PW > SERVO_RIGHT_PW) ? SERVO_RIGHT_PW : SERVO_PW;
    SERVO_PW = (SERVO_PW < SERVO_LEFT_PW) ? SERVO_LEFT_PW : SERVO_PW;

	PCA0CP0 = 0xFFFF - SERVO_PW;
}

//----------------------------------------------------------------------------
//Set_Motor_PWM
//----------------------------------------------------------------------------
void Set_Motor_PWM(void)
{
    if ( range <= 50 )
        //detected something at/closer than 50, stop
    {
        MOTOR_PW = MOTOR_NEUTRAL_PW;
    }
    else
        //nothing found too close, drive
    {
        MOTOR_PW = MOTOR_NEUTRAL_PW + ((float)(range-50)/(90-50))*(MOTOR_FORWARD_PW - MOTOR_NEUTRAL_PW);
    }
    PCA0CP2 = 0xFFFF - MOTOR_PW;
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
