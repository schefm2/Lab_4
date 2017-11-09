/* Sample code for speed control using PWM. */
#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h>
#include <i2c.h>

//#define ON 1
//#define OFF 0

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
void Wait(void);
unsigned int pow(unsigned int a, unsigned char b);
unsigned int calibrate(void);
unsigned char parallel_input(void);
unsigned char read_AD_input(unsigned char pin_number);


void Car_Parameters(void);
void Set_Motion(void);
void Set_Neutral(void);
void Print_Data(void);

unsigned char addr=0xE0; // the address of the ranger is 0xE0
unsigned char Data[2];
unsigned int desired_heading = 0;
unsigned int initial_speed = 0;
unsigned int PCA_overflows, current_heading, heading_error, range, Servo_PW, Motor_PW;
unsigned char r_check, keyboard, keypad, r_count, answer;
float time, gain;

//sbits
__sbit __at 0xB7 SS; //Port 3.7 slideswitch run/stop
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
	
	Car_Parameters();
	r_count = 0;
	while(r_count<3);
	r_count = 0;

	while(1)
	{
		printf("\r\nEntered the main while loop.");
		Set_Motion();
		printf("\r\nFinished Set_Motion.");
		Set_Neutral();
		printf("\r\nFinished Set_Neutral");
		Print_Data();

    	if (range <= 50)
        //detected something at/closer than 50, stop
    	{
        	Motor_PW = MOTOR_NEUTRAL_PW;
        	printf("Press 4 for left or 6 for right\n\r");

        	while(answer != '4' && answer != '6'){answer=parallel_input();}
        	
        	if(answer=='4')
        	{
        		while(getchar() != ' ') {Servo_PW = 10.2*(heading_error) + SERVO_CENTER_PW;}
        	}
        	if(answer=='6')
        	{
        		while(getchar() != ' ') {Servo_PW = 10.2*(heading_error) + SERVO_CENTER_PW;}
        	}

    	}
	}
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
	Servo_PW = SERVO_CENTER_PW;		//Initialize car to straight steering and no movement
	Motor_PW = MOTOR_NEUTRAL_PW;
	PCA0CP0 = 0xFFFF - Servo_PW;
	PCA0CP2 = 0xFFFF - Motor_PW;

	
	printf("\nStart");
	
	Wait();
    lcd_clear();
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");
	Wait();
	
	
	lcd_clear();
	lcd_print("Set gain with pot");
	printf("\r\nTurn the potentiometer clockwise to increase the steering gain from 0 to 10.2.\r\nPress # when you are finished.");
	calibrate();
	gain = ((float)read_AD_input(7) / 255) * 10.2;
	printf_fast_f("Your gain is %3.1f", gain);
	lcd_print("Gain is %d",gain/10.2*100);
	Wait();
	
	do
	{
		lcd_clear();
		lcd_print("Press 5 keys.\n");
		printf("\r\nSelect a desired heading (0 to 3599) by inputing 5 digits. Lead with a 0. Press # to confirm.\r\n");
		desired_heading = calibrate();
		Wait();
	}
	while (desired_heading > 3599);
	printf("\r\nYou selected %u as your heading", desired_heading);
	Wait();
	
	lcd_clear();
	lcd_print("Press 5 keys.\n");
	do
	{
		lcd_clear();
		lcd_print("Press 5 keys.\n");	
		printf("\r\nSelect an initial speed (2765 to 3502) by inputing 5 digits. Lead with a 0. Press # to confirm.\r\n");
		initial_speed = calibrate();
		Wait();
	}
	while (initial_speed < 2765 || initial_speed > 3502);
	printf("\r\nYou selected %u as your speed", initial_speed);
	Wait();
	PCA0CP0 = 0xFFFF - 0;
	PCA0CP2 = 0xFFFF - initial_speed;
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
    if (SS)
    {
        Servo_PW = SERVO_CENTER_PW;
        Motor_PW = MOTOR_NEUTRAL_PW;
        while(SS) {}
        //wait until !SS
    }
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
		r_count=0;
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

	Servo_PW = gain*(heading_error) + SERVO_CENTER_PW;		//Limits the change from PW_CENTER to 750

	//Additional precaution: if SERVO_PW somehow exceeds the limits set in Lab 3-1,
	//then SERVO_PW is set to corresponding endpoint of PW range [PW_LEFT, PW_RIGHT]
    /*
	if (SERVO_PW > PW_RIGHT)
		SERVO_PW = PW_RIGHT;
	if (SERVO_PW < PW_LEFT)
		SERVO_PW = PW_LEFT;
    */
    Servo_PW = (Servo_PW > SERVO_RIGHT_PW) ? SERVO_RIGHT_PW : Servo_PW;
    Servo_PW = (Servo_PW < SERVO_LEFT_PW) ? SERVO_LEFT_PW : Servo_PW;

	PCA0CP0 = 0xFFFF - Servo_PW;
}

//----------------------------------------------------------------------------
//Set_Motor_PWM
//----------------------------------------------------------------------------
void Set_Motor_PWM(void)
{
	
        //nothing found too close, drive
    {
        Motor_PW = MOTOR_NEUTRAL_PW + ((float)(range-50)/(90-50))*(MOTOR_FORWARD_PW - MOTOR_NEUTRAL_PW);
    }
    PCA0CP2 = 0xFFFF - Motor_PW;
}

//----------------------------------------------------------------------------
//Pause
//----------------------------------------------------------------------------
void Pause(void)
{
	r_count = 0;
	while (r_count < 2);
}

//----------------------------------------------------------------------------
//Wait
//----------------------------------------------------------------------------
void Wait(void)
{
	r_count = 0;
	while (r_count < 50);
}

//----------------------------------------------------------------------------
//Pow
//----------------------------------------------------------------------------
unsigned int pow(unsigned int a, unsigned char b)
{
	unsigned char i;
	unsigned char base = a;

	if (b == 0) return 1;
	for(i = 1; i < b; i++)
		a = a*base;
	return a;
}

//----------------------------------------------------------------------------
//calibrate
//----------------------------------------------------------------------------
unsigned int calibrate(void)
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
		Pause();					//Pause necessary to prevent overreading the keypad
		
		if (keyboard == '#' || keypad == '#') //# is a confirm key, so it will finish calibrate()
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
//----------------------------------------------------------------------------
//parallel_input
//----------------------------------------------------------------------------
unsigned char parallel_input(void)
{
	unsigned char keypad;
	unsigned char keyboard;
	while(1)
	{
		keyboard = getchar_nw();	//This constantly sets keyboard to whatever char is in the terminal
		keypad = read_keypad();		//This constantly sets the keypad to whatever char is on the LCD
		Pause();					//Pause necessary to prevent overreading the keypad
		
		if (keyboard != 0xFF)
			return keyboard;
		if (keypad != 0xFF)
			return keypad;
	}
	
}

//----------------------------------------------------------------------------
//read_AD_input
//----------------------------------------------------------------------------
unsigned char read_AD_input(unsigned char pin_number)
{
    AMX1SL = pin_number;		//Sets multiplexer to convert correct pin
    ADC1CN &= ~0x20;			//Clears the A/D conversion complete bit
    ADC1CN |= 0x10;				//Starts A/D conversion
    while(!(ADC1CN & 0x20));	//Waits until conversion completes 
    return ADC1;				//returns converted input, 0-255 inclusive
}
//----------------------------------------------------------------------------
//ADC_Init
//----------------------------------------------------------------------------
void ADC_Init(void)
{
	REF0CN = 0x03;	//Sets V_ref as 2.4V
	ADC1CN = 0x80;	//Enables AD/C converter

	//Gives capacitors in A/D converter time to charge
	r_count = 0; //makes sure r_count isn't altered while waiting 
	while(r_count < 6);

	//Sets gain to 1
	ADC1CF |= 0x01;
	ADC1CF &= 0xFD;
}

//-----------------------------------------------------------------------------
//Port_Init
//-----------------------------------------------------------------------------
void Port_Init()
{
	//Initailize POT
	P1MDOUT |= 0x05;	//Set output pin for CEX0 and CEX2 in push-pull mode
	P1MDOUT &= ~0x80;	//Set POT pin (P1.7) to open drain
	P1 |= 0x80;		//Set impedance high on P1.7
	P1MDIN &= ~0x80;
	
	P3MDOUT &= ~0x80; //Pin 3.7 open drain
	P3 |= 0x80; //Pin 3.7 high impedance
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
	PCA0CPM0 = PCA0CPM2 = 0xC2;		//Sets both CCM0 and CCM2 in 16-bit compare mode, enables PWM
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



