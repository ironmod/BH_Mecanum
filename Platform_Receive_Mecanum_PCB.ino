/*

 ______   ______  _____  _     _ _______ _______  _____  _______ _     _
 |_____] |_____/ |     | |_____| |______ |  |  | |     |    |    |_____|
 |_____] |    \_ |_____| |     | |______ |  |  | |_____|    |    |     |

|-------------------------------------------------------------------|
|							DRONENET								|
|					Platform Motor Control Firmware					|
|					University of Central Florida					|
|					Coded in Stino for ATMEGA328					|
|							Brandon Frazer							|
|																	|						
|					REV7:   Now acts acts a slave					|
|							Added 2 modes for RL and FB Speed 		|
|								to speed up Program 				|
|					REV6: 	Motor Faults Added						|
|						  	IO Changed to match PCB Pinouts			|
|						  	PWM Fixed								|
|							Neutral Added							|
|							TX_Updater to PCT   					|
|-------------------------------------------------------------------|


			 _____  ______  _____  __   _ ______  _____ _______
			   |   |_____/ |     | | \  | |_____]   |      |   
			 __|__ |    \_ |_____| |  \_| |_____] __|__    |   

|-------------------------------------------------------------------|
|					Recieving Command Decimal Values                |
|-------------------------------------------------------------------|
|						Forward 	 	= 170 						|
|						Reverse  	 	= 85 						|
|						Left	     	= 90						|
|						Right			= 165						|
|						Strafe Left  	= 150 						|
|						Strafe Right 	= 105						|
|						Diagonal Left  	=  68						|
|						Diagonal Right 	=  34						|
|																	|
| 					B        Reverse								|
| 					F        Forward								|
|					R        Straffe Left 							|
|					L        Straffe Right 							|
| 					Q        PIVOT LEFT 							|
| 					W        Pivot Right 							|
| 					E        Turn Left 								|
| 					A        Turn Right 							|
|					Z        Diagonal Front Left 					|
| 					X        Diagonal Front Right					|
| 					C        Diagonal Back Left 					|
| 					V        Diagonal Back Right 					|
|																	|
|-------------------------------------------------------------------|

*/


#include "math.h"
#include <stdio.h>

/************************************************************************/
/*						Initialize Variables                           */
/************************************************************************/
//Motor Driver Left
	int M1_A 		= 3;
	int M1_B 		= 4;
	int M2_A		= 7;
	int M2_B		= 2;
//Motor Driver Right
	int M3_A 		= 8;
	int M3_B 		= 9;
	int M4_A		= 12;
	int M4_B		= 13;
//Motor Speed Variables
	int M1_SPEED 	= 5;
	int M2_SPEED	= 6;
	int M3_SPEED 	= 11;
	int M4_SPEED	= 10;
	int M_ALL_PWM 	= 0;
//Analog Current Reads (Speed)
	int M1_CUR		= A3;
	int M2_CUR		= A4;
	int M3_CUR		= A7;
	int M4_CUR		= A1;
	int M_AN_PINS[] = { M1_CUR, M2_CUR, M3_CUR, M4_CUR };
//Motor Fault Inputs 
	int M1_EN		= A2;
	int M2_EN		= A5;
	int M3_EN		= A6;
	int M4_EN		= A0;
	int M_EN_PINS[]	= { M1_EN, M2_EN, M3_EN, M4_EN };

//Motor Direction Deffinitions (change here instead of code)
	byte MOTOR_FWD 		= 0b10101010; // M1^ M2^ M3^ M4^	170
	byte MOTOR_RVS 		= 0b01010101; // M1v M2v M3v M4v	55	
	byte MOTOR_LFT      = 0b01011010; // M1v M2v M3^ M4^	90
	byte MOTOR_RGT 		= 0b10100101; // M1^ M2^ M3v M4^	165
	byte MOTOR_STR_LFT 	= 0b10010110; // M1^ M2v M3v M4^	150
	byte MOTOR_STR_RGT	= 0b01101001; // M1v M2^ M3^ M4v	105
	byte MOTOR_PVT_LFT	= 0b10100101; // M1^ M2^ M3v M4v	165
	byte MOTOR_PVT_RGT	= 0b01011010; // M1v M2v M3^ M4^	90
	byte MOTOR_DIA_FL	= 0b10000010; // M1^ M2  M3^ M4v 	130
	byte MOTOR_DIA_FR	= 0b00101000; // M1  M2^ M3  M4^	40
	byte MOTOR_DIA_BL	= 0b01000001; // M1  M2v M3v M4 	65
	byte MOTOR_DIA_BR	= 0b00010100; // M1v M2  M3  M4v 	20
	byte MOTOR_DIR_NEU  = 0b00000000; // All Motors Off 	0

//Arrays for Motor (program control)
	int M_SELECT = 0; //Select Which Motor to change speed
	int M_DIR[] = { M1_A, M1_B, M2_A, M2_B, M3_A, M3_B, M4_A, M4_B };
	int M_SPD[] = { 0, 0, 0, 0 };
	int M_CUR[] = { 0, 0, 0, 0 };
	int M_PWM[] = { M1_SPEED, M2_SPEED, M3_SPEED, M4_SPEED };
	int M_EN[]  = { 0, 0, 0, 0 };
	int data_read = B10101101;
	int MOTOR_DIR = B00000000;
        int M_LEFT_PWM = 0;
        int M_RIGHT_PWM = 0;
        int M_FRONT_PWM = 0;
        int M_REAR_PWM = 0;
//Program Control Variables
	boolean DEBUG 			= false;
	boolean UPDATE_COUNTER 	= false;	//counter for updating the display values
	boolean clear_screen 	= false;	//clear the LCD screen
	boolean CURRENT_ENABLE 	= false; 	//enable analog reads (encoder inputs)
	int DEBUG_ENABLE		= false;
	boolean LCD_ENABLE 		= false;
	int verify = 0;
	int i = 0; //index counter
	long previousMillis = 0;        // will store last time LED was updated
	// the follow variables is a long because the time, measured in miliseconds,
	// will quickly become a bigger number than can be stored in an int.
	long interval = 200;           // interval at which to blink (milliseconds)

/************************************************************************/
/*						Initial Setup Routine                           */
/************************************************************************/
void setup ()
{

	Serial.begin(57600);
	if(!DEBUG)
		LCD_Init();
		
	//ENABLE DEBUG MODE
	else if(DEBUG)
	{
		DEBUG_INIT();
	}
		
	//Set all pins as OUTPUTS
	for (i=0; i<8; i++)
		{
			pinMode(M_DIR[i], OUTPUT);
		}
		
	//Clear the PWM Values on startup
	for (i=0; i<8; i++)
		{
			digitalWrite(M_DIR[i], LOW);
		}

	//Initialize the Enable Pins as Inputs
	for(i=0; i<4; i++)
	{
			pinMode(M_EN_PINS[i], INPUT);
	}

}//end setup

/************************************************************************/
/*						Infinite Loop/Run Mode                          */
/************************************************************************/
void loop ()
{

	while(Serial.available())
	{
		data_read = Serial.read();
		switch(data_read)
		{
		//Assign Motor Direction
			case '0':		
				MOTOR_DIR = Serial.parseInt();	
				WRITE_DIR();
			break;
		
		//Assign Motor Speeds Individually 
			case '1':
				M_SPD[0] = Serial.parseInt();
				M_SPD[0] = constrain(M_SPD[0], 0 ,255);
				analogWrite(M1_SPEED, M_SPD[0]);
			break;
			
			case '2':
				M_SPD[1] = Serial.parseInt();
				M_SPD[1] = constrain(M_SPD[1], 0 ,255);
				analogWrite(M2_SPEED, M_SPD[1]);
			break;
			
			case '3':
				M_SPD[2] = Serial.parseInt();
				M_SPD[2] = constrain(M_SPD[2], 0 ,255);
				analogWrite(M3_SPEED, M_SPD[2]);
			break;
			
			case '4':
				M_SPD[3] = Serial.parseInt();
				M_SPD[3] = constrain(M_SPD[3], 0 ,255);
				analogWrite(M4_SPEED, M_SPD[3]);
			break;
		
		//Use the same speed for all motors
			case '5':
				M_ALL_PWM = Serial.parseInt();
				M_ALL_PWM = constrain(M_ALL_PWM, 0 ,255);
				for(i=0; i<4; i++)
				{
					M_SPD[i] = M_ALL_PWM;
					analogWrite(M_PWM[i], M_ALL_PWM);
				}
			break;
		//Control left motors only
			case '6':
				M_LEFT_PWM = Serial.parseInt();
				M_LEFT_PWM = constrain(M_LEFT_PWM, 0 ,255);
				analogWrite(M1_SPEED, M_LEFT_PWM);
				analogWrite(M2_SPEED, M_LEFT_PWM);
			break;

		//Control Right motors only
			case '7':
				M_RIGHT_PWM = Serial.parseInt();
				M_RIGHT_PWM = constrain(M_RIGHT_PWM, 0 ,255);
				analogWrite(M3_SPEED, M_RIGHT_PWM);
				analogWrite(M4_SPEED, M_RIGHT_PWM);
			break;					

		//Control Front motors only
			case '8':
				M_FRONT_PWM = Serial.parseInt();
				M_FRONT_PWM = constrain(M_FRONT_PWM, 0 ,255);
				analogWrite(M1_SPEED, M_FRONT_PWM);
				analogWrite(M3_SPEED, M_FRONT_PWM);
			break;	

		//Control Rear motors only
			case '9':
				M_REAR_PWM = Serial.parseInt();
				M_REAR_PWM = constrain(M_REAR_PWM, 0 ,255);
				analogWrite(M2_SPEED, M_REAR_PWM);
				analogWrite(M4_SPEED, M_REAR_PWM);
			break;	

		//Read the Current from each motor (analogread)
		//****PARTIALLY tested function*****
			case 'A':
				CURRENT_READ();

			break;

			case 'T':
				CURRENT_READ();
				TX_UPDATE();
			break;

		//Enable DEBUG Mode
			case 'D':
				DEBUG_ENABLE = Serial.parseInt();
				if(DEBUG_ENABLE== true )
				{
					DEBUG_INIT();
					DEBUG= true;	
				}
				
				else if(DEBUG_ENABLE== false && LCD_ENABLE == true)
				{
					DEBUG = false;
					Serial.println("Debug Disabled");
					LCD_JTC();
				}
			break;

//--------------------LEGACY MOTOR CONTROLS ------------------------------//
//-------Retains the previous Controls from before doing bitreads--------//

		//Motor Direction Controls
			case 'B':
				 //Reverse
				MOTOR_DIR = MOTOR_RVS;
				WRITE_DIR();
				break;
				
			case 'F':
				//Forward				 
				MOTOR_DIR = MOTOR_FWD;
				WRITE_DIR();
				break;
			
			case 'L':
				//Strafe Left
				MOTOR_DIR = MOTOR_STR_LFT;
				WRITE_DIR();
				break; 
				
			case 'R':
				//Strafe Right
				MOTOR_DIR = MOTOR_STR_RGT;
				WRITE_DIR();
				break; 	

			case 'Q':
				//Pivot Left
				MOTOR_DIR = MOTOR_PVT_LFT;
				WRITE_DIR();
				break; 	

			case 'W':
				//Pivot Right
				MOTOR_DIR = MOTOR_PVT_RGT;
				WRITE_DIR();
				break; 

			case 'E':
				//Turn Left
				MOTOR_DIR = MOTOR_LFT;
				WRITE_DIR();
				break; 	

			case 'P':
				//Turn Right
				MOTOR_DIR = MOTOR_RGT;
				WRITE_DIR();
				break;					

			case 'Z':
				//Diagonal Front Left
				MOTOR_DIR = MOTOR_DIA_FL;
				WRITE_DIR();
				break;

			case 'X':
				//Diagonal Front Right
				MOTOR_DIR = MOTOR_DIA_FR;
				WRITE_DIR();
				break; 	

			case 'C':
				//Diagonal Back Left
				MOTOR_DIR = MOTOR_DIA_BL;
				WRITE_DIR();
				break; 				

			case 'V':
				//Diagonal Back Right
				MOTOR_DIR = MOTOR_DIA_BR;
				WRITE_DIR();
				break; 										

			case 'N':
				//Shut everything off (Neutral)
				for(i=0; i<4; i++) //Turn off PWM first to discharge drivers
				{
					analogWrite(M_PWM[i], 0);
				}
				MOTOR_DIR = MOTOR_DIR_NEU;
				WRITE_DIR();
				break;

			}//end Motor FSM

		
	//Update the LCD Screen when NOT in debug 
	//update_counter helps with the refresh rate(1 time vs 3)
		if(!DEBUG && UPDATE_COUNTER && LCD_ENABLE)
		{
			UPDATE_COUNTER = false;
			LCD_UPDATE(); 
		}
		
	//update debug serial monitor
		else if (DEBUG && UPDATE_COUNTER)
		{	
			UPDATE_COUNTER = false;
			DEBUG_SERIAL();
		}
			
	}//end Serial Read
	
	//Serial.flush();
  //if(!Serial.available())
  //{/	
    //    CURRENT_READ();
//	TX_UPDATE();
  //}
	//Reset the Update Counter  
	UPDATE_COUNTER = true;
}//End Void loop

/************************************************************************/
/*						Main Control Terminal Update                    */
/************************************************************************/
void TX_UPDATE()
{
  unsigned long currentMillis = millis();
 
	if(currentMillis - previousMillis > interval) 
	{
	    // save the last time you blinked the LED 
	    previousMillis = currentMillis;   
            if(i==0)
            {
              Serial.print("0");
  	    Serial.write(M_CUR[i]);
            }
           else if(i==1)
            {
              Serial.print("1");
  	    Serial.write(M_CUR[i]);
            }
            else if(i==2)
            
            {
              Serial.print("2");
  	    Serial.write(M_CUR[i]);
            }
            else if(i==3)
            {
              Serial.print("3");
  	    Serial.write(M_CUR[i]);
            }
	    i++;
	}

	if(i == 3)
	{
		i = 0;
	}

	/*
	for (i=3; i>=0; i--)
	{
		//Serial.print("M");
		Serial.print(i);
		//Serial.print("S");
		//Serial.println(M_SPD[i]);
		//Serial.print("C");
		Serial.println(M_CUR[i]);
		//Serial.print("E");
		//Serial.println(M_EN[i]);
	}
	//Serial.println();
	*/
	
}//END DEBUG_SERIAL


/************************************************************************/
/*			Digital Write the Pins for Motor Direction                  */
/************************************************************************/
void WRITE_DIR()
{
	for (i=7; i>=0; i--)
	{
		digitalWrite(M_DIR[i], bitRead(MOTOR_DIR, i));
	}

	EN_READ();
	CURRENT_READ();
}

/************************************************************************/
/*						Update the PWM Values                           */
/************************************************************************/
void PWM_UPDATE()
{
	for(i=0; i<4; i++)
	{
		analogWrite(M_PWM[i], M_SPD[i]);
	}

	EN_READ();
	CURRENT_READ();
}//END PWM UPDATE

/************************************************************************/
/*				    Read the Motor Fault Codes (ENx)                    */
/************************************************************************/
void EN_READ()
{
	for(i=0; i<4; i++)
	{
		M_EN[i] = digitalRead(M_EN_PINS[i]);
	}

}//END PWM UPDATE

/************************************************************************/
/*				    Read the Current From Each Motor                   */
/************************************************************************/
void CURRENT_READ()
{
	for(i=0; i<4; i++)
	{
		M_CUR[i] = analogRead(M_AN_PINS[i]);
	}
}//END CURRENT READ

/************************************************************************/
/*			Initialize Serial "DEBUG" Monitor Update                   */
/************************************************************************/
void DEBUG_INIT()
{
	Serial.println();
	Serial.println(" ********************************** ");
	Serial.println(" *           DRONENET             *  ");
	Serial.println(" *     The Platform Of Peace	  *  ");
	Serial.println(" *     Parameters and Settings    *  ");
	Serial.println(" ********************************** ");

}//end DEBUG_INIT

/************************************************************************/
/*						Serial "DEBUG" Monitor Update                   */
/************************************************************************/
void DEBUG_SERIAL()
{
	Serial.println("----------------------------------- ");
	Serial.println("|           Parameters            | ");
	Serial.println("----------------------------------- ");
	Serial.print("Motor Direction: ");
	for(i = 7; i>=0; i--)
	{
		Serial.print(bitRead(MOTOR_DIR, i));
	}
	Serial.println();

	int j=7;
	//Print the Motor Speed Array
	for (i=3; i>=0; i--)
	{
		Serial.print("M");
		Serial.print(i);
		Serial.print(" Set Speed: ");
		Serial.println(M_SPD[i]);
		Serial.print("   Current: ");
		Serial.println(M_CUR[i]);
		Serial.print("   EN Stat: ");
		Serial.println(M_EN[i]);
		Serial.print("   Direction: ");
		Serial.print(bitRead(MOTOR_DIR, j)); //I should store these in an array
		j--;
		Serial.println(bitRead(MOTOR_DIR, j));
		j--;
	}
	Serial.println();

}//END DEBUG_SERIAL



/************************************************************************/
/*						Initialize the LCD Screen                      */
/************************************************************************/
void LCD_Init()
{
/*
	Serial.write(254); // move cursor to beginning of first line
	Serial.write(128);
	Serial.write("                "); // clear display
	Serial.write("                ");
	Serial.write("    Dronenet    ");
	Serial.write(254); // move cursor to beginning of first line
	Serial.write(192);
	Serial.write("Motor Controls");
	delay(1000);
	LCD_JTC();
*/	
}//END LCD_INIT


/************************************************************************/
/*						Clear the LCD(Neutral)	                       */
/************************************************************************/	
void LCD_JTC()
{
/*	Serial.write(254); // move cursor to beginning of first line
	Serial.write(128);
	Serial.write("                "); // clear display
	Serial.write("                ");
	Serial.write("Just Chillin");
	Serial.write(254); // move cursor to beginning of first line
	Serial.write(192);
	Serial.write("Speed: JTC");
*/
}//end LCD_JTC


/************************************************************************/
/*						Update the LCD Screen                           */
/************************************************************************/
void LCD_UPDATE()
{
	if(MOTOR_DIR==B00000000)
		LCD_JTC();
	else
	{
		//************* Motor 1********************//
			Serial.write(254);
			Serial.write(130);
			Serial.write("M1 ");
			Serial.write(254); // move cursor to beginning of first line
			Serial.write(133);

			if(M_SPD[0] < 10)
				Serial.write("  ");
			else if(M_SPD[0] <100)
				Serial.write(" ");

			Serial.print(M_SPD[0]);
			
		//************* Motor 2********************//
			Serial.write(254); // move cursor to beginning of first line
			Serial.write(136);
			Serial.write(" M2 ");
			if(M_SPD[1] < 10)
				Serial.write("  ");
			else if(M_SPD[1] <100)
				Serial.write(" ");
				
			Serial.print(M_SPD[1]);

		//************* Motor 3********************//
			Serial.write(254); // move cursor to beginning of first line
			Serial.write(192);
			Serial.write("  M3 ");
			Serial.write(254); // move cursor to beginning of first line
			Serial.write(197);

			if(M_SPD[2] < 10)
				Serial.write("  ");
			else if(M_SPD[2] <100)
				Serial.write(" ");
				
			Serial.print(M_SPD[2]);
			
		//************* Motor 4********************//
			Serial.write(254); // move cursor to beginning of first line
			Serial.write(200);
			Serial.write(" M4 ");
			Serial.write(254); // move cursor to beginning of first line
			Serial.write(204);

			if(M_SPD[3] < 10)
				Serial.write("  ");
			else if(M_SPD[3] <100)
				Serial.write(" ");
				
			Serial.print(M_SPD[3]);
	}//end else
}//end LCD_UPDATE

