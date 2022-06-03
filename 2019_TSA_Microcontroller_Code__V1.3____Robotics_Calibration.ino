/*
2019 TSA Microcontroller project
Theme: Smart Home
Project: 4 Channel Home Power Meter
Author: EK and TM
V1.3
*/

#include "rgb_lcd.h"
#include "EmonLib.h"

//#define demo //define if the demo setup is used
#define robotics

#ifdef demo //Calibration values for the demo setup
//Voltage and Current sensor calibrations
#define VOLT_CAL 161.2 
#define CH1_CURR_CAL 9.1  
#define CH2_CURR_CAL 10.4 
#define CH3_CURR_CAL 9.3
#define CH4_CURR_CAL 9.3

#endif

#ifdef dev //Calibration values for the developement setup.
//Voltage and current senosr calibrations
#define VOLT_CAL 166.4 
#define CH1_CURR_CAL 11.1
#define CH2_CURR_CAL 11.1
#define CH3_CURR_CAL 11.1
#define CH4_CURR_CAL 11.1

#endif
#ifdef robotics //Calibration values for Robotics Interview.

#define VOLT_CAL 155.8 //was 166.4 
#define CH1_CURR_CAL 10.2
#define CH2_CURR_CAL 11.1
#define CH3_CURR_CAL 11.1
#define CH4_CURR_CAL 11.1

#endif

EnergyMonitor emon1; //Create Emon instance for channel 1
EnergyMonitor emon2; //Create Emon instance for channel 2
EnergyMonitor emon3; //Create Emon instance for channel 3
EnergyMonitor emon4; //Create Emon instance for channel 4

// LCD Parameters
rgb_lcd lcd; //Create LCD instance
//RGB backlight values (White)
const int backlightR = 255; 
const int backlightG = 255;
const int bakclightB = 255;

//Loop control vars
const int screen_count_max = 75; //Set LCD screen switch's max value
int screen_count;
int emon_loop_count;

//Channel select variables
int CHSELBUT = 9; //Button GPIO pin
int channelnumber = 0; 

int last_button_val; //storage var
int selected_channel = 0; 

enum LEDChannels {RESET, Ch1P, Ch2P, Ch3P, Ch4P, Ch1I, Ch2I, Ch3I, Ch4I, CHSEL, LEDMAX}; 
int LEDPin[LEDMAX]; //LED GPIO output pins array

enum Channels {TOTAL, Ch1, Ch2, Ch3, Ch4, CHANNELMAX};

//arrays for storing Voltage, Current, Power, and Energy measurements
double voltage[CHANNELMAX];
double current[CHANNELMAX];
double power[CHANNELMAX];
double energy[CHANNELMAX];

int last_millis[CHANNELMAX]; 

//Threshholds for Active chanel LEDs
const float active_channel_threshhold = 0.1; //Ignore current readings below this value for Active LED indicators
int voltage_threshhold = 100; //Active LED turn on threshold

//Conversion factors for Energy calculation
#define w2kw (1/1000.0)
#define ms2h (1/(1000.0 * 60.0 * 60.0))
#define ms2m (1/(1000.0 * 60.0))
#define ms2s (1/(1000.0))

//-------------------------------------------------------------------
// Support functions
//-------------------------------------------------------------------

// Initialize channel select button
void initCHSELbutton() 
  {
    pinMode(CHSELBUT, INPUT_PULLUP); 
    last_button_val = digitalRead(CHSELBUT);
  }

// Initialize LCD display
void initLcd() 
  {
    lcd.begin(16, 2);
    lcd.setRGB(backlightR, backlightG, bakclightB);
  }

// Funciton for turning an LED on
void LEDon(int LED)
  {
    digitalWrite(LEDPin[LED], LOW); //active low
  }

// Function for turning an LED off
void LEDoff(int LED){
  digitalWrite(LEDPin[LED], HIGH); //active low
}

// Initialize LED GPIO pins and test them
void initLED() 
  {
    //define Digital Pin #s for each LED
    LEDPin[RESET] = 10;
    LEDPin[Ch1P] = 6;
    LEDPin[Ch2P] = 3;
    LEDPin[Ch3P] = 5;
    LEDPin[Ch4P] = 4;
    LEDPin[Ch1I] = 8;
    LEDPin[Ch2I] = 12; // was 1
    LEDPin[Ch3I] = 7;
    LEDPin[Ch4I] = 2;
    LEDPin[CHSEL] = 13; //was 0, not used
  
    //set LEDs to output mode and test them
    for(int channel = 0; channel < LEDMAX; channel++) 
      {
        pinMode(LEDPin[channel], OUTPUT);
        LEDon(channel);
        delay(150);
        LEDoff(channel);
      }
   }

// Function to turn on selected channel LEDs
void setChannelLED(int setChannel) 
  {
    if (setChannel == TOTAL) 
      {
        LEDon(Ch1I);
        LEDon(Ch2I);
        LEDon(Ch3I);
        LEDon(Ch4I);
      }
    else 
      {
        LEDoff(Ch1I);
        LEDoff(Ch2I);
        LEDoff(Ch3I);
        LEDoff(Ch4I);
      }
    if (setChannel == Ch1) 
      {
        LEDon(Ch1I);
      }
    if (setChannel == Ch2) 
      {
        LEDon(Ch2I);
      }
    if (setChannel == Ch3) 
      {
        LEDon(Ch3I);
      }
    if (setChannel == Ch4) 
      {
        LEDon(Ch4I);
      }
  }  

//Initialize EmonLib
void initemon() 
  {
    emon1.voltage(A0, VOLT_CAL, 1.7); //Voltage input pin calibration, phase_shift
    emon1.current(A1, CH1_CURR_CAL);       // Current: input pin, calibration.

    emon2.voltage(A0, VOLT_CAL, 1.7);
    emon2.current(A2, CH2_CURR_CAL);

    emon3.voltage(A0, VOLT_CAL, 1.7);
    emon3.current(A3, CH3_CURR_CAL);

    emon4.voltage(A0, VOLT_CAL, 1.7);
    emon4.current(A4, CH4_CURR_CAL);
  
    pinMode(A0, INPUT); //voltage pin
    pinMode(A1, INPUT); //CH1 current pin1
    pinMode(A2, INPUT); //CH2 current pin2
    pinMode(A3, INPUT); //CH3 current pin3
    pinMode(A4, INPUT); //CH4 current pin4
  }

void displayno1() 
  {
    lcd.setCursor(15, 1);
    lcd.print("0");
  }

void displayno2() 
  {
    lcd.setCursor(15, 1);
    lcd.print("1");
  }

//-------------------------------------------------------------------
// Setup
//-------------------------------------------------------------------

void setup() 
  {
    Serial.begin(9600);
    initLcd();
    initLED();
    initCHSELbutton(); 
    initemon();

    LEDon(RESET);
    setChannelLED(selected_channel);
    screen_count = 0;
    emon_loop_count = 0;

    last_millis[TOTAL] = millis();
    last_millis[Ch1] = millis();
    last_millis[Ch2] = millis();
    last_millis[Ch3] = millis();
    last_millis[Ch4] = millis();  
  }

//------------------------------------------------------------------
// Main Program Loop
//------------------------------------------------------------------
void loop() 
  {
    //EmonLib
    int curr_millis; //Current time stamp
    
//------------------------------------------------------------------
// Process Channel Select button presses 
//------------------------------------------------------------------
  int curr_button_val = digitalRead(CHSELBUT);
  
  if ((curr_button_val == LOW) && (last_button_val == HIGH))
    {
      // High to low transition of CHSELBUT pin
      selected_channel++;
      if (selected_channel == CHANNELMAX)
        { 
          selected_channel = 0;
        }
      setChannelLED(selected_channel); //turn on or off Indicator LEDS.
    }
        
  last_button_val = curr_button_val;
 
//------------------------------------------------------------------
// Voltage, Current, Power Calculations
//------------------------------------------------------------------

  curr_millis = millis();

  if (emon_loop_count == 0) //Total of all channels
    {
      voltage[TOTAL] = (voltage[Ch1] + voltage[Ch2] + voltage[Ch3] + voltage[Ch4]) / 4;
      current[TOTAL] = current[Ch1] + current[Ch2] + current[Ch3] + current[Ch4];
      power[TOTAL] = power[Ch1] + power[Ch2] + power[Ch3] + power[Ch4];
      energy[TOTAL] = energy[Ch1] + energy[Ch2] + energy[Ch3] + energy[Ch4];
      emon_loop_count++;
    }
  else if (emon_loop_count == 1) // Channel 1
    {
      emon1.calcVI(10,50); //Measure Voltage and Current
      voltage[Ch1] = emon1.Vrms;
      current[Ch1] = emon1.Irms;
      power[Ch1] = voltage[Ch1] * current[Ch1];
      int diff_millis = curr_millis - last_millis[Ch1];
      energy[Ch1] = energy[Ch1] + (power[Ch1] * diff_millis);
      last_millis[Ch1] = curr_millis;
      emon_loop_count++;
    }  
  else if (emon_loop_count == 2) // Channel 2
    {
      emon2.calcVI(10,50); //Measure Voltage and Current
      voltage[Ch2] = emon2.Vrms;
      current[Ch2] = emon2.Irms;
      power[Ch2] = voltage[Ch2] * current[Ch2];
      int diff_millis = curr_millis - last_millis[Ch1];
      energy[Ch2] = energy[Ch2] + (power[Ch2] * diff_millis);
      last_millis[Ch2] = curr_millis;
      emon_loop_count++;
    }
  else if (emon_loop_count == 3) // Channel 3
    {
      emon3.calcVI(10,50); //Measure Voltage and Current
      voltage[Ch3] = emon3.Vrms;
      current[Ch3] = emon3.Irms;
      power[Ch3] = voltage[Ch3] * current[Ch3];
      int diff_millis = curr_millis - last_millis[Ch3];
      energy[Ch3] = energy[Ch3] + (power[Ch3] * diff_millis);
      last_millis[Ch3] = curr_millis;
      emon_loop_count++;
    }
  else if (emon_loop_count == 4) // Channel 4
    {
      emon4.calcVI(10,50); //Measure Voltage and Current
      voltage[Ch4] = emon4.Vrms;
      current[Ch4] = emon4.Irms;
      power[Ch4] = voltage[Ch4] * current[Ch4];
      int diff_millis = curr_millis - last_millis[Ch4];
      energy[Ch4] = energy[Ch4] + (power[Ch4] * diff_millis);
      last_millis[Ch4] = curr_millis; 
      emon_loop_count = 0;
    }

//------------------------------------------------------------------
// Turn on or off Active LED indicators based on current draw
//------------------------------------------------------------------

  //ingore all current readings unless there is 110V AC presenton a channel
  if (voltage[TOTAL] > voltage_threshhold) 
    { 

      // if the current values are above the active_channel_threshhold, then turn on the LED
      if (current[Ch1] > active_channel_threshhold) 
        {
          LEDon(Ch1P);
        }
      else 
        {
          LEDoff(Ch1P);
        }
      if (current[Ch2] > active_channel_threshhold) 
        {
          LEDon(Ch2P);
        }
    else 
        {
          LEDoff(Ch2P);
        }
     if (current[Ch3] > active_channel_threshhold) 
        {
          LEDon(Ch3P);
        }
    else 
        {
          LEDoff(Ch3P);
        }

    if (current[Ch4] > active_channel_threshhold) 
        {
          LEDon(Ch4P);
        }
    else 
        {
          LEDoff(Ch4P);
        }
    }
    
//------------------------------------------------------------------
// Display Voltage and Current measurments on LCD. 
// Display Power and Energy calculations on LCD.
// Alternate Screens that display Measurements and Calculations
//------------------------------------------------------------------

  char str[17]; 
  if(screen_count < (screen_count_max / 2))
    { 
      //Display Voltage 
      lcd.setCursor(0, 0); 
      memset(str, ' ', sizeof(str)); str[17] = 0;
      dtostrf(voltage[selected_channel],9,1,str);
      lcd.print("V: ");
      lcd.print(str);
      lcd.print("V       "); // clear to end of line

      //Display Current
      lcd.setCursor(0,1);
      memset(str, ' ', sizeof(str)); str[17] = 0;
      dtostrf(current[selected_channel],9,2,str);
      lcd.print("I: ");
      lcd.print(str);
      lcd.print("A       "); // clear to end of line
    }
  else
    {
      //Display Power
      lcd.setCursor(0,0);
      memset(str, ' ', sizeof(str)); str[17] = 0;
      dtostrf(power[selected_channel],9,1,str); 
      lcd.print("P: ");
      lcd.print(str);
      lcd.print("W       "); // clear to end of line

      //Display Energy
      lcd.setCursor(0,1);
      memset(str, ' ', sizeof(str)); str[17] = 0;
      dtostrf(energy[selected_channel] * ms2h,9,0,str); // convert Wms to Wh
      lcd.print("E: ");
      lcd.print(str);
      lcd.print("Wh       "); // clear to end of line
    }

  //Indicate Seleceted channel on upper right corner of LCD
  lcd.setCursor(15,0);
  if(selected_channel == 0) 
    {
      lcd.print("T");
    }
  else
    {
      lcd.print(selected_channel);
    }

  screen_count++; //Advance the screen count every time the loop is executed
  if(screen_count > screen_count_max) // If the counter reaches max, reset the counter
    {
      screen_count = 0;
    }
}
