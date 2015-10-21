// This sketch is for Arduino versions 1.0 and later
// If you're using an Arduino version older than 1.0, use
// the other example code available on the tutorial page.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
#include <SoftwareSerial.h>
#include <stdio.h>  //not needed yet
#include <string.h>  //not needed yet
#include <OneWire.h>

/*************************************************************************************/
// Declarations
/*************************************************************************************/
// Ports constants 
const int LEFT = 0;
const int RIGHT = 1;
const int MIDDLE = 2;
const int UP = 3;
const int DOWN = 4;

const int THERMISTORPIN = 51;  // Port for sensor
const int LCD_PORT = 53;
const int RELAY = 42;  // Set to high to turn the heat on and low to turn off

// Temperature calculation variables
OneWire  ds(51);  // on pin 10 (a 4.7K resistor is necessary)
byte data[12];
byte addr[8];
const int HOTPLATE_ON = HIGH; 
const int HOTPLATE_OFF = LOW;
const int OFF = 0;
const int ON = 1;
int RELAY_STATE = OFF;

// Menu constants
const int TEMPERATURE_SCREEN = 0;
const int TOLERANCE_SCREEN = 1;
const int SENSOR_SCREEN = 2;
const int CANCEL_SCREEN = 3;
const int NUMBER_OF_SCREENS = 4;

// Menu variables
int actualTemperature = -1;
int actualTemperatureF = -1;
float averageAnalog = 0;
int temperature = 33;
int tolerance = 2;
int temp_temperature = 33;
int temp_tolerance = 2;
int currentScreen = -1;
char* screenTitles[]={"Set Temperature:", "Set Tolerance:  ", "Cancel?         "};

// Buttons
int buttons[] = {44,46,48,50,52};     // the number of the pushbutton pin
int numberOfButtons = 5;     // the number of the pushbutton pin

// LCD
SoftwareSerial mySerial(3,LCD_PORT);




/*************************************************************************************/
// Arduino Methods
/*************************************************************************************/
void setup()
{
  mySerial.begin(9600); // set up serial port for 9600 baud
  Serial.begin(9600);
  // Buttons initializations
  for (int thisPin = 0; thisPin < numberOfButtons; thisPin++) {
    pinMode(buttons[thisPin], INPUT);
    digitalWrite(buttons[thisPin], HIGH);
  }
  
  pinMode(RELAY, OUTPUT);
//  pinMode(RELAY, INPUT);
  digitalWrite(RELAY, HOTPLATE_OFF);
  RELAY_STATE = OFF;
  analogReference(EXTERNAL); // For sensor
  ds.search(addr);

  goToDefaultScreen();
}

void loop()
{
  handleButtonActions();
  refreshScreen();
  handleSensor();
  handleHeatControl();
  
}
/*************************************************************************************/
// Support Functions
/*************************************************************************************/

void handleHeatControl(){
 Serial.print("Checking hotplate for action.  Target: ");
 Serial.print(temperature);
 Serial.print(".  Actual: ");
 Serial.print(actualTemperature);
 Serial.print(".  Tolerance: ");
 Serial.print(tolerance);
 Serial.print(".  State: ");
 Serial.print(RELAY_STATE == OFF ? "OFF" : "ON");
 Serial.println(digitalRead(RELAY) == HIGH ? " | HIGH" : " | LOW");

 
 if( actualTemperature < ( temperature - tolerance ) && RELAY_STATE == OFF){
  digitalWrite(RELAY, HOTPLATE_ON);
  Serial.println("Turned on hotplate");
  mySerial.write("Adjusting Temp");
  RELAY_STATE = ON;
  delay(10);
  
 }
 if ( actualTemperature > ( temperature + tolerance ) && RELAY_STATE == ON ){
   digitalWrite(RELAY, HOTPLATE_OFF);
   Serial.println("Turned off hotplate");
   RELAY_STATE = OFF;
   delay(10);
 }
}

// If a buttons is pressed return the button id.  If no button is pressed return -1
int getButtonPress(){
  for (int thisPin = 0; thisPin < numberOfButtons; thisPin++) {
    if(digitalRead(buttons[thisPin]) == LOW)
      return thisPin;
  }
  return -1;
}

void handleSensor(){
  // convert to C
  actualTemperature = getTemp();
  actualTemperatureF = actualTemperature * 1.8 + 32.0;
  Serial.print(", Temperature: ");
  Serial.print(actualTemperature);
  Serial.print(" C and ");
  Serial.print(actualTemperatureF);
  Serial.println(" F");
}

float getTemp()
{
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(10);     // maybe 750ms is enough, maybe not  
  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( byte i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  return convertToTemp(data) / 16.0;
}

float convertToTemp(byte data[12]){
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time
  return (float)raw;
}

void handleButtonActions(){
  int pressedButton = getButtonPress();
  switch(pressedButton){
    case LEFT:
      if(currentScreen <= 0) currentScreen = NUMBER_OF_SCREENS;
      currentScreen = (currentScreen - 1) % NUMBER_OF_SCREENS;
      break;
    case RIGHT:
      currentScreen = (currentScreen + 1) % NUMBER_OF_SCREENS;
      break;
    case DOWN:
      if(currentScreen == TEMPERATURE_SCREEN)
          temp_temperature--;
      else if(currentScreen == TOLERANCE_SCREEN)
          temp_tolerance--;
      break;
    case UP:
      if(currentScreen == TEMPERATURE_SCREEN)
          temp_temperature++;
      else if(currentScreen == TOLERANCE_SCREEN)
          temp_tolerance++;
      break;
    case MIDDLE:
      if(currentScreen != CANCEL_SCREEN){
        temperature = temp_temperature;
        tolerance = temp_tolerance;
      }
      currentScreen = -1;
      break;
  }
}

void refreshScreen(){
  switch(currentScreen){
    case -1: goToDefaultScreen(); break;
    case TOLERANCE_SCREEN: printScreen(screenTitles[currentScreen], temp_tolerance, false); break;
    case TEMPERATURE_SCREEN: printScreen(screenTitles[currentScreen], temp_temperature, true); break;
    case CANCEL_SCREEN:
      clearDisplay();
      mySerial.write("Cancel?");
  }
}
void printScreen(char* str, int val, bool printBoth){
  clearDisplay();
  mySerial.write(str);
  char tempstring[16];
  if(printBoth)
    sprintf(tempstring, "%3d F     %3d C", ((val*9/5) + 32), val);
  else
    sprintf(tempstring, "%3d C           ", val);
    
  mySerial.write(tempstring);
}
void clearDisplay(){
  mySerial.write(254); // move cursor to beginning of first line
  mySerial.write(128);
  mySerial.write("                "); // clear display
  mySerial.write("                "); // clear display
  mySerial.write(254); // move cursor to beginning of first line
  mySerial.write(128);
  
}

void goToDefaultScreen(){
  clearDisplay();
  mySerial.write("Temperature:    ");
  char tempstring[16];
  sprintf(tempstring, "%3d F     %3d C", actualTemperatureF, actualTemperature);
  mySerial.write(tempstring);
}


