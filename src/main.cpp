#include <Arduino.h>
#include "SPI.h"

/*
* Authors: Shaurya Kumar, Tommy Wong, Siqi Yuan
* Code: Uses accelerometer to detect if patient has stopped breathing for 10 seconds  
*
*/


// globals
float curr = 0; //the current acceleration value
float prev = 0; //the previous acceleration value over the last second
int iterations = 0; //how many times we haven't seen sufficient change in acceleration

/// @brief sets up Accelerometer 
void SetupAccel()
{
  //begin SPI protocol by writing pin 8 to low
  SPI.begin();
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);

  //reading at setup register, transfer parameters for 1Hz read, normal operation, all axis enabled
  SPI.transfer(0x20);
  uint8_t data = SPI.transfer(0b00010111);

  //stop transmission
  digitalWrite(8,HIGH);
}

/// @brief reads the low register of the Z axis of accelerometer (left justified)
/// @return data from low register
uint8_t ReadLowZ()
{
  //we want to read from Z low register
  digitalWrite(8, LOW);
  SPI.transfer(0x2C | (1<<7)); //read from Z Low
  uint8_t data = SPI.transfer(0b00000000);

  //stop transmission
  digitalWrite(8, HIGH);

  return data;
}

/// @brief reads the high register of the Z axis of accelerometer (left justified)
/// @return data from high register
uint8_t ReadHighZ()
{
  //we want to read from Z high register
  digitalWrite(8, LOW);
  SPI.transfer(0x2D | (1<<7)); //read Z High
  uint8_t data = SPI.transfer(0b00000000);

  //stop transmission
  digitalWrite(8, HIGH);

  return data;
}

/// @brief sets the onboard LED to an output and turns it off to start
void SetupLED()
{
  //set to an output
  DDRC |= (1<<7);

  //Start off
  PORTC &= ~(1<<7);
}

/// @brief sets the speaker to an output 
void SetupSpeaker()
{
  //set speaker to output, C6
  DDRC |= (1<<6);
}

/// @brief sounds the alarm if breathing has stopped with both noise and light; press reset to exit loop
void Alarm()
{
  //creates high frequency sound rapidly, plus rapidly blinking light
  for (int i = 0; i < 2500; i++)
  {
    //light and sound on
    PORTC |= (1<<6) | (1<<7);
    delay(20);
    //light and sound off
    PORTC &= ~((1<<6) | (1<<7));
    delay(20);
  }
}

/// @brief takes an unsigned 10-bit integer and converts it to signed representation
/// @param toConvert the number to convert
/// @return the converted number
int16_t convert(int16_t toConvert)
{
  int16_t num = 0;
  if(toConvert > 512)
  {
    num = (512-toConvert);
  }
  else
  {
    num = toConvert;
  }
  return num;
}

/// @brief setup code that runs at startup
void setup() {
  //serial code for monitoring; not necessary to run program
  Serial.begin(9600);
  //Setup the speaker and LED
  SetupSpeaker();
  SetupLED();
  //Setup the accelerometer 
  SetupAccel();
}

/// @brief code that continues to run until reset button pressed
void loop() {
  //read in the two 8-bit z values and combine them into one 16-bit integer
  uint8_t low_z = ReadLowZ();
  uint8_t high_z = ReadHighZ();
  int16_t z = (high_z << 2) | (low_z >> 6);

  //convert the binary integer into G's, easier to work with in code
  z = convert(z);
  curr = z*4/1000.0;

  //find the change from the previous conversion
  float change = curr - prev;
  prev = curr;
  Serial.println(change);

  //if not sufficient change start counting; else reset counter
  if (abs(change) < 0.015)
  {
    iterations += 1;
  }
  else
  {
    iterations = 0;
  }

  //if after 10 times there is not sufficient change in breathing GO CRAZY
  if(iterations >= 10)
  {
    Alarm();
  }
  
  //1 iteration = 1 second
  delay(1000);
}


