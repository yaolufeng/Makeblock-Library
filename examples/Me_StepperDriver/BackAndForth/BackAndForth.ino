/*************************************************************************
* File Name          : BackAndForth.ino
* Author             : Evan
* Updated            : Evan
* Version            : V0.6.0
* Date               : 11/15/2013
* Description        : Test for Makeblock Electronic modules of Me_Stepper 
                       Driver. The module can only be connected to the 
                       PORT_1, PORT_2 of Me - Base Shield.. 
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeStepperMotor stepperDriver(PORT_1,0x5);
void initStepper(){
  stepperDriver.begin(STP_SIXTEENTH,10000,10000); // initialize stepper driver.
  stepperDriver.run(); // output pulse
  Serial.println("Stepper Begin");
}

void setup()
{
  Serial.begin(9600);
  initStepper();
}
void loop()
{
  if(!stepperDriver.isRunning()){
    initStepper();
  }
  long stp = stepperDriver.currentPosition(); // get the current position
 
  if(stp > 24000) // set the limit
    stepperDriver.moveTo(0); // Set the target position. Negative is anticlockwise from the 0 position.
  if(stp < 1) // set the limit
    stepperDriver.moveTo(24001); // set the target position.
}
