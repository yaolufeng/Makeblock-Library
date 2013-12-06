/*************************************************************************
* File Name          : TestRJ11Adapter.ino
* Author             : Steve
* Updated            : Steve
* Version            : V1.0.0
* Date               : 2/22/2013
* Description        : Test for Makeblock Electronic modules of Me - 
                       RJ11 Adapter. The module can ONLY be connected 
                       to the PORT_3, PORT_4, PORT_5, PORT_6, PORT_7, 
                       PORT_8 of Me - Base Shield. 
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeOutput output(PORT_4); 
                                    
void setup()
{
	Serial.begin(9600);

}

void loop()
{
	output.write1(1023);
	delay(50);
        output.write1(0);
        delay(50);
}
