#include "Makeblock.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/wdt.h>
MeSerial serial(PORT_5);
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);
MeDCMotor motor3(PORT_1);
MeDCMotor motor4(PORT_2);
MeUltrasonicSensor ultrasonic;
MeLimitSwitch limitSwitch;
void setup() {
  wdt_enable(WDTO_1S);
  serial.begin(9600);
}
char *device;
char *method;
int port;
int value;
int slot;
int ultrasonic_port=-1;
int switch_port=-1;
int switch_slot=1;
void loop() {
  if(serial.paramAvailable()){
    
    device = serial.getParamCode("device");
    port = serial.getParamValue("port");
    value = serial.getParamValue("value");
    slot = serial.getParamValue("slot");
    method = serial.getParamCode("method");
    if(strcmp(device,"motor")==0){
      switch(port){
        case M1:{
         motor1.run(value); 
        }
        break;
        case M2:{
         motor2.run(value); 
        }
        break;
        case PORT_1:{
         motor3.run(value); 
        }
        break;
        case PORT_2:{
         motor4.run(value); 
        }
        break;
      }
    }else if(strcmp(device,"Ultrasonic")==0){
      if(strcmp(method,"add")==0){
        ultrasonic = MeUltrasonicSensor(port);
        ultrasonic_port = port;
      }
    }else if(strcmp(device,"Limit Switch")==0){
      if(strcmp(method,"add")==0){
        limitSwitch = MeLimitSwitch(port,slot);
        switch_port = port;
        switch_slot = slot;
      }
    }else if(strcmp(device,"poll")==0){
      if(ultrasonic_port>-1){
        serial.print("Ultrasonic/Port");
        serial.print(ultrasonic_port);
        serial.print(" ");
        serial.println(ultrasonic.distanceCm());
      }
      if(switch_port>-1){
        serial.print("Limit Switch/Port");
        serial.print(switch_port);
        serial.print("/Slot");
        serial.print(switch_slot);
        serial.print(" ");
        serial.println(limitSwitch.touched());
      }
    }
  }
  wdt_reset();
}
