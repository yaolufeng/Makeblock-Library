#include "Makeblock.h"

MePort_t mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {13, 12}, {2, 8},
                      {19, 18}, {17, 16}, {15,NC}, {14, NC}, {5,4}, {6, 7}}; 

/*        Port       */
MePort::MePort(uint8_t _port)
{
	s1 = mePort[_port].s1;
    s2 = mePort[_port].s2;
}

/*      Digital Input       */

MeDigital::MeDigital(uint8_t _port):MePort(_port)
{
    pinMode(s1, INPUT);
    pinMode(s2, INPUT);
}

boolean MeDigital::read1() {
    boolean val;
    val = digitalRead(s1);         
    return val;
}

boolean MeDigital::read2() {
    boolean val;
    val = digitalRead(s2);         
    return val;
}
/*      Analog Input       */

MeAnalog::MeAnalog(uint8_t _port):MePort(_port)
{
    pinMode(s1, INPUT);
    pinMode(s2, INPUT);
}

int MeAnalog::read1() {
    int val;
    val = analogRead(s1);         
    return val;
}

int MeAnalog::read2() {
    int val;
    val = analogRead(s2);         
    return val;
}
/*      Output       */

MeOutput::MeOutput(uint8_t _port):MePort(_port)
{
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
}

void MeOutput::write1(int value) {
    if(value<=1023&&value>=-1023){
		analogWrite(s1,value*0.25);         
    }
}

void MeOutput::write2(int value) {
    if(value<=1023&&value>=-1023){
		analogWrite(s2,value*0.25);         
    }
}
/*             Wire               */
MeWire::MeWire(uint8_t _port):MePort(_port)
{
}
void MeWire::begin(int slaveAddress)
{
	_slaveAddress = slaveAddress;
	Wire.begin();
}
byte MeWire::read(byte dataAddress)
{
  byte rxByte;
  Wire.beginTransmission(_slaveAddress); // transmit to device
  Wire.write(dataAddress); // sends one byte
  Wire.endTransmission(); // stop transmitting
  delayMicroseconds(1);
  Wire.requestFrom(_slaveAddress,1); // request 6 bytes from slave device
  while(Wire.available()) // slave may send less than requested
    return rxByte = Wire.read(); // receive a byte as character
  return 0;
}

void MeWire::write(byte dataAddress, byte data)
{
  Wire.beginTransmission(_slaveAddress); // transmit to device
  Wire.write(dataAddress); // sends one byte
  Wire.endTransmission(); // stop transmitting
  
  Wire.beginTransmission(_slaveAddress); // transmit to device
  Wire.write(data); // sends one byte
  Wire.endTransmission(); // stop transmitting
}


/*             Serial                  */
MeSerial::MeSerial(uint8_t _port):MePort(_port),swSerial(s1,s2){

}
void MeSerial::begin(long baudrate){
	swSerial.begin(baudrate);
}
size_t MeSerial::write(uint8_t byte){
	return swSerial.write(byte);
}
int MeSerial::read(){
	return swSerial.read();
}
int MeSerial::available(){
	return swSerial.available();
}
/*             LineFinder              */
MeLineFinder::MeLineFinder(uint8_t _port):MeDigital(_port){

}
int MeLineFinder::readSensors(){
	int state = S1_IN_S2_IN;
	int s1State = MeDigital::read1();
	int s2State = MeDigital::read2();
	state = ((1&s1State)<<1)|s2State;
	return state;
}
int MeLineFinder::readSensor1(){
	return MeDigital::read1();
}
int MeLineFinder::readSensor2(){
	return MeDigital::read2();
}
/*             LimitSwitch              */
MeLimitSwitch::MeLimitSwitch(uint8_t _port):MeDigital(_port){
}
boolean MeLimitSwitch::on(){
	return MeDigital::read1();
}
boolean MeLimitSwitch::off(){
	return MeDigital::read2();
}
boolean MeLimitSwitch::touched(){
	return MeDigital::read1();
}
/*             MotorDriver              */
MeDCMotor::MeDCMotor(uint8_t _port):MeOutput(_port){

}
void MeDCMotor::run(int speed){
	speed = speed>1023? 1023:speed;
	speed = speed<-1023?-1023:speed;

	if(speed>=0)
	{
		MeOutput::write2(1023);
		MeOutput::write1(speed);
	}
	else
	{
		MeOutput::write2(0);
		MeOutput::write1(-speed);
	}
}
void MeDCMotor::stop(){
	MeOutput::write1(0);
}
/*           UltrasonicSenser                 */
MeUltrasonicSensor::MeUltrasonicSensor(uint8_t _port):MeOutput(_port){
}
long MeUltrasonicSensor::distanceCm(){
	long distance = MeUltrasonicSensor::measure();
	return ((distance/29)>>1);
}
long MeUltrasonicSensor::distanceInch(){
	long distance = measure();
	return ((distance/74)>>1);
}
long MeUltrasonicSensor::measure(){
	long duration;
	pinMode(s2, OUTPUT);
	MeOutput::write1(0);
	delayMicroseconds(2);
	MeOutput::write1(1023);
	delayMicroseconds(10);
	MeOutput::write1(0);
	pinMode(s1,INPUT);
	duration = pulseIn(s1,HIGH);
	return duration;
}
/*          shutter       */
MeShutter::MeShutter(uint8_t _port):MeOutput(_port)
{
	MeOutput::write1(1023);
	MeOutput::write2(1023);
}
void MeShutter::shutOn(){
	MeOutput::write1(0);
}
void MeShutter::shutOff(){

	MeOutput::write1(1023);
}
void MeShutter::focusOn(){
	MeOutput::write2(0);
}
void MeShutter::focusOff(){
	MeOutput::write2(1023);
}

/*           Bluetooth                 */
MeBluetooth::MeBluetooth(uint8_t _port):MeSerial(_port)
{
}
void MeBluetooth::begin(long baudrate){
	MeSerial::begin(baudrate);
}
size_t MeBluetooth::write(uint8_t byte){
	return MeSerial::write(byte);
}
int MeBluetooth::read(){
	return MeSerial::read();
}
int MeBluetooth::available(){
	return MeSerial::available();
}
/*           Infrared Receiver                 */
MeInfraredReceiver::MeInfraredReceiver(uint8_t _port):MeSerial(_port)
{
}
void MeInfraredReceiver::begin(){
	MeSerial::begin(9600);
}
char MeInfraredReceiver::read(){
	return MeSerial::read();
}
int MeInfraredReceiver::available(){
	return MeSerial::available();
}
/*           Servo                 */
MeServo::MeServo(uint8_t _port,uint8_t _device):MePort(_port)
{
	int x=DEV1;
	servoPin =( _device == x ? s1 : s2);
	
	// = _device == DEV1 ? s1 : s2;
}
uint8_t MeServo::begin(){
	return servo.attach(servoPin);
}
uint8_t MeServo::begin(int min, int max)
{
	return servo.attach(servoPin,min,max);
}
int MeServo::read(){
	return servo.read();
}
void MeServo::write(int value)
{
	return servo.write(value);
}
void MeServo::detach(){
	servo.detach();
}
/*         LED Strip        */
// portNum can ONLY be PORT_1 or PORT_2
MeLedStrip::MeLedStrip(uint8_t _port):MeWire(_port)
{

}
// initialize ledStrip Driver and set the led quantity. (value: 1-60)
void MeLedStrip::begin(int ledCount){
	MeWire::begin(0x05); // join i2c bus (address optional for master)
    MeWire::write(LS_LED_COUNT, ledCount);
    reset();
}
void MeLedStrip::autoFlash(int flashSpeed){
	MeWire::write(LS_SET_SPEED, flashSpeed);
	MeWire::write(LS_RUN_CTRL, LS_AUTO_FLASH);
}

void MeLedStrip::onceFlash(){
	MeWire::write(LS_RUN_CTRL, LS_ONCE_FLASH);
}

void MeLedStrip::stopFlash(){
	MeWire::write(LS_RUN_CTRL, LS_STOP_FLASH);
}


void MeLedStrip::reset(){
	MeWire::write(LS_RUN_CTRL, LS_RESET);
}


boolean MeLedStrip::readState(){
	if(MeWire::read(LS_RUN_STATE))
		return true;
	  else
		return false;
}

void MeLedStrip::setPixelColor(byte lsNum, byte lsR,byte lsG, byte lsB, byte lsMode){
	MeWire::write(LS_SET_PIXEL_R, lsR);
	MeWire::write(LS_SET_PIXEL_G, lsG);
	MeWire::write(LS_SET_PIXEL_B, lsB);
	MeWire::write(LS_SET_PIXEL_NUM, lsNum);
	MeWire::write(LS_RUN_CTRL, lsMode);
}


void MeLedStrip::color_loop(){
	MeWire::write(LS_RUN_CTRL, LS_COLOR_LOOP);
}

void MeLedStrip::indicators(byte lsNum, byte lsR, byte lsG, byte lsB, byte lsSpd){
	MeWire::write(LS_SET_COUNT, lsNum);
    MeWire::write(LS_SET_IN_SPEED, lsSpd);
    MeWire::write(LS_SET_PIXEL_R, lsR);
	MeWire::write(LS_SET_PIXEL_G, lsG);
	MeWire::write(LS_SET_PIXEL_B, lsB);
	MeWire::write(LS_RUN_CTRL, LS_INDICATORS);
}
/*			Stepper		*/
MeStepperMotor::MeStepperMotor(uint8_t _port):MeWire(_port)
{
}

void MeStepperMotor::begin(byte microStep,long speed,long acceleration)
{
    MeWire::begin(0x04); // join i2c bus (address optional for master)
    setCurrentPosition(0);
	enable();
	setMicroStep(microStep);
	setMaxSpeed(speed);
	setAcceleration(acceleration);
}

void MeStepperMotor::setMicroStep(byte microStep)
{
	MeWire::write(STP_MS_CTRL, microStep);
}

void MeStepperMotor::reset()
{
	MeWire::write(STP_RUN_CTRL, STP_RESET_CTRL);
}

void MeStepperMotor::moveTo(long stepperMoveTo)
{
	MeWire::write(STP_MOVE_TO_L1, *((char *)(&stepperMoveTo)));
	MeWire::write(STP_MOVE_TO_L2, *((char *)(&stepperMoveTo) + 1));
	MeWire::write(STP_MOVE_TO_H1, *((char *)(&stepperMoveTo) + 2));
	MeWire::write(STP_MOVE_TO_H2, *((char *)(&stepperMoveTo) + 3));
}

void MeStepperMotor::move(long stepperMove)
{
	MeWire::write(STP_MOVE_L1, *((char *)(&stepperMove)));
	MeWire::write(STP_MOVE_L2, *((char *)(&stepperMove) + 1));
	MeWire::write(STP_MOVE_H1, *((char *)(&stepperMove) + 2));
	MeWire::write(STP_MOVE_H2, *((char *)(&stepperMove) + 3));
}

void MeStepperMotor::runSpeed()
{
	MeWire::write(STP_RUN_CTRL, STP_RUN_SPEED);
}

void MeStepperMotor::setMaxSpeed(long stepperMaxSpeed)
{
	MeWire::write(STP_MAX_SPEED_L1, *((char *)(&stepperMaxSpeed)));
	MeWire::write(STP_MAX_SPEED_L2, *((char *)(&stepperMaxSpeed) + 1));
	MeWire::write(STP_MAX_SPEED_H1, *((char *)(&stepperMaxSpeed) + 2));
	MeWire::write(STP_MAX_SPEED_H2, *((char *)(&stepperMaxSpeed) + 3));
}

void MeStepperMotor::setAcceleration(long stepperAcceleration)
{
	MeWire::write(STP_ACC_L1, *((char *)(&stepperAcceleration)));
	MeWire::write(STP_ACC_L2, *((char *)(&stepperAcceleration) + 1));
	MeWire::write(STP_ACC_H1, *((char *)(&stepperAcceleration) + 2));
	MeWire::write(STP_ACC_H2, *((char *)(&stepperAcceleration) + 3));
}

void MeStepperMotor::setSpeed(long stepperSpeed)
{
	MeWire::write(STP_SPEED_L1, *((char *)(&stepperSpeed)));
	MeWire::write(STP_SPEED_L2, *((char *)(&stepperSpeed) + 1));
	MeWire::write(STP_SPEED_H1, *((char *)(&stepperSpeed) + 2));
	MeWire::write(STP_SPEED_H2, *((char *)(&stepperSpeed) + 3));
}

long MeStepperMotor::speed()
{
	*((char *)(&stepperSpeedRead)) = MeWire::read(STP_SPEED_RL1);
	*((char *)(&stepperSpeedRead)+1) = MeWire::read(STP_SPEED_RL2);
	*((char *)(&stepperSpeedRead)+2) = MeWire::read(STP_SPEED_RH1);
	*((char *)(&stepperSpeedRead)+3) = MeWire::read(STP_SPEED_RH2);
	return stepperSpeedRead;
}

long MeStepperMotor::distanceToGo()
{
	*((char *)(&stepperDistanceToGoRead)) = MeWire::read(STP_DIS_TOGO_RL1);
	*((char *)(&stepperDistanceToGoRead)+1) = MeWire::read(STP_DIS_TOGO_RL2);
	*((char *)(&stepperDistanceToGoRead)+2) = MeWire::read(STP_DIS_TOGO_RH1);
	*((char *)(&stepperDistanceToGoRead)+3) = MeWire::read(STP_DIS_TOGO_RH2);
	return stepperDistanceToGoRead;
}

long MeStepperMotor::targetPosition()
{
	*((char *)(&stepperTargetPositionRead)) = MeWire::read(STP_TARGET_POS_RL1);
	*((char *)(&stepperTargetPositionRead)+1) = MeWire::read(STP_TARGET_POS_RL2);
	*((char *)(&stepperTargetPositionRead)+2) = MeWire::read(STP_TARGET_POS_RH1);
	*((char *)(&stepperTargetPositionRead)+3) = MeWire::read(STP_TARGET_POS_RH2);
	return stepperTargetPositionRead;
}

long MeStepperMotor::currentPosition()
{
	*((char *)(&stepperCurrentPositionRead)) = MeWire::read(STP_CURRENT_POS_RL1);
	*((char *)(&stepperCurrentPositionRead)+1) = MeWire::read(STP_CURRENT_POS_RL2);
	*((char *)(&stepperCurrentPositionRead)+2) = MeWire::read(STP_CURRENT_POS_RH1);
	*((char *)(&stepperCurrentPositionRead)+3) = MeWire::read(STP_CURRENT_POS_RH2);
	return stepperCurrentPositionRead;
}

void MeStepperMotor::setCurrentPosition(long stepperCuttentPos)
{
	MeWire::write(STP_CURRENT_POS_L1, *((char *)(&stepperCuttentPos)));
	MeWire::write(STP_CURRENT_POS_L2, *((char *)(&stepperCuttentPos) + 1));
	MeWire::write(STP_CURRENT_POS_H1, *((char *)(&stepperCuttentPos) + 2));
	MeWire::write(STP_CURRENT_POS_H2, *((char *)(&stepperCuttentPos) + 3));
}

void MeStepperMotor::enable()
{
	MeWire::write(STP_EN_CTRL, STP_ENABLE);
}

void MeStepperMotor::disable()
{
	MeWire::write(STP_EN_CTRL, STP_DISABLE);
}

void MeStepperMotor::run()
{
	MeWire::write(STP_RUN_CTRL, STP_RUN);
}

void MeStepperMotor::stop()
{
	MeWire::write(STP_RUN_CTRL, STP_STOP);
}

void MeStepperMotor::wait()
{
	MeWire::write(STP_RUN_CTRL, STP_WAIT);
}

boolean MeStepperMotor::readState()
{
	if(MeWire::read(STP_RUN_STATE))
		return true;
	else
		return false;
}