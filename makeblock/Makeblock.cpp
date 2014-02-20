#include "Makeblock.h"

#define MeBaseBoard


#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

MePort_Sig mePort[11] = {{NC, NC}, {11, A8}, {13, A11}, {A10, A9}, {1, 0},
    {MISO, SCK}, {A0, A1}, {A2, A3}, {A4, A5}, {6, 7}, {5, 4}
};
#else // else ATmega328
MePort_Sig mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {12, 13}, {8, 2},
    {NC, NC}, {A2, A3}, {NC, A1}, {NC, A0}, {6, 7}, {5, 4}
};

#endif

union{
    byte b[4];
    float fVal;
    long lVal;
}u;

/*        Port       */
MePort::MePort(){
	s1 = mePort[0].s1;
    s2 = mePort[0].s2;
    _port = 0;
}
MePort::MePort(uint8_t port)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
	//The PWM frequency is 976 Hz
#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

TCCR1A =  _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

TCCR3A = _BV(WGM30);
TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

TCCR4B = _BV(CS42) | _BV(CS41) | _BV(CS40);
TCCR4D = 0;

#else if defined(__AVR_ATmega328__) // else ATmega328

TCCR1A = _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

TCCR2A = _BV(WGM21) |_BV(WGM20);
TCCR2B = _BV(CS22);

#endif
}
uint8_t MePort::getPort(){
	return _port;
}
uint8_t MePort::getSlot(){
	return _slot;
}
bool MePort::Dread1()
{
    bool val;
    pinMode(s1, INPUT);
    val = digitalRead(s1);
    return val;
}

bool MePort::Dread2()
{
    bool val;
	pinMode(s2, INPUT);
    val = digitalRead(s2);
    return val;
}

void MePort::Dwrite1(bool value)
{
    pinMode(s1, OUTPUT);
    digitalWrite(s1, value);
}

void MePort::Dwrite2(bool value)
{
    pinMode(s2, OUTPUT);
    digitalWrite(s2, value);
}

int MePort::Aread1()
{
    int val;
    val = analogRead(s1);
    return val;
}

int MePort::Aread2()
{
    int val;
    val = analogRead(s2);
    return val;
}

void MePort::Awrite1(int value)
{   
    analogWrite(s1, value);  
}

void MePort::Awrite2(int value)
{
    analogWrite(s2, value); 
}
void MePort::reset(uint8_t port){
	s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
void MePort::reset(uint8_t port,uint8_t slot){
	s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
    _slot = slot;
}
/*             Wire               */
MeWire::MeWire(uint8_t selector): MePort(){
	_slaveAddress = selector + 1;
}
MeWire::MeWire(uint8_t port, uint8_t selector): MePort(port)
{
    _slaveAddress = selector + 1;
}
void MeWire::begin()
{
    delay(1500);
    Wire.begin();
    write(BEGIN_FLAG, 0x01);
}
bool MeWire::isRunning()
{
    return read(BEGIN_STATE);
}
void MeWire::setSelectorIndex(uint8_t selectorIndex)
{
    write(LS_SET_ST_ADD, selectorIndex);
}

byte MeWire::read(byte dataAddress){
	byte *b={0};
	read(dataAddress,b,1);
	return b[0];
}

void MeWire::read(byte dataAddress,uint8_t *buf,int len)
{
	byte rxByte;
	Wire.beginTransmission(_slaveAddress); // transmit to device
	Wire.write(dataAddress); // sends one byte
	Wire.endTransmission(); // stop transmitting
	delayMicroseconds(1);
	Wire.requestFrom(_slaveAddress,len); // request 6 bytes from slave device
	int index =0;
	while(Wire.available()) // slave may send less than requested
	{
		rxByte = Wire.read(); // receive a byte as character
		buf[index] = rxByte;
		index++;
	}
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
void MeWire::request(byte* writeData,byte*readData,int wlen,int rlen)
{
   
	uint8_t rxByte;
	uint8_t index =0;

	Wire.beginTransmission(_slaveAddress); // transmit to device

	Wire.write(writeData,wlen);

	Wire.endTransmission(); 
	
	delay(10);
	Wire.requestFrom(_slaveAddress,rlen); // request 6 bytes from slave device
	
	while(Wire.available()) // slave may send less than requested
	{
		rxByte = Wire.read(); // receive a byte as character
		readData[index] = rxByte;
		index++; 
	}
}
/*      MeParams       */
MeParams::MeParams()
{
    _root = createObject();
    memset(_root->child, 0, sizeof(MeParamObject));
}
void MeParams::parse(char* s){
	clear();
  char *p=NULL;
  char *v=NULL;
  p = (char*)malloc(20*sizeof(char));
  v = (char*)malloc(40*sizeof(char));
  int i;
  int len = strlen(s);
  int pIndex = 0;
  int vIndex = 0;
  bool pEnd = false;
  bool vEnd = true;
  for(i=0;i<len;i++){
    if(s[i]=='&'){
      pEnd = false;
      vEnd = true;
      setParam(p,v);
      memset(p,0,20);
      memset(v,0,40);
      pIndex = i+1;
    }
    if(s[i]=='='){
      pEnd=true;
      vEnd = false;
      vIndex = i+1;
    }
    if(vEnd==false){
      if(i-vIndex>=0){
        v[i-vIndex]=s[i];
      } 
    }
    if(pEnd==false){
      if(i-pIndex>=0){
       p[i-pIndex]=s[i];
      }
    }
  }
  setParam(p,v);
  memset(p,0,20);
  memset(v,0,40);
  free(p);
  free(v);
}
MeParamObject *MeParams::getParam(const char *string)
{
    MeParamObject *c = _root->child;
    while (c && strcasecmp(c->name, string))
        c = c->next;
    return c;
}
void MeParams::setParam(char *name, char *n)
{
	bool isStr = true;
	double v = atof(n);	
	isStr = v==0;
	int i=0;
	int len = strlen(n);
	for(i=0;i<len;i++){
		if(i==0){
			if(n[i]==43||n[i]==45){
				continue;
			}
		}
		if(n[i]==46){
			continue;
		}
		if(!(n[i]>=48&&n[i]<=57)){
			isStr = true;	
			break;
		}
	}
    deleteParam(name);
    if(isStr) {
        addItemToObject(name, createCharItem(n));
    } else {
        addItemToObject(name, createItem(v));
    }
}
double MeParams::getParamValue(const char *string)
{
    return getParam(string)->value;
}
char *MeParams::getParamCode(const char *string)
{
    return getParam(string)->code;
}
void MeParams::clear()
{
    unsigned char i = 0;
    MeParamObject *c = _root->child;
    MeParamObject *prev;
    while (c){
		prev = c;
    	c = c->next;
    }
    c = prev;
    while(prev){
    	c = prev->prev;
		deleteParam(prev->name);
    	prev = c;
    }

}
void MeParams::deleteParam(char *string)
{	
    deleteItemFromRoot(detachItemFromObject(string));
}
MeParamObject *MeParams::createObject()
{
    MeParamObject *item = (MeParamObject *) malloc(sizeof(MeParamObject));
    if (item) {
        memset(item, 0, sizeof(MeParamObject));
    }
    return item;
}
MeParamObject *MeParams::createItem(double n)
{
    MeParamObject *item = (MeParamObject *) malloc(sizeof(MeParamObject));
    if (item) {
        memset(item, 0, sizeof(MeParamObject));
        item->value = n;
        item->type = 1;
    }
    return item;
}
MeParamObject *MeParams::createCharItem(char *n)
{
    MeParamObject *item = (MeParamObject *) malloc(sizeof(MeParamObject));
    if (item) {
        memset(item, 0, sizeof(MeParamObject));
        item->code = strdup(n);
        item->type = 2;
    }
    return item;
}

void MeParams::addItemToObject(char *string, MeParamObject *item)
{
    if (!item)
        return;
    if (item->name){
    	free(item->name);
    }
        
    item->name = strdup(string);
    MeParamObject *c = _root->child;
    if (!item)
        return;

    if (!c) {
        _root->child = item;
    } else {
        while (c && c->next)
            c = c->next;
        suffixObject(c, item);
    }
}
void MeParams::deleteItemFromRoot(MeParamObject *c)
{
    MeParamObject *next;
    while (c) {
        next = c->next;
        if (c->name) {
            free(c->name);
        }
        if (c->child) {
            deleteItemFromRoot(c->child);
        }
        if(c->code&&c->type==2){
			free(c->code);
		}
		c->type=0;
        free(c);
        c = next;
    }
}
MeParamObject *MeParams::detachItemFromObject( char *string)
{
    unsigned char i = 0;
    MeParamObject *c = _root->child;
    while (c && strcasecmp(c->name, string))
        i++, c = c->next;
    if (c)
        return detachItemFromArray(i);
    return 0;
}
void MeParams::deleteItemFromArray(unsigned char which)
{
    deleteItemFromRoot(detachItemFromArray(which));
}
MeParamObject *MeParams::detachItemFromArray(unsigned char which)
{
    MeParamObject *c = _root->child;
    while (c && which > 0)
        c = c->next, which--;
    if (!c)
        return 0;

    if (c->prev)
        c->prev->next = c->next;
    if (c->next)
        c->next->prev = c->prev;
    if (c == _root->child)
        _root->child = c->next;
    c->prev = c->next = 0;
    return c;
}

void MeParams::suffixObject(MeParamObject *prev, MeParamObject *item)
{
    prev->next = item;
    item->prev = prev;
}

/*             Serial                  */
MeSerial::MeSerial():MePort(),SoftwareSerial(NC,NC){
    _hard = true;
    _scratch = true;
    _polling = false;
}
MeSerial::MeSerial(uint8_t port):MePort(port),SoftwareSerial(mePort[port].s2,mePort[port].s1)
{
	_scratch = false;
    _hard = false;
    _polling = false;
    #if defined(__AVR_ATmega32U4__)
        _polling = getPort()>PORT_4;
        _hard = getPort()==PORT_4;
    #else
    	_hard = getPort()==PORT_5;    
    #endif
}

void MeSerial::begin(long baudrate)
{
    _bitPeriod = 1000000/baudrate;
    if(_hard) {
		#if defined(__AVR_ATmega32U4__)
            _scratch?Serial.begin(baudrate):Serial1.begin(baudrate);
        #else
            Serial.begin(baudrate);
		#endif
    } else {
        SoftwareSerial::begin(baudrate);
    }
}
size_t MeSerial::write(uint8_t byte)
{
    if(_isServoBusy == true)return -1;
    if(_hard){
    	#if defined(__AVR_ATmega32U4__)
            return (_scratch?Serial.write(byte):Serial1.write(byte));
        #else
            return Serial.write(byte);
		#endif
    }else return SoftwareSerial::write(byte);
}
int MeSerial::read()
{
    if(_isServoBusy == true)return -1;
    
    if(_polling){
        int temp = _byte;
        _byte = -1;
        return temp>-1?temp:poll();
    }
    if(_hard){
		#if defined(__AVR_ATmega32U4__)
        	return (_scratch?Serial.read():Serial1.read());
        #else
        	return Serial.read();
		#endif
    }else return SoftwareSerial::read();
}
int MeSerial::available()
{
    if(_polling){
        _byte = poll();
        return _byte>-1?1:0;
    }
    if(_hard){
    	#if defined(__AVR_ATmega32U4__)
        	return (_scratch?Serial.available():Serial1.available());
        #else
        	return Serial.available();
		#endif
    }else return SoftwareSerial::available();
}
bool MeSerial::listen()
{
    if(_hard)
        return true;
    else return SoftwareSerial::listen();
}
bool MeSerial::isListening()
{
    if(_hard)
        return true;
    else return SoftwareSerial::isListening();
}
int MeSerial::poll()
{
    int val = 0;
    int bitDelay = _bitPeriod - clockCyclesToMicroseconds(50);
    if (digitalRead(s2) == LOW) {
        for (int offset = 0; offset < 8; offset++) {
            delayMicroseconds(bitDelay);
            val |= digitalRead(s2) << offset;
        }
        delayMicroseconds(bitDelay);
        return val&0xff;
    }
    return -1;
}
bool MeSerial::paramAvailable()
{
    char c = this->read();
    bool isParse = (millis()-_lastTime)>100&&_index>0;
    if(c > -1||isParse) {
        if(c == '\r'||isParse) {
            char str[_index];
            _cmds[_index] = '\0';
            strcpy(str, _cmds);
            _params.parse(str);
            _index = 0;
            return true;
        } else {
            _cmds[_index] = c;
            _index++;
        }
        
    	_lastTime = millis();
    }
    return false;
}

double MeSerial::getParamValue(char *str)
{
    return _params.getParamValue(str);
}
char *MeSerial::getParamCode(char *str)
{
    return _params.getParamCode(str);
}
MeParams MeSerial::getParams()
{
    return _params;
}
void MeSerial::findParamName(char *str, int len)
{
    byte i = 0;
    for(i = 0; i < len; i++) {
        if(str[i] == '=') {
            char name[i+1];
            memcpy(name, str, i);
            name[i] = '\0';
            char s[len];
            int j;
            for(j = i + 1; j < len; j++) {
                s[j-i-1] = str[j];
            }
            findParamValue(s, len - i - 1, name);
            break;
        }
    }
}
void MeSerial::findParamValue(char *str, int len, char *name)
{
    byte i = 0;
    for(i = 0; i < len; i++) {
        if(str[i] == '&' || str[i] == '\0' || i == len - 1) {
            char v[i+1];
            memcpy(v, str, i);
            v[i] = '\0';

            _params.setParam(name, v);
            if(i < len - 1) {
                char s[len];
                int j;
                for(j = i + 1; j < len; j++) {
                    s[j-i-1] = str[j];
                }
                findParamName(s, len - i - 1);
                break;
            }
        }
    }
}
/*             LineFinder              */
MeLineFinder::MeLineFinder(): MePort(0){
	
}
MeLineFinder::MeLineFinder(uint8_t port): MePort(port)
{

}
int MeLineFinder::readSensors()
{
    int state = S1_IN_S2_IN;
    int s1State = MePort::Dread1();
    int s2State = MePort::Dread2();
    state = ((1 & s1State) << 1) | s2State;
    return state;
}
int MeLineFinder::readSensor1()
{
    return MePort::Dread1();
}
int MeLineFinder::readSensor2()
{
    return MePort::Dread2();
}
/*             LimitSwitch              */
MeLimitSwitch::MeLimitSwitch(): MePort(0)
{
}
MeLimitSwitch::MeLimitSwitch(uint8_t port): MePort(port)
{
    _device = DEV1;
    pinMode(s2,INPUT_PULLUP);
}
MeLimitSwitch::MeLimitSwitch(uint8_t port,uint8_t slot): MePort(port)
{
    reset(port,slot);
    if(getSlot()==DEV2){
        pinMode(s1,INPUT_PULLUP);
    }else{
        pinMode(s2,INPUT_PULLUP);
    }
}
bool MeLimitSwitch::touched()                                                                                                                                                          
{
    if(getSlot()==DEV2){
        pinMode(s1,INPUT_PULLUP);
    }else{
        pinMode(s2,INPUT_PULLUP);
    }
    return getSlot()==DEV2?digitalRead(s1):digitalRead(s2);
}

/*             MotorDriver              */
MeDCMotor::MeDCMotor(): MePort(0)
{

}
MeDCMotor::MeDCMotor(uint8_t port): MePort(port)
{

}
void MeDCMotor::run(int speed)
{
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;

    if(speed >= 0) {
        MePort::Dwrite2(HIGH);
        MePort::Awrite1(speed);
    } else {
        MePort::Dwrite2(LOW);
        MePort::Awrite1(-speed);
    }
}
void MeDCMotor::stop()
{
    MeDCMotor::run(0);
}
/*           UltrasonicSenser                 */
MeUltrasonicSensor::MeUltrasonicSensor(): MePort(0)
{
}

MeUltrasonicSensor::MeUltrasonicSensor(uint8_t port): MePort(port)
{
}

long MeUltrasonicSensor::distanceCm()
{
    long distance = measure();
    return ((distance / 29) >> 1);
}

long MeUltrasonicSensor::distanceInch()
{
    long distance = measure();
    return ((distance / 74) >> 1);
}

long MeUltrasonicSensor::measure()
{
    long duration;
    MePort::Dwrite2(LOW);
    delayMicroseconds(2);
    MePort::Dwrite2(HIGH);	
    delayMicroseconds(10);
    MePort::Dwrite2(LOW);
    pinMode(s2, INPUT);
    duration = pulseIn(s2, HIGH); 
    return duration;
}

/*          shutter       */
MeShutter::MeShutter(): MePort(0){
	
}
MeShutter::MeShutter(uint8_t port): MePort(port)
{
    MePort::Dwrite1(LOW);
    MePort::Dwrite2(LOW);
}
void MeShutter::shotOn()
{
    MePort::Dwrite1(HIGH);
}
void MeShutter::shotOff()
{

    MePort::Dwrite1(LOW);
}
void MeShutter::focusOn()
{
    MePort::Dwrite2(HIGH);
}
void MeShutter::focusOff()
{
    MePort::Dwrite2(LOW);
}

/*           Bluetooth                 */
MeBluetooth::MeBluetooth(): MeSerial(0)
{
}
MeBluetooth::MeBluetooth(uint8_t port): MeSerial(port)
{
}

/*           Infrared Receiver                 */
MeInfraredReceiver::MeInfraredReceiver(): MeSerial(0)
{
	
}
MeInfraredReceiver::MeInfraredReceiver(uint8_t port): MeSerial(port)
{
}
void MeInfraredReceiver::begin()
{
    MeSerial::begin(9600);
}
bool MeInfraredReceiver::buttonState()        // Not available in Switching mode
{
    return !(MePort::Dread1());
}
/*         LED Strip        */
// portNum can ONLY be PORT_1 or PORT_2

MeLedStrip::MeLedStrip(uint8_t port, uint8_t selector): MeWire(port, selector)

{

}
// initialize ledStrip Driver and set the led quantity. (value: 1-60)
void MeLedStrip::begin(int ledCount)
{
    MeWire::begin(); // join i2c bus (address optional for master)
    MeWire::write(LS_LED_COUNT, ledCount);
    reset();
}
void MeLedStrip::autoFlash(int flashSpeed)
{
    MeWire::write(LS_SET_SPEED, flashSpeed);
    MeWire::write(LS_RUN_CTRL, LS_AUTO_FLASH);
}

void MeLedStrip::onceFlash()
{
    MeWire::write(LS_RUN_CTRL, LS_ONCE_FLASH);
}

void MeLedStrip::stopFlash()
{
    MeWire::write(LS_RUN_CTRL, LS_STOP_FLASH);
}

void MeLedStrip::reset()
{
    MeWire::write(LS_RUN_CTRL, LS_RESET);
}

void MeLedStrip::setPixelColor(byte lsNum, byte lsR, byte lsG, byte lsB, byte lsMode)
{
    MeWire::write(LS_SET_PIXEL_R, lsR);
    MeWire::write(LS_SET_PIXEL_G, lsG);
    MeWire::write(LS_SET_PIXEL_B, lsB);
    MeWire::write(LS_SET_PIXEL_NUM, lsNum);
    MeWire::write(LS_RUN_CTRL, lsMode);
}


void MeLedStrip::color_loop()
{
    MeWire::write(LS_RUN_CTRL, LS_COLOR_LOOP);
}

void MeLedStrip::indicators(byte lsNum, byte lsR, byte lsG, byte lsB, byte lsSpd)
{
    MeWire::write(LS_SET_COUNT, lsNum);
    MeWire::write(LS_SET_IN_SPEED, lsSpd);
    MeWire::write(LS_SET_PIXEL_R, lsR);
    MeWire::write(LS_SET_PIXEL_G, lsG);
    MeWire::write(LS_SET_PIXEL_B, lsB);
    MeWire::write(LS_RUN_CTRL, LS_INDICATORS);
}
/*          EncoderMotor        */

MeEncoderMotor::MeEncoderMotor(uint8_t selector):MeWire(selector){
    
}
boolean MeEncoderMotor::setCounter(uint8_t counter,uint8_t slot){
    byte w[4]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x23;
    w[2]=slot;
    w[3]=counter;
    request(w,r,4,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setRatio(float ratio,uint8_t slot){
    byte w[7]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x22;
    w[2]=slot;
    u.fVal = ratio;
    w[3]=u.b[0];
    w[4]=u.b[1];
    w[5]=u.b[2];
    w[6]=u.b[3];
    request(w,r,7,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setPID(float mp,float mi,float md,uint8_t mode,uint8_t slot){
    
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x24;
    w[2]=slot;
    w[4]=mode;
    
    int i;
    
    w[3]=0;
    u.fVal = mp;
    for(i=0;i<4;i++){
        w[5+i]=u.b[0];
    }
    request(w,r,9,4);
    
    w[3]=1;
    u.fVal = mi;
    for(i=0;i<4;i++){
        w[5+i]=u.b[0];
    }
    request(w,r,9,4);
    
    w[3]=2;
    u.fVal = md;
    for(i=0;i<4;i++){
        w[5+i]=u.b[0];
    }
    request(w,r,9,4);
    return r[3]==1;
}
boolean MeEncoderMotor::runWithAngleAndSpeed(long degrees,float speed,uint8_t slot){
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x31;
    w[2]=slot;
    int i;
    u.lVal = degrees;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}
boolean MeEncoderMotor::runWithTurns(float turns,uint8_t slot){
    runWithAngleAndSpeed(turns*360,10,slot);
}
boolean MeEncoderMotor::runWithSpeedAndTime(float speed,long time,uint8_t slot){
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x32;
    w[2]=slot;
    int i;
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.lVal = time;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setCommandFlag(boolean flag,uint8_t slot){
    byte w[4]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x41;
    w[2]=slot;
    w[3]=flag;
    request(w,r,4,4);
    return r[3]==1;
}
float MeEncoderMotor::getCurrentSpeed(uint8_t slot){
    byte w[3]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x51;
    w[2]=slot;
    request(w,r,3,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.fVal;
}
long MeEncoderMotor::getCurrentPosition(uint8_t slot){
    byte w[3]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x52;
    w[2]=slot;
    request(w,r,3,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.lVal;
}
float MeEncoderMotor::getPIDParam(uint8_t type,uint8_t mode,uint8_t slot){
    
    byte w[5]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x53;
    w[2]=slot;
    w[3]=type;
    w[4]=mode;
    request(w,r,5,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.fVal;
    
}
/*          Stepper     */

MeStepperMotor::MeStepperMotor(uint8_t port, uint8_t selector): MeWire(port, selector)
{
}

void MeStepperMotor::begin(byte microStep, long speed, long acceleration)
{
    MeWire::begin(); // join i2c bus (address optional for master)
    delay(10);
    reset();
	delay(10);
    setCurrentPosition(0);
    enable();
    delay(10);
    setMicroStep(microStep);
    delay(10);
    setMaxSpeed(speed);
    delay(10);
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
    *((char *)(&stepperSpeedRead))     = MeWire::read(STP_SPEED_RL1);
    *((char *)(&stepperSpeedRead) + 1) = MeWire::read(STP_SPEED_RL2);
    *((char *)(&stepperSpeedRead) + 2) = MeWire::read(STP_SPEED_RH1);
    *((char *)(&stepperSpeedRead) + 3) = MeWire::read(STP_SPEED_RH2);
    return stepperSpeedRead;
}

long MeStepperMotor::distanceToGo()
{
    *((char *)(&stepperDistanceToGoRead))     = MeWire::read(STP_DIS_TOGO_RL1);
    *((char *)(&stepperDistanceToGoRead) + 1) = MeWire::read(STP_DIS_TOGO_RL2);
    *((char *)(&stepperDistanceToGoRead) + 2) = MeWire::read(STP_DIS_TOGO_RH1);
    *((char *)(&stepperDistanceToGoRead) + 3) = MeWire::read(STP_DIS_TOGO_RH2);
    return stepperDistanceToGoRead;
}

long MeStepperMotor::targetPosition()
{
    *((char *)(&stepperTargetPositionRead))     = MeWire::read(STP_TARGET_POS_RL1);
    *((char *)(&stepperTargetPositionRead) + 1) = MeWire::read(STP_TARGET_POS_RL2);
    *((char *)(&stepperTargetPositionRead) + 2) = MeWire::read(STP_TARGET_POS_RH1);
    *((char *)(&stepperTargetPositionRead) + 3) = MeWire::read(STP_TARGET_POS_RH2);
    return stepperTargetPositionRead;
}

long MeStepperMotor::currentPosition()
{
    *((char *)(&stepperCurrentPositionRead))     = MeWire::read(STP_CURRENT_POS_RL1);
    *((char *)(&stepperCurrentPositionRead) + 1) = MeWire::read(STP_CURRENT_POS_RL2);
    *((char *)(&stepperCurrentPositionRead) + 2) = MeWire::read(STP_CURRENT_POS_RH1);
    *((char *)(&stepperCurrentPositionRead) + 3) = MeWire::read(STP_CURRENT_POS_RH2);
    return stepperCurrentPositionRead;
}

void MeStepperMotor::setCurrentPosition(long stepperCurrentPos)
{
    MeWire::write(STP_CURRENT_POS_L1, *((char *)(&stepperCurrentPos)));
    MeWire::write(STP_CURRENT_POS_L2, *((char *)(&stepperCurrentPos) + 1));
    MeWire::write(STP_CURRENT_POS_H1, *((char *)(&stepperCurrentPos) + 2));
    MeWire::write(STP_CURRENT_POS_H2, *((char *)(&stepperCurrentPos) + 3));
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

/*servo*/

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

//#define NBR_TIMERS        (MAX_SERVOS / SERVOS_PER_TIMER)

static servo_t servos[MAX_SERVOS];                          // static array of servo structures
static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t ServoCount = 0;                                     // the total number of attached servos

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/
static bool isTimerActive(timer16_Sequence_t timer)
{
    // returns true if any servo is active on this timer
    for(uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
        if(SERVO(timer, channel).Pin.isActive == true)
            return true;
    }
    return false;
}
static void finISR(timer16_Sequence_t timer)
{
    //disable use of the given timer
#if defined WIRING   // Wiring
    if(timer == _timer1) {
#if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
        TIMSK1 &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
#else
        TIMSK &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
#endif
        timerDetach(TIMER1OUTCOMPAREA_INT);
    } else if(timer == _timer3) {
#if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
        TIMSK3 &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
#else
        ETIMSK &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
#endif
        timerDetach(TIMER3OUTCOMPAREA_INT);
    }
#else
    //For arduino - in future: call here to a currently undefined function to reset the timer
#endif
}
static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t *OCRnA)
{
    if( Channel[timer] < 0 ) {
        *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer
        _isServoBusy = false;
    } else {
        if( SERVO_INDEX(timer, Channel[timer]) < ServoCount && SERVO(timer, Channel[timer]).Pin.isActive == true ) {
            digitalWrite( SERVO(timer, Channel[timer]).Pin.nbr, LOW); // pulse this channel low if activated
            _isServoBusy = false;
        }
    }

    Channel[timer]++;    // increment to the next channel
    if( SERVO_INDEX(timer, Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
        *OCRnA = *TCNTn + SERVO(timer, Channel[timer]).ticks;
        if(SERVO(timer, Channel[timer]).Pin.isActive == true) {   // check if activated
            digitalWrite( SERVO(timer, Channel[timer]).Pin.nbr, HIGH); // its an active channel so pulse it high
            _isServoBusy = true;

        }

    } else {

        // finished all channels so wait for the refresh period to expire before starting over
        if( ((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
            *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
        else
            *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed


        Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel

    }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform
// Interrupt handlers for Arduino
#if defined(_useTimer1)
ISR(TIMER1_COMPA_vect)
{
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif
#if defined(_useTimer3)
ISR(TIMER3_COMPA_vect)
{
    handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif

#if defined(_useTimer4)
ISR(TIMER4_COMPA_vect)
{
    handle_interrupts(_timer4, &TCNT4, &OCR4A);
}
#endif

#if defined(_useTimer5)
ISR(TIMER5_COMPA_vect)
{
    handle_interrupts(_timer5, &TCNT5, &OCR5A);
}
#endif

#elif defined WIRING
// Interrupt handlers for Wiring
#if defined(_useTimer1)
void Timer1Service()
{
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif
#if defined(_useTimer3)
void Timer3Service()
{
    handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif
#endif


static void initISR(timer16_Sequence_t timer)
{
#if defined (_useTimer1)
    if(timer == _timer1) {
        TCCR1A = 0;             // normal counting mode
        TCCR1B = _BV(CS11);     // set prescaler of 8
        TCNT1 = 0;              // clear the timer count
#if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIFR |= _BV(OCF1A);      // clear any pending interrupts;
        TIMSK |=  _BV(OCIE1A) ;  // enable the output compare interrupt
#else
        // here if not ATmega8 or ATmega128
        TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
        TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt
#endif
#if defined(WIRING)
        timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
#endif
    }
#endif


#if defined (_useTimer3)
    if(timer == _timer3) {
        TCCR3A = 0;             // normal counting mode
        TCCR3B = _BV(CS31);     // set prescaler of 8
        TCNT3 = 0;              // clear the timer count
#if defined(__AVR_ATmega128__)
        TIFR |= _BV(OCF3A);     // clear any pending interrupts;
        ETIMSK |= _BV(OCIE3A);  // enable the output compare interrupt
#else
        TIFR3 = _BV(OCF3A);     // clear any pending interrupts;
        TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt
#endif
#if defined(WIRING)
        timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only
#endif
    }
#endif

#if defined (_useTimer4)
    if(timer == _timer4) {
        TCCR4A = 0;             // normal counting mode
        TCCR4B = _BV(CS41);     // set prescaler of 8
        TCNT4 = 0;              // clear the timer count
        TIFR4 = _BV(OCF4A);     // clear any pending interrupts;
        TIMSKEY4 =  _BV(OCIE4A) ; // enable the output compare interrupt
    }
#endif

#if defined (_useTimer5)
    if(timer == _timer5) {
        TCCR5A = 0;             // normal counting mode
        TCCR5B = _BV(CS51);     // set prescaler of 8
        TCNT5 = 0;              // clear the timer count
        TIFR5 = _BV(OCF5A);     // clear any pending interrupts;
        TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt
    }
#endif
}

/****************** end of static functions ******************************/
MeServo::MeServo(): MePort(0){
	
}
MeServo::MeServo(uint8_t port, uint8_t device): MePort(port)
{
    servoPin = ( device == DEV1 ? s2 : s1);
    if( ServoCount < MAX_SERVOS) {
        this->servoIndex = ServoCount++;                    // assign a servo index to this instance
        servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
    } else
        this->servoIndex = INVALID_SERVO ;  // too many servos
}

uint8_t MeServo::begin()
{
    return this->begin(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t MeServo::begin(int min, int max)
{
    if(this->servoIndex < MAX_SERVOS ) {
        pinMode( servoPin, OUTPUT) ;                                   // set servo pin to output
        servos[this->servoIndex].Pin.nbr = servoPin;
        // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
        this->min  = (MIN_PULSE_WIDTH - min) / 4; //resolution of min/max is 4 uS
        this->max  = (MAX_PULSE_WIDTH - max) / 4;
        // initialize the timer if it has not already been initialized
        timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
        if(isTimerActive(timer) == false)
            initISR(timer);
        servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
    }
    return this->servoIndex ;
}

void MeServo::detach()
{
    servos[this->servoIndex].Pin.isActive = false;
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false) {
        finISR(timer);
    }
}

void MeServo::write(int value)
{
    int delayTime = abs(value - this->read());
    this->begin();
    if(value < MIN_PULSE_WIDTH) {
        // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
        if(value < 0) value = 0;
        if(value > 180) value = 180;
        value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
    }
    this->writeMicroseconds(value);
    //delay(delayTime);
    //this->detach();
}

void MeServo::writeMicroseconds(int value)
{
    // calculate and store the values for the given channel
    byte channel = this->servoIndex;
    if( (channel < MAX_SERVOS) ) { // ensure channel is valid
        if( value < SERVO_MIN() )          // ensure pulse width is valid
            value = SERVO_MIN();
        else if( value > SERVO_MAX() )
            value = SERVO_MAX();

        value = value - TRIM_DURATION;
        value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

        uint8_t oldSREG = SREG;
        cli();
        servos[channel].ticks = value;
        SREG = oldSREG;
    }
}

int MeServo::read()   // return the value as degrees
{
    return  map( this->readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int MeServo::readMicroseconds()
{
    unsigned int pulsewidth;
    if( this->servoIndex != INVALID_SERVO )
        pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;   // 12 aug 2009
    else
        pulsewidth  = 0;

    return pulsewidth;
}

bool MeServo::attached()
{
    return servos[this->servoIndex].Pin.isActive ;
}


/*Me4Button*/
Me4Button::Me4Button() : MePort(0){
	
}
Me4Button::Me4Button(uint8_t port) : MePort(port)
{
    _toggleState = NULL_KEY;
    _oldState = NULL_KEY_VALUE;
	_prevPressedState = 0;
    _pressedState = NULL_KEY;
    _releasedState = NULL_KEY;
    _heldState = NULL_KEY;
    _heldTime = millis();
}

bool Me4Button::update()
{
	if(millis()-_heldTime<10){
		return false;
	}
    uint8_t update_temp;
    uint16_t newState = 0;
	uint16_t t=0;
	uint16_t tmax = 0;
    for(uint8_t i = 0; i < 16; i++) {
		t = MePort::Aread2();
		if(i<4){
			continue;
		}
        newState += t;
		if(tmax<t){
			tmax = t;
		}
    }
	newState -=tmax;
    newState /= 11;
    if(abs(newState-_oldState) > 40)update_temp = 1;
	else update_temp = 0;	
    if (update_temp) {
		if(newState > KEY4_VALUE+50) { //released?
	            _toggleState = !_toggleState;
	            if(_oldState < KEY1_VALUE+5)_releasedState = KEY1;
	            else if(_oldState > KEY2_VALUE - 5 && _oldState < KEY2_VALUE + 5)_releasedState = KEY2;
	            else if(_oldState > KEY3_VALUE - 5 && _oldState < KEY3_VALUE + 5)_releasedState = KEY3;
	            else if(_oldState > KEY4_VALUE - 5 && _oldState < KEY4_VALUE + 5)_releasedState = KEY4;
				

				_pressedState = NULL_KEY;
	    }else{
	        if(newState < KEY1_VALUE+5)_pressedState = KEY1;
	        else if((newState > KEY2_VALUE - 5) && (newState < KEY2_VALUE + 5))_pressedState = KEY2;
	        else if((newState > KEY3_VALUE - 5) && (newState < KEY3_VALUE + 5))_pressedState = KEY3;
	        else if((newState > KEY4_VALUE - 5) && (newState < KEY4_VALUE + 5))_pressedState = KEY4;	
		}
        //delay(10); // debouncing
    } else {
       	if(_oldState < (KEY4_VALUE + 10))_pressedState = NULL_KEY;
		if(newState > KEY4_VALUE+5 && _oldState > KEY4_VALUE+5){
			_releasedState = NULL_KEY;
		}
    }
    _oldState = newState;
	_heldTime = millis();
	return true;
}


uint8_t Me4Button::pressed()
{
	update();
	uint8_t returnKey = _pressedState;
	_pressedState = NULL_KEY;
	return returnKey;
}

uint8_t Me4Button::released()
{
	update();
	uint8_t returnKey = _releasedState;
	_releasedState = NULL_KEY;
	return returnKey;
}

/*      Joystick        */
MeJoystick::MeJoystick() : MePort(0){}
MeJoystick::MeJoystick(uint8_t port) : MePort(port){}

int MeJoystick::readX()
{	
	int mapX = map(MePort::Aread1(),1,980,-255,255);
	return abs(mapX)<15?0:mapX ;
}

int MeJoystick::readY()
{
    
    int mapY = map(MePort::Aread2(),24,980,-255,255);
	return abs(mapY)<15?0:mapY ;
}

float MeJoystick::angle(){
	return atan2(readY(),readX())*180.0/PI;
}

float MeJoystick::strength(){
	long dx = abs(readX());
	long dy = abs(readY());
	long dist = dx*dx+dy*dy;
	return min(1.0,sqrt(dist)/255.0);
}

/*      Light Sensor        */
MeLightSensor::MeLightSensor() : MePort(0){}
MeLightSensor::MeLightSensor(uint8_t port) : MePort(port){}
int MeLightSensor::read()
{	
	return MePort::Aread2();
}

void MeLightSensor::lightOn()
{	
	MePort::Dwrite1(HIGH);
}

void MeLightSensor::lightOff()
{	
	MePort::Dwrite1(LOW);
}

float MeLightSensor::strength()
{
    
    return map(MePort::Aread2(),0,1023,0,1023);
}

/*      Sound Sensor        */
MeSoundSensor::MeSoundSensor() : MePort(0){}
MeSoundSensor::MeSoundSensor(uint8_t port) : MePort(port){}
bool MeSoundSensor::Dread()
{	
	return MePort::Dread1();
}

int MeSoundSensor::Aread()
{
    
    return MePort::Aread2();
}

