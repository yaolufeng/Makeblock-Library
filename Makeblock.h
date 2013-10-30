#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>


#ifndef Makeblock_h
#define Makeblock_h
#define NC -1

#define PORT_1 0x01;
#define PORT_2 0x02;
#define PORT_3 0x03;
#define PORT_4 0x04;
#define PORT_5 0x05;
#define PORT_6 0x06;
#define PORT_7 0x07;
#define PORT_8 0x08;
#define M1     0x09;
#define M2     0xa0;

#define DEV1 1;
#define DEV2 2;

typedef struct {
  uint8_t s1;
  uint8_t s2;
}MePort_t;
extern MePort_t mePort[11];//mePort[0] is nonsense




//states of two linefinder sensors
#define	S1_IN_S2_IN 0x00 //sensor1 and sensor2 are both inside of black line
#define	S1_IN_S2_OUT 0x01 //sensor1 is inside of black line and sensor is outside of black line
#define	S1_OUT_S2_IN 0x02 //sensor1 is outside of black line and sensor is inside of black line 
#define	S1_OUT_S2_OUT 0x03 //sensor1 is outside of black line and sensor is outside of black line

//ledstrip
//address table
#define LS_RUN_STATE 0x91
#define LS_CURRENT 0x92
#define LS_GET_PIXEL_R 0x93
#define LS_GET_PIXEL_G 0x94
#define LS_GET_PIXEL_B 0x95
#define LS_GET_PIXEL_NUM 0x96

#define LS_SET_PIXEL_NUM 0x02
#define LS_SET_PIXEL_R 0x03
#define LS_SET_PIXEL_G 0x04
#define LS_SET_PIXEL_B 0x05
#define LS_SET_SPEED 0x06
#define LS_SET_COUNT 0x07
#define LS_SET_IN_SPEED 0x08

#define LS_RUN_CTRL 0x1A
#define LS_LED_COUNT                 0x1B

//data table
#define LS_NO_FLASH                        0x00
#define LS_STOP_FLASH                0x00
#define LS_AUTO_FLASH                0x01
#define LS_ONCE_FLASH                0x02
#define LS_CLOSE                                0x04
#define LS_RESET                                0x05
#define LS_COLOR_LOOP                0x06
#define LS_INDICATORS 0x07
#define LS_ALL_PIXEL                 0xFE

#define MAX_BRI                                        200
#define MIN_BRI                                        0
#define IN_SPEED                                2


//stepper
//address table
#define STP_RUN_STATE 0x91
#define STP_SPEED_RL1 0x92
#define STP_SPEED_RL2 0x93
#define STP_SPEED_RH1 0x94
#define STP_SPEED_RH2 0x95
#define STP_DIS_TOGO_RL1 0x96
#define STP_DIS_TOGO_RL2 0x97
#define STP_DIS_TOGO_RH1 0x98
#define STP_DIS_TOGO_RH2 0x99
#define STP_TARGET_POS_RL1 0x9A
#define STP_TARGET_POS_RL2 0x9B
#define STP_TARGET_POS_RH1 0x9C
#define STP_TARGET_POS_RH2 0x9D
#define STP_CURRENT_POS_RL1 0x9E
#define STP_CURRENT_POS_RL2 0x9F
#define STP_CURRENT_POS_RH1 0xA0
#define STP_CURRENT_POS_RH2 0xA1

#define STP_ACC_L1 0x01 //This is an expensive call since it requires a square root to be calculated. Don't call more ofthen than needed.
#define STP_ACC_L2 0x02
#define STP_ACC_H1 0x03
#define STP_ACC_H2 0x04
#define STP_MAX_SPEED_L1 0x05
#define STP_MAX_SPEED_L2 0x06
#define STP_MAX_SPEED_H1 0x07
#define STP_MAX_SPEED_H2 0x08
#define STP_SPEED_L1 0x09
#define STP_SPEED_L2 0x0A
#define STP_SPEED_H1 0x0B
#define STP_SPEED_H2 0x0C
#define STP_MOVE_TO_L1 0x0D
#define STP_MOVE_TO_L2 0x0E
#define STP_MOVE_TO_H1 0x0F
#define STP_MOVE_TO_H2 0x10
#define STP_MOVE_L1 0x11
#define STP_MOVE_L2 0x12
#define STP_MOVE_H1 0x13
#define STP_MOVE_H2 0x14
#define STP_CURRENT_POS_L1 0x15
#define STP_CURRENT_POS_L2 0x16
#define STP_CURRENT_POS_H1 0x17
#define STP_CURRENT_POS_H2 0x18

#define STP_RUN_CTRL 0x1C
#define STP_EN_CTRL 0x1D
#define STP_SLEEP_CTRL 0x1F
#define STP_MS_CTRL 0x20

//data table
#define STP_RUN 0x01
#define STP_STOP 0x02
#define STP_WAIT 0x03
#define STP_RESET_CTRL 0x04
#define STP_RUN_SPEED 0x05
#define STP_TRUE 0x01
#define STP_FALSE 0x00
#define STP_ENABLE 0x01
#define STP_DISABLE 0x02
#define STP_FULL 0x01
#define STP_HALF 0x02
#define STP_QUARTER 0x04
#define STP_EIGHTH 0x08
#define STP_SIXTEENTH 0x16

//NEC Code table
#define IR_BUTTON_POWER 0x45
#define IR_BUTTON_MENU 0x47
#define IR_BUTTON_TEST 0x44
#define IR_BUTTON_PLUS 0x40
#define IR_BUTTON_RETURN 0x43
#define IR_BUTTON_PREVIOUS 0x07
#define IR_BUTTON_PLAY 0x15
#define IR_BUTTON_NEXT 0x09
#define IR_BUTTON_MINUS 0x19
#define IR_BUTTON_CLR 0x0D
#define IR_BUTTON_0 0x16
#define IR_BUTTON_1 0x0C
#define IR_BUTTON_2 0x18
#define IR_BUTTON_3 0x5E
#define IR_BUTTON_4 0x08
#define IR_BUTTON_5 0x1C
#define IR_BUTTON_6 0x5A
#define IR_BUTTON_7 0x42
#define IR_BUTTON_8 0x52
#define IR_BUTTON_9 0x4A

class MePort
{
	public:
		MePort(uint8_t _port);
	protected:
		uint8_t s1;
		uint8_t s2;
};
class MeDigital:public MePort
{
	public:
		MeDigital(uint8_t _port);
		boolean read1();
		boolean read2();
};
class MeAnalog:public MePort
{
	public:
		MeAnalog(uint8_t _port);
		int read1();
		int read2();
};
class MeOutput:public MePort
{
	public:
		MeOutput(uint8_t _port);
		void write1(int value);
		void write2(int value);
};
class MeSerial:public MePort
{
	public:
		MeSerial(uint8_t _port);
		void begin(long baudrate);
		size_t write(uint8_t byte);
		int read();
		int available();
	protected:
		SoftwareSerial swSerial;
};
class MeWire:public MePort
{
	public:
		MeWire(uint8_t _port);
		void begin(int slaveAddress);
		byte read(byte dataAddress);
        void write(byte dataAddress, byte data);
	protected:
		int _slaveAddress;
};
//--------------modules---------------//


class MeLineFinder:public MeDigital
{
	public:
		MeLineFinder(uint8_t _port);
		int readSensors();
		int readSensor1();
		int readSensor2();
};
class MeLimitSwitch:public MeDigital
{
	public:
		MeLimitSwitch(uint8_t _port);
		boolean on();
		boolean off();	
		boolean touched();		
};
class MeUltrasonicSensor:public MeOutput
{
	public :
		MeUltrasonicSensor(uint8_t _port);
		long distanceCm();
		long distanceInch();
		long measure();
};
class MeDCMotor:public MeOutput
{
	public:
		MeDCMotor(uint8_t _port);
		void run(int speed);
		void stop();
};
class MeShutter:public MeOutput
{
	public:
		MeShutter(uint8_t _port);
		void shutOn();
		void shutOff();
		void focusOn();
		void focusOff();
};
class MeBluetooth:public MeSerial
{
	public:
		MeBluetooth(uint8_t _port);
		void begin(long baudrate);
		size_t write(uint8_t byte);
		int read();
		int available();
};
class MeServo:public MePort
{
	public:
		MeServo(uint8_t _port,uint8_t _device);
		uint8_t begin();
		uint8_t begin(int min, int max);
		int read();
		void write(int value);
		void detach();
	protected:
		int servoPin;
		Servo servo;
};
class MeInfraredReceiver:public MeSerial
{
	public:
		MeInfraredReceiver(uint8_t _port);
		void begin();
		char read();
		int available();
};

class MeLedStrip :public MeWire
{
	public:
        // portNum can ONLY be PORT_1 or PORT_2
        MeLedStrip(uint8_t _port);
        
        // initialize ledStrip Driver and set the led quantity. (value: 1-60)
        void begin(int ledCount);
        
        // Automatic cycle refresh each LED. After perform this function, the led refresh automatically.
        // You can set the reflash time. Max value = 255.
        void autoFlash(int flashSpeed = 0);
        
        // Refresh once LED, Entirely by the loop function to refresh the LED Strip. When you need to write multiple leds,
        // use the LS_ONCE_FLASH mode to refresh all LED when writing the last LED, in front of the led use LS_NO_FLASH mode,
        // it can speed up the refresh.
        void onceFlash();
        
        // Stop all led of strip. But it doesn't close the leds. All LED to keep the last state.
        void stopFlash();
        
        // Stop and reset all led of strip.
        void reset();
        
        boolean readState();
        
        // Write each LED color and refresh mode.
        // LS_AUTO_FLASH : Automatic cycle refresh each LED. After perform this function, the led refresh automatically.
        //                                                                 This mode is not commonly used.
        // LS_ONCE_FLASH : Please refer to the instructions of onceFlash().
        // ___________________________________________________________________________________________
        // Parameter description: | LED Number | Red brightness | Green brightness | Blue brightness | Indicators flash speed |
        // |___________________________________________________________________________________________|
        void setPixelColor(byte lsNum = LS_ALL_PIXEL, byte lsR = MIN_BRI,byte lsG = MIN_BRI, byte lsB = MIN_BRI, byte lsMode = LS_ONCE_FLASH);
        
		//long getPixelColor(byte n);

        // Color gradient LED Scroller function.
        // Automatically refresh after initialization, you can use stopFlash() to stop flash, but it doesn't close leds.
        // use the reset() to stop and reset led.
        void color_loop();
        
        // This is an indicator function that is used to display range quickly.
        // ___________________________________________________________________________________________
        // Parameter description: | LED Number | Red brightness | Green brightness | Blue brightness | Indicators flash speed |
        // |___________________________________________________________________________________________|
        void indicators(byte lsNum, byte lsR, byte lsG, byte lsB, byte lsSpd = IN_SPEED);
};

class MeStepperMotor:public MeWire
{
	public:
        //portNum can ONLY be PORT_1 or PORT_2
        MeStepperMotor(uint8_t _port);
        
        // initialize stepper driver.
        void begin(byte microStep = STP_SIXTEENTH,long speed = 10000,long acceleration = 5000);
        
        // set micro step. Mode: STP_FULL, STP_HALF, STP_QUARTER, STP_EIGHTH, STP_SIXTEENTH
        void setMicroStep(byte microStep);

        // stop stepper and reset current position to zero.
        void reset();
    
		/// Set the target position. The run() function will try to move the motor
		/// from the current position to the target position set by the most
		/// recent call to this function. Caution: moveTo() also recalculates the speed for the next step.
		/// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
		/// \param[in] absolute The desired absolute position. Negative is anticlockwise from the 0 position.
		void moveTo(long stepperMoveTo);

		/// Set the target position relative to the current position
		/// \param[in] relative The desired position relative to the current position.
		/// Negative is anticlockwise from the current position.
		void move(long stepperMove);

		/// Poll the motor and step it if a step is due, implmenting a constant
		/// speed as set by the most recent call to setSpeed(). You must call this as
		/// frequently as possible, but at least once per step interval,
		/// \return true if the motor was stepped.
		void runSpeed();

		/// Sets the maximum permitted speed. the run() function will accelerate
		/// up to the speed set by this function.
		/// \param[in] speed The desired maximum speed in steps per second. Must
		/// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
		/// Result in non-linear accelerations and decelerations.
		void setMaxSpeed(long stepperMaxSpeed);

		/// Sets the acceleration and deceleration parameter.
		/// \param[in] acceleration The desired acceleration in steps per second
		/// per second. Must be > 0.0. This is an expensive call since it requires a square
		/// root to be calculated. Dont call more ofthen than needed
		void setAcceleration(long stepperAcceleration);

		/// Sets the desired constant speed for use with runSpeed().
		/// \param[in] speed The desired constant speed in steps per
		/// second. Positive is clockwise. Speeds of more than 1000 steps per
		/// second are unreliable. Very slow speeds may be set (eg 0.00027777 for
		/// once per hour, approximately. Speed accuracy depends on the Arduino
		/// crystal. Jitter depends on how frequently you call the runSpeed() function.
		void setSpeed(long stepperSpeed);

		/// The most recently set speed
		/// \return the most recent speed in steps per second
		long speed();

		/// The distance from the current position to the target position.
		/// \return the distance from the current position to the target position in steps.
		/// Positive is clockwise from the current position.
		long distanceToGo();

		/// The most recently set target position.
		/// \return the target position in steps. Positive is clockwise from the 0 position.
		long targetPosition();

		/// The currently motor position.
		/// \return the current motor position
		/// in steps. Positive is clockwise from the 0 position.
		long currentPosition();

		/// Resets the current position of the motor, so that wherever the motor
		/// happens to be right now is considered to be the new 0 position. Useful
		/// for setting a zero position on a stepper after an initial hardware
		/// positioning move.
		/// Has the side effect of setting the current motor speed to 0.
		/// \param[in] position The position in steps of wherever the motor happens to be right now.
		void setCurrentPosition(long stepperCuttentPos);

        // enable stepper driver, Keep the micro step current position.
        void enable();
        
        // disable stepper driver, release the stepper.
        void disable();

        // output pulse
        void run();

        /// Sets a new target position that causes the stepper
		/// to stop as quickly as possible, using to the current speed and acceleration parameters.
        void stop();

        // stop all dispose, keep the user setting data.
        void wait();

        boolean readState();
        
	private:
        long stepperSpeedRead;
        long stepperDistanceToGoRead;
        long stepperTargetPositionRead;
        long stepperCurrentPositionRead;
};
#endif