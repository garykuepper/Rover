#ifndef SERVOSHOCK_H_
#define SERVOSHOCK_H_

#define INPUT_PACKET_SIZE 96
#define OUTPUT_PACKET_SIZE 39

/*
 * Rev 1.0: Initial Release
 * Rev 2.0: Fixed array packing.
The library enables the Arduino to interact with the Servoshock 2 board for the PS4 controller.
  It can read the controller state, read the output state of the Servoshock board, write the
  Servoshock board outputs, and control the rumble motors and lightbar on the controller.


The basic structure of the program is:

	#include <SPI.h>
	#include "servoshock_PS4.h"

	// set the slave select pin for the Servoshock.  Set jumper JP3 on the Shield to D10 if using digital output 10.
	const int16_t slaveSelect = 10;
	Servoshock Servoshock1(slaveSelect);  //create instance of Servoshock

	void setup()
	{
		//initialize SPI:
		digitalWrite(slaveSelect, HIGH);
		SPI.setDataMode(SPI_MODE0);
		SPI.setClockDivider(SPI_CLOCK_DIV16);
		SPI.setBitOrder(MSBFIRST);
		SPI.begin();
	}

	void loop()
	{
		Servoshock1.Update(); //this updates the input and output SPI packets.

		//Rest of the program, for example:
		if (Servoshock1.inPacket.dPadUp)
		{
			...
		}

		//if you want to control a servo:
		Servoshock1.outPacket.overrideLStickX = 1;
		Servoshock1.outPacket.lStickX_uS = servo_uS; //output this pulse width in microseconds.

		delay(10); //don't update faster than 100Hz.
	}
*/

/************************************************************************************************
 struct TOUCHPAD_RAW_DATA
 *************************************************************************************************
 raw touchpad data from Servoshock
 ************************************************************************************************/
typedef struct __attribute__((packed)) _FINGER {
	uint8_t touchID :7; // Increments every time a finger is touching the touchpad
	uint8_t noFinger :1; // The top bit is cleared if the finger is touching the touchpad
	uint16_t x :12;  //x position
	uint16_t y :12;  //y position
} FINGER_DATA;

typedef struct __attribute__((packed)) _TOUCHPAD_RAW_DATA {
	uint8_t packetCounter;
	FINGER_DATA finger[2]; // 0 = first finger, 1 = second finger
} TOUCHPAD_RAW_DATA;

/************************************************************************************************
 struct SPI_INPUT_PACKET
 *************************************************************************************************
 structure to hold SPI input packet (Servoshock to Arduino)
 ************************************************************************************************/

typedef union __attribute__((packed)) _INPUT_FIELDS //WARNING: if you edit this, you need to change the PS4_REPORT_LENGTH!!!
{
	struct __attribute__((packed)){
		uint8_t :8; //old buffer data, discard.  //byte 0
		uint8_t :8; //old buffer data, discard.
		uint8_t :8; //old buffer data, discard.
		uint8_t :8; //old buffer data, discard.
		uint8_t :8; //old buffer data, discard.
		uint8_t :8; //old buffer data, discard.
		uint8_t :8; //old buffer data, discard.
		uint8_t :8; //old buffer data, discard.

		//start of packet in both USB and BT modes
		uint8_t reportID; //byte 8

		//Stick analog values
		uint8_t lStickX; //left stick X axis (0-255)
		uint8_t lStickY; //left stick Y axis (0-255)
		uint8_t rStickX; //right stick X axis (0-255)
		uint8_t rStickY; //right stick Y axis (0-255) 12

		//Button states
		uint8_t dPadUp :1; //13
		uint8_t dPadRight :1;
		uint8_t dPadDown :1;
		uint8_t dPadLeft :1; 
		uint8_t square :1;
		uint8_t cross :1;
		uint8_t circle :1;
		uint8_t triangle :1;

		uint8_t lBumper :1; //14
		uint8_t rBumper :1;
		uint8_t lTriggerDigital :1;
		uint8_t rTriggerDigital :1;
		uint8_t share :1;
		uint8_t options :1;
		uint8_t lStickPress :1;
		uint8_t rStickPress :1;

		uint8_t psButton :1; //byte 15
		uint8_t tpadPress :1;
		uint8_t counter :6;

		uint8_t lTriggerAnalog; //(0-255)16
		uint8_t rTriggerAnalog; //(0-255)17
		uint8_t bytes10_12; //18/19/20
    uint8_t bytes11; //18/19/20
    uint8_t bytes12; //18/19/20

		//IMU readings
		int16_t gyroX; //21-22
		int16_t gyroY; //23-24
		int16_t gyroZ; //25-26
		int16_t accelX; //27-28
		int16_t accelY; //29-30
		int16_t accelZ; //31-32

		uint8_t bytes25_29[5]; //33-37
		uint8_t battery :4; //38
		uint8_t USBData :1; //usb plugged in
		uint8_t :3; //
    
		uint8_t bytes31_32[2]; //39-40

		//TOUCHPAD readings
		uint8_t numTpadPackets; //byte 41
		TOUCHPAD_RAW_DATA tpad[3]; //bytes 42-68

		//These members will tell pulse width of the servo signal
		uint16_t lStickXState_uS; //byte 0-1 of spiOutputStatePacket in Servoshock firmware
		uint16_t lStickYState_uS; //2-3
		uint16_t rStickXState_uS; //4-5
		uint16_t rStickYState_uS; //6-7
		uint16_t lTriggerAnalogState_uS; //8-9
		uint16_t rTriggerAnalogState_uS; //10-11
		uint16_t lTpadXState_uS; //12-13
		uint16_t lTpadYState_uS; //14-15
		uint16_t rTpadXState_uS; //16-17
		uint16_t rTpadYState_uS; //18-19
		uint16_t tiltXState_uS; //20-21
		uint16_t tiltYState_uS; //22-23

		//these members tell the state of the I/O output
		uint8_t dPadUpState :1; //byte 24
		uint8_t dPadRightState :1;
		uint8_t dPadDownState :1;
		uint8_t dPadLeftState :1;
		uint8_t triangleState :1;
		uint8_t circleState :1;
		uint8_t crossState :1;
		uint8_t squareState :1;

		uint8_t lBumperState :1; //byte 25
		uint8_t rBumperState :1;
		uint8_t lTriggerDigitalState :1;
		uint8_t rTriggerDigitalState :1;
		uint8_t lStickPressState :1;
		uint8_t rStickPressState :1;
		uint8_t shareState :1;
		uint8_t optionsState :1;

		uint8_t tpadPressState :1; //byte 26
		uint8_t psButtonState :1;
		uint8_t :6;

	};
	uint8_t array[INPUT_PACKET_SIZE];
} SPI_INPUT_PACKET; //96 bytes total

/************************************************************************************************
 struct SPI_OUTPUT_PACKET
 *************************************************************************************************
 structure to hold SPI output packet (Arduino to Servoshock)
 Setting the overrideXXXX bits will enable you to set the outputs with the Arduino instead of the controller.
 For example, to override the left stick X axis, use:
	Servoshock1.outPacket.overrideLStickX = 1; //enable override
	Servoshock1.outPacket.lStickX_uS = 1500; //set output pulse width to 1500 microseconds

To give control back to the Dualshock, set the override bit back to 0.

 ************************************************************************************************/
typedef struct __attribute__((packed)) _OUTPUT_FIELDS {
	union __attribute__((packed)) {
		struct __attribute__((packed)) {
			//override feedback
			uint8_t overrideLED :1; //byte 0 bit 0
			uint8_t overrideRumbleL :1;
			uint8_t overrideRumbleH :1;
			uint8_t :5; //bit 7

			uint8_t LEDRed;  //byte 1
			uint8_t LEDGreen;
			uint8_t LEDBlue;
			uint8_t LEDBlinkOnDuration;
			uint8_t LEDBlinkOffDuration;
			uint8_t rumbleL;
			uint8_t rumbleH; //byte 7

			uint8_t overrideLStickX :1; //byte 8
			uint8_t overrideLStickY :1;
			uint8_t overrideRStickX :1;
			uint8_t overrideRStickY :1;
			uint8_t overrideLTriggerAnalog :1;
			uint8_t overrideRTriggerAnalog :1;
			uint8_t overrideLTpadX :1;
			uint8_t overrideLTpadY :1;

			uint8_t overrideRTpadX :1; //byte 9
			uint8_t overrideRTpadY :1;
			uint8_t overrideTiltX :1;
			uint8_t overrideTiltY :1;
			uint8_t overrideDPadUp :1;
			uint8_t overrideDPadRight :1;
			uint8_t overrideDPadDown :1;
			uint8_t overrideDPadLeft :1;

			uint8_t overrideTriangle :1; //byte 10
			uint8_t overrideCircle :1;
			uint8_t overrideCross :1;
			uint8_t overrideSquare :1;
			uint8_t overrideLBumper :1;
			uint8_t overrideRBumper :1;
			uint8_t overrideLTriggerDigital :1;
			uint8_t overrideRTriggerDigital :1;

			uint8_t overrideLStickPress :1; //byte 11
			uint8_t overrideRStickPress :1;
			uint8_t overrideShare :1;
			uint8_t overrideOptions :1;
			uint8_t overrideTpadPress :1;
			uint8_t overridePsButton :1;
			uint8_t :2;

			uint16_t lStickX_uS; //byte 12-13
			uint16_t lStickY_uS;
			uint16_t rStickX_uS;
			uint16_t rStickY_uS;
			uint16_t lTriggerAnalog_uS; //byte 20-21
			uint16_t rTriggerAnalog_uS;
			uint16_t lTpadX_uS;
			uint16_t lTpadY_uS;
			uint16_t rTpadX_uS;
			uint16_t rTpadY_uS; //byte 30-31
			uint16_t tiltX_uS;
			uint16_t tiltY_uS; //byte 34-35

			uint8_t dPadUp :1; //byte 36
			uint8_t dPadRight :1;
			uint8_t dPadDown :1;
			uint8_t dPadLeft :1;
			uint8_t triangle :1;
			uint8_t circle :1;
			uint8_t cross :1;
			uint8_t square :1;

			uint8_t lBumper :1; //byte 37
			uint8_t rBumper :1;
			uint8_t lTriggerDigital :1;
			uint8_t rTriggerDigital :1;
			uint8_t lStickPress :1;
			uint8_t rStickPress :1;
			uint8_t share :1;
			uint8_t options :1;

			uint8_t tpadPress :1; //byte 38
			uint8_t psButton :1;
			uint8_t :6;
		};
		uint8_t array[OUTPUT_PACKET_SIZE];
	};
} SPI_OUTPUT_PACKET;



/************************************************************************************************
 class Servoshock
 *************************************************************************************************
 Create an instance of Servoshock.

 Input: Slave Select digital I/O pin number

 Usage Example:
 Servoshock servoshock1(10);
 ************************************************************************************************/
class Servoshock {
public:
	Servoshock(int16_t slaveSelectPin);
	void Update(void);
	void SetLED(uint8_t red, uint8_t green, uint8_t blue,
	uint8_t blinkOnDuration, uint8_t blinkOffDuration);

	SPI_OUTPUT_PACKET outPacket = { };
	SPI_INPUT_PACKET inPacket = { };

private:
	int16_t SSPin;
};

Servoshock::Servoshock(int16_t slaveSelectPin) {
	pinMode(slaveSelectPin, OUTPUT);
	SSPin = slaveSelectPin;
}
;

/************************************************************************************************
 function Servoshock::update
 *************************************************************************************************
 Performs the SPI packet transfer to read from and write to the Servoshock module.  Also converts
 the 16-bit parameters (accels and gyro) between little endian (Arduino) to big endian (DualShock)

 Usage Example:
 Servoshock1.Update();
 ************************************************************************************************/
void Servoshock::Update(void) {
	uint8_t dPadDirection;
	digitalWrite(SSPin, LOW);
	for (uint16_t i = 0; i < INPUT_PACKET_SIZE; i++) {
		if (i < OUTPUT_PACKET_SIZE) {
			inPacket.array[i] = SPI.transfer(outPacket.array[i]);
		} else {
			inPacket.array[i] = SPI.transfer(0xCD); //padding bytes
		}
	}
	//dPad bits are actually a 4-bit number indicating direction, but we want button state.
	dPadDirection = inPacket.dPadUp + inPacket.dPadRight*2 + inPacket.dPadDown*4 + inPacket.dPadLeft*8;
	inPacket.dPadUp =  (dPadDirection == 7 || dPadDirection == 0 || dPadDirection == 1);
	inPacket.dPadRight = (dPadDirection == 1 || dPadDirection == 2 || dPadDirection == 3);
	inPacket.dPadDown = (dPadDirection == 3 || dPadDirection == 4 || dPadDirection == 5);
	inPacket.dPadLeft = (dPadDirection == 5 || dPadDirection == 6 || dPadDirection == 7);

	digitalWrite(SSPin, HIGH);
}

/************************************************************************************************
 function Servoshock::SetLED
 *************************************************************************************************
Sets the LEDs.  Takes RGB values 0-255, and blink on and off durations.  Durations are from 0-255, 255 is about 2.5 seconds.
 Usage Example:
Servoshock1.SetLED(255, 255, 255, 0, 0);
 ************************************************************************************************/
void Servoshock::SetLED(uint8_t red, uint8_t green,
		uint8_t blue, uint8_t blinkOnDuration,
		uint8_t blinkOffDuration) {
	outPacket.LEDRed = red;
	outPacket.LEDGreen = green;
	outPacket.LEDBlue = blue;
	outPacket.LEDBlinkOnDuration = blinkOnDuration;
	outPacket.LEDBlinkOffDuration = blinkOffDuration;
}


#endif
