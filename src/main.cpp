#include <Arduino.h>
#include <SPI.h>
#include "servoshock_PS4.h"

const int slaveSelect = 10;
Servoshock Servoshock1(slaveSelect);  //create instance of Servoshock

void setup() {
  // put your setup code here, to run once:
  digitalWrite(slaveSelect, HIGH);
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV16);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
  Serial.begin(115200);  //initialize serial monitor

}

void loop() {
  // put your main code here, to run repeatedly:
  // dude
  //
  // Servoshock1.Update();  // update every 20 ms or 100Hz
  //
  //Servoshock1.inPacket.lStickX;	
	//Servoshock1.inPacket.lStickY;	
	//Servoshock1.inPacket.rStickX;	
	//Servoshock1.inPacket.rStickY;
	
}