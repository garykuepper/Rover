#include <Arduino.h>
#include <SPI.h>
#include "servoshock_PS4.h"
#include <SimpleTimer.h>

const int slaveSelect = 10;
const int leftMotorPin = 9;
const int leftMotorA = 8;
const int leftMotorB = 7;
const int rightMotorPin = 3;
const int rightMotorA = 4;
const int rightMotorB = 2;
int leftSpeed = 0;
int rightSpeed = 0;
Servoshock Servoshock1(slaveSelect);  //create instance of Servoshock
SimpleTimer timer;

void servoshockUpdate();
void motor(int stickIn, int &out);

void setup() {
  // put your setup code here, to run once:
  digitalWrite(slaveSelect, HIGH);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  motor(127, leftMotorPin, leftMotorA, leftMotorB);
  motor(127, rightMotorPin, rightMotorA, rightMotorB);
  
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV16);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
  Serial.begin(115200);  //initialize serial monitor
  timer.setInterval(20, servoshockUpdate);
}

void loop() {
  //
  timer.run();
	
}

void servoshockUpdate() {
  Servoshock1.Update();  // update every 20 ms or 100Hz
  motor(Servoshock1.inPacket.lStickY, leftMotorPin, leftMotorA, leftMotorB);
  motor(Servoshock1.inPacket.rStickY, rightMotorPin, rightMotorA, rightMotorB);
  //Servoshock1.inPacket.lStickX;	
	//Servoshock1.inPacket.lStickY;	
	//Servoshock1.inPacket.rStickX;	
	//Servoshock1.inPacket.rStickY;
}

void motor(int stickIn, int speedPin, int A, int B) {
  int deadBand = 5;
  // fwd
  if (stickIn > 127 + deadBand) {
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    analogWrite(speedPin, map(stickIn, 127, 255, 0, 255));
  }
  //reverse
  else if (stickIn < 127 - deadBand) {
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    analogWrite(speedPin, map(stickIn, 127, 0, 0, 255));
  }
  //stop
  else {
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    analogWrite(speedPin, 0);
  }
  

}
