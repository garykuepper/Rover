#include <Arduino.h>
#include <SPI.h>
#include "servoshock_PS4.h"
#include <SimpleTimer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
const int servoshockSelect = 10;
const int sdcardSelect = A7;
const int leftMotorPin = 9;
const int leftMotorA = 8;
const int leftMotorB = 7;
const int rightMotorPin = 3;
const int rightMotorA = 4;
const int rightMotorB = 2;
const int lightPin = 6;
int leftSpeed = 0;
int rightSpeed = 0;
int lightStatus = 0;
int prevButton = 0;
float batteryVoltage = 0.0;
unsigned long timeLight = 0;

Servoshock Servoshock1(servoshockSelect);  //create instance of Servoshock
SimpleTimer timer;
SimpleTimer timerDisplay;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

void servoshockUpdate();
void motor(int stickIn, const int speedPin, const int A, const int B) ;
void lightSwitch(int buttonIn);
void refreshDisplay();
void setup() {
  // put your setup code here, to run once:
  digitalWrite(servoshockSelect, HIGH);
  digitalWrite(sdcardSelect, HIGH);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(lightPin, OUTPUT);
  analogWrite(lightPin, 0);
  motor(127, leftMotorPin, leftMotorA, leftMotorB);
  motor(127, rightMotorPin, rightMotorA, rightMotorB);

	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV16);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
  Serial.begin(115200);  //initialize serial monitor
  timer.setInterval(20, servoshockUpdate);
  timer.setInterval(100, refreshDisplay);
  
  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.display();
  delay(500); // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
}

void loop() {
  //
  timer.run();
  timerDisplay.run();	
}

void refreshDisplay() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int batteryValue = analogRead(A3);  
  batteryVoltage = batteryValue * (5.0 / 1023.0) * (47.2+9.87)/10.0;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Bat: ");
  display.print(batteryVoltage);
  display.println("V");
  //display.println("Accelerometer - m/s^2");
  display.print(a.acceleration.x, 1);
  display.print(", ");
  display.print(a.acceleration.y, 1);
  display.print(", ");
  display.print(a.acceleration.z, 1);
  display.println("");
  display.println("Gyroscope - rps");
  display.print(g.gyro.x, 1);
  display.print(", ");
  display.print(g.gyro.y, 1);
  display.print(", ");
  display.print(g.gyro.z, 1);
  display.println("");
  display.display();
}

void servoshockUpdate() {
  Servoshock1.Update();  // update every 20 ms or 100Hz
  motor(Servoshock1.inPacket.lStickY, leftMotorPin, leftMotorA, leftMotorB);
  motor(Servoshock1.inPacket.rStickY, rightMotorPin, rightMotorA, rightMotorB);
  lightSwitch(Servoshock1.inPacket.square);

}

void motor(int stickIn, const int speedPin, const int A, const int B)  {
  int deadBand = 20;
  // fwd  
  //Serial.println(stickIn);
  if (stickIn > 127 + deadBand) {
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
    analogWrite(speedPin, map(stickIn, 130 , 255, 0, 255));
  }
  //reverse
  else if (stickIn < 127 - deadBand) {
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
    analogWrite(speedPin, map(stickIn, 127 , 0, 0, 255));
  }
  //stop
  else {
    digitalWrite(A, LOW);
    digitalWrite(B, LOW);
    analogWrite(speedPin, 0);
  }
}
void lightSwitch(int buttonIn) {   
  unsigned long debounce = 1000; 
  //Serial.println(buttonIn);
  if(millis() - timeLight > debounce) {
    if(buttonIn != prevButton) {      
      if (lightStatus) {
        analogWrite(lightPin, 0);
        lightStatus = false;         
      }
      else {
        analogWrite(lightPin, 50);
        lightStatus = true;         
      }         
      timeLight = millis();             
    }
  }
  prevButton = buttonIn;
}