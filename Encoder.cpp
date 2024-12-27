#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Arduino.h"
#define R1_PWM          8
#define R1_Direction    A0
#define L1_PWM          9
#define L1_Direction    A1

#define R2_PWM          10
#define R2_Direction    A2
#define L2_PWM          11
#define L2_Direction    A3

#define R3_PWM          12
#define R3_Direction    A4
#define L3_PWM          13
#define L3_Direction    A5

const int encoderPinAR1 = 2;  
const int encoderPinBR1 = 3;
const int encoderPinAL1 = 4;  
const int encoderPinBL1 = 5;
const int encoderPinAR2 = 6;  
const int encoderPinBR2 = 7;
const int encoderPinAL2 = 8;  
const int encoderPinBL2 = 9;
const int encoderPinAR3 = 10;  
const int encoderPinBR3 = 11;
const int encoderPinAL3 = 12;  
const int encoderPinBL3 = 13;

const int PPR = 7;         
const float wheelRadius = 10.0;  
const unsigned long measurementPeriod = 1000;  

volatile signed int encoderCount = 0;      
volatile signed int lastEncoderCount = 0;   
unsigned long startTime = 0;         

void setup() 
{
  Serial.begin(9600);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(L1_PWM,OUTPUT);
  pinMode(L1_Direction,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
}

void loop() 
{  
  
 
  startTime = millis();
  encoderCount = 0;
  lastEncoderCount = 0;
  delay(measurementPeriod);

  unsigned long elapsedTime = millis() - startTime;

  float rpm = (float(encoderCount) / PPR) / (elapsedTime / 60000.0);
  float speed = (2 * PI * wheelRadius * rpm) / 60.0;
  float distance = (2 * PI * wheelRadius * encoderCount) / PPR;

  Serial.print("RPM: ");
  Serial.println(rpm);
  Serial.print("Speed: ");
  Serial.print(speed);
  Serial.println(" cm/s");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.println("\n");
}

void handleEncoder() 
{
 
  int encoderStateB = digitalRead(encoderPinB);
  
  if (lastEncoderCount == 1 && encoderStateB == HIGH) 
  {
    encoderCount++;
    //Serial.println("Foward");
  } 
  else if (lastEncoderCount == 0 && encoderStateB == LOW) 
  {
    encoderCount--;
    //Serial.println("Backward");

  }

  lastEncoderCount = encoderStateB;
}
