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

#define Forward_speed      60.0
#define Rotate_speed      150.0


#define echo1 A6
#define trig1 A7

#define echo2 A8
#define trig2 A9

#define echo3 A10
#define trig3 A11

#define echo4 A12
#define trig4 A13

#define echo5 A14
#define trig5 A15

unsigned char ultra_1=0 ;
unsigned char ultra_2=0 ;
unsigned char ultra_3=0 ;
unsigned char ultra_4=0 ;
unsigned char ultra_5=0 ;

unsigned char Max_distance= 40;
unsigned char Min_distance= 20;

//*******************************************************************************
const int encoderPinAL1 = 4; 
const int encoderPinBL1 = 5; 
#define L1_PWM          9
#define L1_Direction    A1

const int PPR = 7;          
const float wheelRadius = 10.0;  
const unsigned long measurementPeriod = 1000;  

volatile signed int encoderCount = 0;    
volatile signed int lastEncoderCount = 0;   
unsigned long startTime = 0;       

//*******************************************************************************


void setup() 
{
  Serial.begin(9600);
  
  encoder_inti();
  encoder_interrupt();
  motor_init();

  ultra_init(trig1,echo1);
  ultra_init(trig2,echo2);
  ultra_init(trig3,echo3);
  ultra_init(trig4,echo4);
  ultra_init(trig5,echo5);
}

void loop() 
{
 
  Rover_movement();

//**************************************************************************
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
//**************************************************************************

}


void ultra_init(unsigned char trig,unsigned char echo)
{
    pinMode(trig,OUTPUT);
    pinMode(echo,INPUT);
}

 
unsigned char ultra_work(unsigned char trig,unsigned char echo)
{
  unsigned int  duration;
  unsigned char distance=0;
  digitalWrite(trig,LOW);
  delayMicroseconds(5);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  duration=pulseIn(echo,HIGH);
  distance=(duration*0.034) / 2;
  return distance; 
}

void motor_init(void)
{
  pinMode(R1_PWM,OUTPUT);
  pinMode(R1_Direction,OUTPUT);
  pinMode(L1_PWM,OUTPUT);
  pinMode(L1_Direction,OUTPUT);



  pinMode(R2_PWM,OUTPUT);
  pinMode(R2_Direction,OUTPUT);
  pinMode(L2_PWM,OUTPUT);
  pinMode(L2_Direction,OUTPUT);



  pinMode(R3_PWM,OUTPUT);
  pinMode(R3_Direction,OUTPUT);
  pinMode(L3_PWM,OUTPUT);
  pinMode(L3_Direction,OUTPUT); 
}


void Forward_motor (void)
{
  analogWrite(R1_PWM,Forward_speed);
  digitalWrite(R1_Direction,LOW);
  analogWrite(L1_PWM,Forward_speed);
  digitalWrite(L1_Direction,HIGH);



  analogWrite(R2_PWM,Forward_speed);
  digitalWrite(R2_Direction,LOW);
  analogWrite(L2_PWM,Forward_speed);
  digitalWrite(L2_Direction,HIGH);



  analogWrite(R3_PWM,Forward_speed);
  digitalWrite(R3_Direction,LOW);
  analogWrite(L3_PWM,Forward_speed);
  digitalWrite(L3_Direction,HIGH);
}


void Backward_motor (void)
{
  analogWrite(R1_PWM,Forward_speed);
  digitalWrite(R1_Direction,HIGH);
  analogWrite(L1_PWM,Forward_speed);
  digitalWrite(L1_Direction,LOW);



  analogWrite(R2_PWM,Forward_speed);
  digitalWrite(R2_Direction,HIGH);
  analogWrite(L2_PWM,Forward_speed);
  digitalWrite(L2_Direction,LOW);



  analogWrite(R3_PWM,Forward_speed);
  digitalWrite(R3_Direction,HIGH);
  analogWrite(L3_PWM,Forward_speed);
  digitalWrite(L3_Direction,LOW);    
}

void Stop_motor (void)
{     
  analogWrite(R1_PWM,LOW);
  analogWrite(L1_PWM,LOW); 
  analogWrite(R2_PWM,LOW);
  analogWrite(L2_PWM,LOW);
  analogWrite(R3_PWM,LOW);
  analogWrite(L3_PWM,LOW);
  
}

void Move_Right (void)
{

  analogWrite(R1_PWM,Rotate_speed);
  digitalWrite(R1_Direction,HIGH);
  analogWrite(L1_PWM,Rotate_speed);
  digitalWrite(L1_Direction,HIGH);



  analogWrite(R2_PWM,Rotate_speed);
  digitalWrite(R2_Direction,HIGH);
  analogWrite(L2_PWM,Rotate_speed);
  digitalWrite(L2_Direction,HIGH);



  analogWrite(R3_PWM,Rotate_speed);
  digitalWrite(R3_Direction,HIGH);
  analogWrite(L3_PWM,Rotate_speed);
  digitalWrite(L3_Direction,HIGH);
  
}

void Move_Left (void)
{

  analogWrite(R1_PWM,Rotate_speed);
  digitalWrite(R1_Direction,LOW);
  analogWrite(L1_PWM,Rotate_speed);
  digitalWrite(L1_Direction,LOW);



  analogWrite(R2_PWM,Rotate_speed);
  digitalWrite(R2_Direction,LOW);
  analogWrite(L2_PWM,Rotate_speed);
  digitalWrite(L2_Direction,LOW);



  analogWrite(R3_PWM,Rotate_speed);
  digitalWrite(R3_Direction,LOW);
  analogWrite(L3_PWM,Rotate_speed);
  digitalWrite(L3_Direction,LOW);
  
}



void Rover_movement()
{

  ultra_1=ultra_work(trig1,echo1);//Forward Right
  ultra_2=ultra_work(trig2,echo2);//Forward Lift
  ultra_3=ultra_work(trig3,echo3);//Right side
  ultra_4=ultra_work(trig4,echo4);//Left side
  ultra_5=ultra_work(trig5,echo5);//Back
        
        Serial.print("ultra_1=");
        Serial.print(ultra_1);
        Serial.print("\n");
        Serial.print("ultra_2=");
        Serial.print(ultra_2);
        Serial.print("\n");
        Serial.print("ultra_3=");
        Serial.print(ultra_3);
        Serial.print("\n");
        Serial.print("ultra_4=");
        Serial.print(ultra_4);
        Serial.print("\n");
        Serial.print("ultra_5=");
        Serial.print(ultra_5);
        Serial.print("\n"); 
        
if ( (ultra_1 > Max_distance)  &&  (ultra_2 > Max_distance) )
{
  if ( (ultra_3 <= Min_distance) )
  {
      Serial.print(" Left 1\n");
      Stop_motor();
      delay(20);
      Move_Left();
      delay(500);
      
  }

  else if ( (ultra_4 <= Min_distance) )
  {
      Serial.print(" Right 1\n");
      Stop_motor();
      delay(20);
      Move_Right();
      delay(500);
      
  }
  else if ( (ultra_3 <= Min_distance) && (ultra_4 <= Min_distance) )
  {
      Serial.print(" Backward 1\n");
      Stop_motor();
      delay(20);
      Backward_motor();
      delay(1000);
      

  }
  else
  {
      Serial.print(" Forward 1\n");
      Stop_motor();
      delay(20);
      Forward_motor();
      delay(1000);
      
      
  }

}



else if ( ( ultra_1 > Max_distance ) && (ultra_2 <= Max_distance) )


    {
     Serial.print(" Right 2\n"); 
     Stop_motor();
     delay(20);
     Move_Right();
     delay(500);
     

    }



else if ( (ultra_1 <= Max_distance) && ( ultra_2 > Max_distance ) )
    {
     Serial.print(" Left 2\n"); 
     Stop_motor();
     delay(20);
     Move_Left();
     delay(500);
     

    }


else if ( ( ultra_1 <= Max_distance ) && ( ultra_2 <= Max_distance ) )
{
      
      while ( ( ultra_1 <= Max_distance ) || ( ultra_2 <= Max_distance ) )
      {
        
        ultra_1=ultra_work(trig1,echo1);//Forward Right
        ultra_2=ultra_work(trig2,echo2);//Forward Lift
        ultra_3=ultra_work(trig3,echo3);//Right side
        ultra_4=ultra_work(trig4,echo4);//Left side
        ultra_5=ultra_work(trig5,echo5);//Back
        Serial.print("ultra_1=");
        Serial.print(ultra_1);
        Serial.print("\n");
        Serial.print("ultra_2=");
        Serial.print(ultra_2);
        Serial.print("\n");
        Serial.print("ultra_3=");
        Serial.print(ultra_3);
        Serial.print("\n");
        Serial.print("ultra_4=");
        Serial.print(ultra_4);
        Serial.print("\n"); 
        Serial.print("ultra_5=");
        Serial.print(ultra_5);
        Serial.print("\n"); 
              
            if ( (ultra_3 > Min_distance) )
          {
            Serial.print("while Right 3\n");
             Stop_motor();
             delay(20);
             Move_Right();
             delay(500);        
          }
    
          else if ( (ultra_4 > Min_distance) )
          {
             Serial.print("while Left 3\n");
             Stop_motor();
             delay(20);
             Move_Left();
             delay(500);
          }
          
          else if ( (ultra_3 <= Min_distance) && (ultra_4 <= Min_distance) )
          {
            if ( (ultra_5 > Min_distance) )
            {
             Serial.print("while Backward 3\n");
             Stop_motor();
             delay(20);
             Backward_motor();
             delay(1000);
            }
            else
            {
              Serial.print("while Stop \n");
              Stop_motor();
            }
               
          }
        
      }
      
        
}

}
//*****************************ENCODER**************************************
void encoder_inti(void)
{
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
}

void encoder_interrupt(void)
{
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);

}

void handleEncoder() 
{
  // Read the current state of encoder pins
  int encoderStateB = digitalRead(encoderPinB);

  // Determine the direction based on the change in encoder state
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