//#include <Arduino.h>
#include "helper.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "Serialcontact.h"
#define MAX_PWM 100

Motor motorl;
Motor motorr;

Encoder encl;
Encoder encr;
float pprR = 2488.2; //average pulse from encoder or ppr
float pprL = 2470.2; //average pulse from encoder or ppr
float wheel_circumference = 68.5; //cm

float kp = 2.0;
float ki = 0.01;
float kd = 0.01;

PID leftPID(kp, ki, kd, MAX_PWM);
PID rightPID(kp, ki, kd, MAX_PWM);

volatile int counterLeft = 0;  // This variable will increase or decrease depending on the rotation of encoder
volatile int counterRight = 0; // This variable will increase or decrease depending on the rotation of encoder

Serialcontact SC;
int joy_mode = +999;
int auto_mode = -999;


int pwmL = 0;
int pwmR = 0;

void enc_left()
{
  encl.update();
}

void enc_right()
{
  encr.update();
}

velocity v;
velocity v_target;

pid_param k;
pwm p;

int mode = 0; // 0: PWM, 1: PID

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  while (!Serial)
    ;
  // waitStartCommand("mobile");
  digitalWrite(13, 1);
  motorl.init(5, 7, 4, 150);
  motorr.init(6, 9, 8, 150);

  encl.init(2, A1, false, pprL, wheel_circumference);
  encr.init(3, A0, true, pprR, wheel_circumference);

  attachInterrupt(digitalPinToInterrupt(encl.ena), enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(encr.ena), enc_right, RISING);
}

const int interval = 50;
long previousMillis = 0;
long currentMillis = 0;
//velocity set point
float setPoint = -5.0;//-20.0

void loop()
{
  for (int i = 0 ; i < 1; i++)
  {
    Serial.print(SC.update());
  }
    motorl.rotate(100);
  
//  currentMillis = millis();
//  previousMillis = currentMillis;
//  while (1)
//  {
//    currentMillis = millis();
//    
//    if (currentMillis - previousMillis > interval)
//    {
//      if (encl.cal_cm() <= getposition || encr.cal_cm() <= getposition )
//      {
//        float deltaT = currentMillis - previousMillis;
//        previousMillis = currentMillis;
//        motorl.cal_velocity(encl.position);
//        motorr.cal_velocity(encr.position);
//        float v_l = motorl.v * 100.0;
//        float v_r = motorr.v * 100.0;
//        pwmL = leftPID.evalu(v_l, setPoint, deltaT);
//        pwmR = rightPID.evalu(v_r, setPoint, deltaT);
//        if (encl.cal_cm() <= getposition)
//        {
//          motorl.rotate(pwmL);
//        }
//        else
//        {
//          motorl.rotate(0);
//        }
//        if (encr.cal_cm() <= getposition)
//        {
//          motorr.rotate(pwmR * (-1));
//        }
//        else
//        {
//          motorr.rotate(0);
//        }
//
////        Serial.print(v_l);
////        Serial.print("\t");
////        Serial.print(v_r);
////        Serial.print("\t");
////        Serial.println(setPoint);
//      }
//    }
//    if (encl.cal_cm() >= getposition && encr.cal_cm() >= getposition )
//    {
//      encl.cm = 0;
//      encr.cm = 0;
//      encl.position = 0;
//      encr.position = 0;
//      pwmL = 0;
//      pwmR = 0;
//      motorl.rotate(0);
//      motorr.rotate(0);
//      Serial.println("StandBy");
//      break;
//    }
//  }
}