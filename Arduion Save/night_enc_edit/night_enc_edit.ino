//#include <Arduino.h>
#include "helper.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

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

const unsigned int WordLength = 9;
int getposition;
int pwmL = 0;
int pwmR = 0;
int getPWML;
int getPWMR;

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
  while (1)
  {

    boolean EndCommand = 0;
    while (Serial.available() > 0)
    {
      static char Command[WordLength];
      static unsigned int WordPosition = 0;
      char InByte = Serial.read();
      if (InByte != '\n')
      {
        Command[WordPosition] = InByte;
        WordPosition++;
      }
      else
      {
        Command[WordPosition] = '\0';
        //Serial.print("input = ");
        //Serial.println(Command);
        int Data0 = (Command[0]);
        if (Data0 == 106) {
          EndCommand = 0;
          getposition = 0;
          int Data1 = (Command[1]);
          int Data2 = (Command[2] - '0') * 100;
          int Data3 = (Command[3] - '0') * 10;
          int Data4 = (Command[4] - '0');
          if (Data1 == 43)
          {
            getPWML = Data2 + Data3 + Data4;
          }
          else if (Data1 == 45)
          {
            getPWML = -(Data2 + Data3 + Data4);
          }
          else
          {
            getPWML = 0;
          }
          int Data5 = (Command[5]);
          int Data6 = (Command[6] - '0') * 100;
          int Data7 = (Command[7] - '0') * 10;
          int Data8 = (Command[8] - '0');
          if (Data5 == 43)
          {
            getPWMR = -(Data2 + Data3 + Data4);
          }
          else if (Data5 == 45)
          {
            getPWMR = (Data2 + Data3 + Data4);
          }
          else
          {
            getPWMR = 0;
          }

          motorl.rotate(getPWMR);
          motorr.rotate(getPWML);

        }
        else if (Data0 == 97) {
          EndCommand = 1;
          getPWML = 0;
          getPWMR = 0;
          int Data1 = (Command[1]);
          int Data2 = (Command[2] - '0') * 100;
          int Data3 = (Command[3] - '0') * 10;
          int Data4 = (Command[4] - '0');
          if (Data1 == 43)
          {
            getposition = Data2 + Data3 + Data4;
          }
          else if (Data1 != 43) {
            getposition = 0;
          }
        }
        break;
      }
    }
    if (EndCommand == 1)
    {
      break;
    }
  }

  encl.cm = 0;
  encr.cm = 0;
  encl.position = 0;
  encr.position = 0;
  pwmL = 0;
  pwmR = 0;
  motorl.rotate(0);
  motorr.rotate(0);
  currentMillis = millis();
  previousMillis = currentMillis;
  while (1)
  {
    currentMillis = millis();

    if (currentMillis - previousMillis > interval)
    {
      if (encl.cal_cm() <= getposition || encr.cal_cm() <= getposition )
      {
        float deltaT = currentMillis - previousMillis;
        previousMillis = currentMillis;
        motorl.cal_velocity(encl.position);
        motorr.cal_velocity(encr.position);
        float v_l = motorl.v * 100.0;
        float v_r = motorr.v * 100.0;
        pwmL = leftPID.evalu(v_l, setPoint, deltaT);
        pwmR = rightPID.evalu(v_r, setPoint, deltaT);
        if (encl.cal_cm() <= getposition)
        {
          motorl.rotate(pwmL);
          //          Serial.println(encl.cal_cm());
        }
        else
        {
          motorl.rotate(0);
        }
        if (encr.cal_cm() <= getposition)
        {
          motorr.rotate(pwmR * (-1));
          //          Serial.println(encr.cal_cm());
        }
        else
        {
          motorr.rotate(0);
        }

        //        Serial.print(v_l);
        //        Serial.print("\t");
        //        Serial.print(v_r);
        //        Serial.print("\t");
        //        Serial.println(setPoint);
      }
    }
    if (encl.cal_cm() >= getposition && encr.cal_cm() >= getposition )
    {
      encl.cm = 0;
      encr.cm = 0;
      encl.position = 0;
      encr.position = 0;
      pwmL = 0;
      pwmR = 0;
      motorl.rotate(0);
      motorr.rotate(0);
      Serial.println("StandBy");
      break;
    }
  }
}
