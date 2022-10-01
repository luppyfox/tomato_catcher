#include <Arduino.h>
#include "helper.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "SendMove.h"

#define MAX_PWM 100

Motor motorl;
Motor motorr;

Encoder encl;
Encoder encr;

float kp = 2.0;
float ki = 0.01;
float kd = 0.01;

PID leftPID(kp, ki, kd, MAX_PWM);
PID rightPID(kp, ki, kd, MAX_PWM);

volatile int counterLeft = 0;  // This variable will increase or decrease depending on the rotation of encoder
volatile int counterRight = 0; // This variable will increase or decrease depending on the rotation of encoder

SendMove sendmove;

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

int mode = 1; // 0: PWM, 1: PID

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  while (!Serial)
    ;
  // waitStartCommand("mobile");
  digitalWrite(13, 1);
  motorl.init(5, 7, 4, 20);
  motorr.init(6, 9, 8, 20);

  encl.init(2, A1, false);
  encr.init(3, A0, true);

  attachInterrupt(digitalPinToInterrupt(encl.ena), enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(encr.ena), enc_right, RISING);
}

int c = 0;
const int interval = 50;
long previousMillis = 0;
long currentMillis = 0;

float Position = 0;
const unsigned int WordLength = 1;

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
        Serial.print("input = ");
        Serial.println(Command);
        int Data0 = Command[0];
        int getPosition;
        if (Data0 == 43)  //+
        {
          getPosition = 2000;
          sendmove.evalu(1);
        }
        else if (Data0 == 45)   //-
        {
          getPosition = 2000;
          sendmove.evalu(0);
        }
        Position = getPosition / 1000 * 1350; //change mm unit to m unit
        EndCommand = 1;
      }
    }
    if (EndCommand == 1)
    {
      break;
    }

  }
  while (1) {
    currentMillis = millis();
    if (c > Position) //1350 unit = 1m
    {
      motorl.rotate(0);
      motorr.rotate(0);
      Serial.println("StandBy");
      c = 0;
      break;
    }
    else
    {
      if (currentMillis - previousMillis > interval)
      {
        float deltaT = currentMillis - previousMillis;
        previousMillis = currentMillis;
        motorl.cal_velocity(encl.position);
        motorr.cal_velocity(encr.position);
        float v_l = motorl.v * 100;
        float v_r = motorr.v * 110;
        if (sendmove.sendD() == 1)
        {
          float setPointForward = -5.0;
          int pwmL = leftPID.evalu(v_l, setPointForward, deltaT);
          int pwmR = rightPID.evalu(v_r, setPointForward, deltaT);
          motorl.rotate(pwmL);
          motorr.rotate(pwmR + 15);
          c += interval;
          Serial.println("forward");
          Serial.println(pwmL);
          Serial.println(pwmR);

        }
        if (sendmove.sendD() == 0)
        {
          int pwmL = leftPID.evalu(v_l, 5, deltaT);
          int pwmR = rightPID.evalu(v_r, 5, deltaT);
          motorl.rotate(pwmL*(-1));
          motorr.rotate((pwmR + 15)*(-1));
          c += interval;
          Serial.println("backward");
          Serial.println(pwmL);
          Serial.println(pwmR);

        }
      }
    }
  }
}


/*
  {"vl":1.00,"vr":2.00}
*/
