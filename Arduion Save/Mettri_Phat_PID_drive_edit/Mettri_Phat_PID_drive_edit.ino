#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#include "Velocity.h"

Encoder enc_L;
Encoder enc_R;
int encR_a = 2; //enc interrupt pin
int encR_b = A1; //enc analog pin
int encL_a = 3; //enc interrupt pin
int encL_b = A0; //enc analog pin
float max_pprR = 2488.2; //average pulse from encoder or ppr
float max_pprL = 2470.2; //average pulse from encoder or ppr
int get_positionL = 0; //recieve position move from AI with arm
int get_positionR = 0;
int wheel_circumference = 68.5; //cm

Velocity vel_L;
Velocity vel_R;
float v_set, v_L, v_R;
long currentMillis = 0;
long previousMillis = 0;

Motor motor_L;
Motor motor_R;
int mL_dig1 = 5; //motor pin
int mL_dig2 = 7;
int mL_ang = 4;
int mR_dig1 = 6;
int mR_dig2 = 9;
int mR_ang = 8;
int dir_R_1;// wheel rotation direction for motor R
int dir_R_2;
int dir_L_1;// wheel rotation direction for motor L
int dir_L_2;

PID pid_L;
PID pid_R;
float kp = 1.7; //1.8
float ki = 0.001;//0.001
float kd = 8;  //8
float er_L, er_R, er_L_prev, er_R_prev;

const unsigned int WordLength = 4;

void encL_return()
{
  enc_L.pulse_a_change(); //attachInterrupt need to use fuction
}
void encR_return()
{
  enc_R.pulse_a_change(); //attachInterrupt need to use fuction
}

void setup()
{
  Serial.begin(9600);
  enc_L.init(encL_a, encL_b, max_pprL, wheel_circumference);
  enc_R.init(encR_a, encR_b, max_pprR, wheel_circumference);

  motor_L.init(mL_dig1, mL_dig2, mL_ang);
  motor_R.init(mR_dig1, mR_dig2, mR_ang);

  pid_L.init(kp, ki, kd);
  pid_R.init(kp, ki, kd);

  attachInterrupt(digitalPinToInterrupt(enc_L.enc_a), encL_return, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(enc_L.enc_b), enc_L.pulse_b_change(), CHANGE); //Arduino nano can in terrupt 2,3
  attachInterrupt(digitalPinToInterrupt(enc_R.enc_a), encR_return, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(enc_R.enc_b), enc_R.pulse_b_change(), CHANGE); //Arduino nano can in terrupt 2,3
}

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
        int getPosition;
        int Data0 = (Command[0]);
        int Data1 = (Command[1] - '0') * 100;
        int Data2 = (Command[2] - '0') * 10;
        int Data3 = (Command[3] - '0');
        if (Data0 == 43)
        {
          get_positionL = Data1 + Data2 + Data3;
          get_positionR = Data1 + Data2 + Data3;
        }
        else if (Data0 != 43)
        {
          get_positionL = 0;
          get_positionR = 0;
        }
        currentMillis = millis();
        previousMillis = millis() - 1;
        enc_L.pulse = 0; //reset encoder pulse to 0
        enc_R.pulse = 0; //reset encoder pulse to 0
        vel_L.previous_cm = 0;
        vel_R.previous_cm = 0;
        float readL = 0;
        float readR = 0;
        EndCommand = 1;
        break;
      }
    }
    if (EndCommand == 1)
    {
      break;
    }
  }
  while (1)
  {
    currentMillis = millis();
    int deltaT = currentMillis - previousMillis;
    v_set = 5;
    float readL = enc_L.cal_cm() * (-1);
    float readR = enc_R.cal_cm();
    //Serial.print("POS = ");
    //Serial.print(readL);
    //Serial.print(" ");
    //Serial.println(readR);

    if (readL >= get_positionL && readR >= get_positionR)
    {
      float deltaT = 0;
      float previous_cm = 0;
      motor_R.rotate(0, 0, 0);
      motor_L.rotate(0, 0, 0);
      Serial.println("StandBy");
      break;
    }
    // move wheel L and wheel R to the same position(distance)
    //    else if (readL >= get_positionL && readR <= get_positionR)
    //    {
    //      v_R = vel_R.cal_vel(readR, deltaT);
    //      delay(20);
    //      er_R = v_set - v_R;
    //      float debt_R = (er_R - pid_R.error_prev) / deltaT;
    //      float eintegral_R = eintegral_R + (er_R * deltaT);
    //      float uR = (kp * er_R) + (ki * eintegral_R) + (kd * debt_R);
    //      float vR_pid = pid_R.cal_pid(er_R, er_R_prev, deltaT, eintegral_R);
    //      er_R_prev = er_R;
    //      previousMillis = currentMillis;
    //      motor_R.rotate(0, 1, motor_R.cal_pwm_R(uR));
    //      motor_L.rotate(0, 0, 0);
    //    }
    //
    //    else if (readL <= get_positionL && readR >= get_positionR)
    //    {
    //      v_L = vel_L.cal_vel(readL, deltaT);
    //      delay(20);
    //      er_L = v_set - v_L;
    //      float debt_L = (er_L - pid_L.error_prev) / deltaT;
    //      float eintegral_L = eintegral_L + (er_L * deltaT);
    //      float uL = (kp * er_L) + (ki * eintegral_L) + (kd * debt_L);
    //      float vL_pid = pid_L.cal_pid(er_L, er_L_prev, deltaT, eintegral_L);
    //      er_L_prev = er_L;
    //      previousMillis = currentMillis;
    //      motor_R.rotate(0, 0, 0);
    //      motor_L.rotate(0, 1, motor_L.cal_pwm_L(uL));
    //    }
    else
    {
      v_L = vel_L.cal_vel(readL, deltaT);
      v_R = vel_R.cal_vel(readR, deltaT);
      delay(5);
      er_L = v_set - v_L;
      er_R = v_set - v_R;
      float debt_R = (er_R - pid_R.error_prev) / deltaT;
      float eintegral_R = eintegral_R + (er_R * deltaT);
      float uR = (kp * er_R) + (ki * eintegral_R) + (kd * debt_R);
      float debt_L = (er_L - pid_L.error_prev) / deltaT;
      float eintegral_L = eintegral_L + (er_L * deltaT);
      float uL = (kp * er_L) + (ki * eintegral_L) + (kd * debt_L);
      float vL_pid = pid_L.cal_pid(er_L, er_L_prev, deltaT, eintegral_L);
      float vR_pid = pid_R.cal_pid(er_R, er_R_prev, deltaT, eintegral_R); // just keep the value of er_prev
      er_L_prev = er_L;
      er_R_prev = er_R;
      previousMillis = currentMillis;
      double PMW_R = motor_R.cal_pwm_R(uR);
      double PMW_L = motor_L.cal_pwm_L(uL);
      if (abs(PMW_R) > 25 || abs(PMW_L) > 25) {
        PMW_R = 25;
        PMW_L = 25;
      } // prevent pwm values too high

      if (uR > 0) {
        dir_R_1 = 0;
        dir_R_2 = 1;
      } //move wheel R forward

      if (uL > 0) {
        dir_L_1 = 1;
        dir_L_2 = 0;
      } //move wheel L forward

      motor_R.rotate(dir_R_1, dir_R_2, abs(PMW_R));
      motor_L.rotate(dir_L_1, dir_L_2, abs(PMW_L));
//      Serial.print(PMW_R);
//        Serial.print(" ");
//        Serial.println(PMW_L);
    }
        Serial.print(v_set);
        Serial.print(" ");
        Serial.print(v_L);
        Serial.print(" ");
        Serial.println(v_R);
    //    Serial.println(deltaT);


  }
}
