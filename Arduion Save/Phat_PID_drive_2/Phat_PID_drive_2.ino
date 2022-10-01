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
int get_position = 1000; //recieve position move from AI with arm
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

PID pid_L;
PID pid_R;
float kp = 1.8;
float ki = 0.01;//0.01
float kd = 10;
float er_L, er_R, er_L_prev, er_R_prev;

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
  Serial.begin(115200);
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
  currentMillis = millis();
  v_set = (get_position) / 3;
  if (v_set >= 10)
  {
    v_set = 10;
  }
  int deltaT = currentMillis - previousMillis;
  //Serial.println(previousMillis);
  float readL = enc_L.cal_cm() * (-1);
  float readR = enc_R.cal_cm();
  v_L = vel_L.cal_vel(readL, deltaT);
  v_R = vel_R.cal_vel(readR, deltaT);
  delay(20);
  er_L = v_set - v_L;
  er_R = v_set - v_R;

  float debt_R = (er_R - pid_R.error_prev) / deltaT;
  float eintegral_R = eintegral_R + (er_R * deltaT);
  float uR = (kp * er_R) + (ki * eintegral_R) + (kd * debt_R);

  float debt_L= (er_L - pid_L.error_prev) / deltaT;
  float eintegral_L = eintegral_L + (er_L * deltaT);
  float uL = (kp * er_L) + (ki * eintegral_L) + (kd * debt_L);

  float vL_pid = pid_L.cal_pid(er_L, er_L_prev, deltaT, eintegral_L);
  float vR_pid = pid_R.cal_pid(er_R, er_R_prev, deltaT, eintegral_R);
  er_L_prev = er_L;
  er_R_prev = er_R;

  Serial.print(v_set);
  Serial.print(" ");
  Serial.print(v_L);
  Serial.print(" ");
  Serial.println(v_R);

  //  Serial.print("Vel Left :: ");
  //  Serial.print(v_set);
  //  Serial.print("Vel Left :: ");
  //  Serial.print(v_R);
  //Serial.print("\t\t");
  //  Serial.print("Vel Right :: ");
  //  Serial.print(eintegral);
  //  Serial.print("\t\t cm_Left :: ");
  //  Serial.print(debt);
  //  Serial.print("\t\t cm_Right :: ");
  //  Serial.println(u);
  //  Serial.print("\t\t pwm :: ");
  //  Serial.println(motor_R.cal_pwm_R(u));

  previousMillis = currentMillis;
  motor_R.rotate(0, 1, motor_R.cal_pwm_R(uR));
  motor_L.rotate(0, 1, motor_L.cal_pwm_L(uL));
  /*if (get_position >= readL || get_position >= readR) //move forward//move forward
    {
    motor_L.rotate(1, 0, motor_L.cal_pwm_L(vL_pid));
    motor_R.rotate(1, 0, motor_R.cal_pwm_R(vR_pid));
    if (get_position <= readL || get_position <= readR)
    {
      motor_L.rotate(0, 0, 0);
      motor_R.rotate(0, 0, 0);
    }
    }
    if (get_positio
    n <= readL || get_position <= readR)//move backward
    {
    motor_L.rotate(0, 1, motor_L.cal_pwm_L(vL_pid));
    motor_R.rotate(0, 1, motor_R.cal_pwm_R(vR_pid));
    //    if (get_position >= readL || get_position >= readR) //move forward//move forward
    //    {
    //      motor_L.rotate(0, 0, 0);
    //      motor_R.rotate(0, 0, 0);
    //    }
    }
    else
    {
    motor_L.rotate(0, 0, 0);
    motor_R.rotate(0, 0, 0);
    }*/
}
