#include "Encoder.h"
//#include "PID.h"
//#include "Motor.h"

Encoder enc_L;
Encoder enc_R;
int encL_a = 2;
int encL_b = A1;
int encR_a = 3;
int encR_b = A0;

void encL_return()
{
  enc_L.pulse_a_change();
}
void encR_return()
{
  enc_R.pulse_a_change();
}

void setup()
{
  Serial.begin(115200);
  enc_L.init(encL_a, encL_b);
  enc_R.init(encR_a, encR_b);
  attachInterrupt(digitalPinToInterrupt(enc_L.enc_a), encL_return, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(enc_L.enc_b), enc_L.pulse_b_change(), CHANGE); //Arduino nano can in terrupt 2,3
  attachInterrupt(digitalPinToInterrupt(enc_R.enc_a), encR_return, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(enc_R.enc_b), enc_R.pulse_b_change(), CHANGE); //Arduino nano can in terrupt 2,3
}

void loop()
{
  Serial.print("encL_a ::   ");
  Serial.print(enc_L.pulse);
  Serial.print("             ");
  Serial.print("encR_a ::   ");
  Serial.println(enc_R.pulse);

}
