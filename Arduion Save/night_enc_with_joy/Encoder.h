#ifndef ENCODER_H
#define ENCODER_H

//#include <Arduino.h>

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

class Encoder
{
  private:
    /* data */
  public:
    Encoder();
    void init(int enaIN, int enbIN, bool invertIn, float ppr_IN, float wheel_circumference_IN);
    void update();
    float cal_cm();

    int ena, enb;
    bool invert;
    volatile int position = 0;
    float ppr, wheel_circumference, cm;
};

Encoder::Encoder()
{
}

void Encoder::init(int enaIN, int enbIN, bool invertIn, float ppr_IN, float wheel_circumference_IN)
{
  this->ena = enaIN;
  this->enb = enbIN;
  this->invert = invertIn;
  this->ppr = ppr_IN;
  this->wheel_circumference = wheel_circumference_IN;

  pinMode(ena, INPUT_PULLUP);
  pinMode(enb, INPUT_PULLUP);
}

void Encoder::update()
{
  if (this->invert)
  {
    if (digitalRead(this->enb) == LOW)
    {
      if (this->position == encoder_maximum)
      {
        this->position = encoder_minimum;
      }
      else
      {
        this->position++;
      }
    }
    else
    {
      if (this->position == encoder_minimum)
      {
        this->position = encoder_maximum;
      }
      else
      {
        this->position--;
      }
    }
  }
  else
  {
    if (digitalRead(this->enb) == LOW)
    {
      if (this->position == encoder_minimum)
      {
        this->position = encoder_maximum;
      }
      else
      {
        this->position--;
      }
    }
    else
    {
      if (this->position == encoder_maximum)
      {
        this->position = encoder_minimum;
      }
      else
      {
        this->position++;
      }
    }
  }
}

float Encoder::cal_cm()
{
  cm = abs(((this->wheel_circumference / this->ppr) * this->position));
  return(cm);
}
#endif
