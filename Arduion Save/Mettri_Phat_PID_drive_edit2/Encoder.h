class Encoder
{
  private:
  public:
    void init(int enca_IN, int encb_IN, int MaxTick, float wheel_circumference_set);
    int enc_a, enc_b;
    void pulse_a_change();
    //void pulse_b_change();
    volatile long pulse = 0;
    float cal_cm();
    float ppr; // ppr
    float wheel_circumference;
};

void Encoder::init(int enca_IN, int encb_IN, int MaxTick, float wheel_circumference_set)
{
  this->enc_a = enca_IN;
  this->enc_b = encb_IN;
  ppr = MaxTick;
  wheel_circumference = wheel_circumference_set;

  pinMode(enc_a, INPUT_PULLUP); //change analog pin to digital input
  pinMode(enc_b, INPUT_PULLUP); //change analog pin to digital input
}

void Encoder::pulse_a_change()
{
  if (digitalRead(this->enc_b) == 0)
  {
    if (digitalRead(this->enc_a) == 0)
    {
      this->pulse--; //moving forward
    }
    else
    {
      this->pulse++; //moving reverse
    }
  }
  else
  {
    if (digitalRead(enc_a) == 0)
    {
      this->pulse++;
    }
    else
    {
      this->pulse--;
    }
  }
}
/*void Encoder::pulse_b_change()
  {
    if (digitalRead(this->enc_a) == 0)
    {
        if(digitalRead(this->enc_b) == 0)
        {
            this->pulse++; //moving forward
        }
        else
        {
            this->pulse--; //moving reverse
        }
    }
    else
    {
        if(digitalRead(enc_b) == 0)
        {
            this->pulse--;
        }
        else
        {
            this->pulse++;
        }
    }
  }*/

float Encoder::cal_cm()
{
  // change tick to cm
  return ((wheel_circumference / ppr) * this->pulse);
}
