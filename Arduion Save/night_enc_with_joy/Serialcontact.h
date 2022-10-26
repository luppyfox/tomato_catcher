class Serialcontact
{
  private:
  public:
    int getposition;
    float joy_value[8];
    double update();

};

double Serialcontact::update()
{
  for (int i = 0 ; i < 8; i++)
  {
    joy_value[i] = Serial.read();
  }
  return (joy_value);
}
