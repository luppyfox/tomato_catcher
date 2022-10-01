class Velocity
{
  private:
  public:
    float cal_vel(float cm_IN, int current_time);
    float cm;
    int time_in;
    float current_cm = 0;
    float previous_cm = 0;
    float deltaCM;
};

float Velocity::cal_vel(float cm_IN, int current_time)
{
  current_cm = cm_IN;
  time_in = current_time;
  deltaCM = current_cm - previous_cm;
  /*vel = ((deltaCM * 1000) / time_in);
  if (vel < 0)
  {

  }*/
  previous_cm = current_cm;
  return ((deltaCM * 1000) / time_in);
}
