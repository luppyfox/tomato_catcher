class PID
{
  private:
    long d_T;
    float kp, ki, kd;

  public:
    void init(float kp_set, float ki_set, float kd_set);
    float cal_pid(float error_v_IN, float error_v_prev_IN,  int deltaT);
    float u;
    float error_v, eintegral, error_prev, dedt;
};

void PID::init(float kp_set, float ki_set, float kd_set)
{
  kp = kp_set;
  ki = ki_set;
  kd = kd_set;
}

float PID::cal_pid(float error_v_IN, float error_v_prev_IN, int deltaT)
{
  d_T = deltaT;
  // error
  error_v = error_v_IN;
  error_prev = error_v_prev_IN;
  //derivative term
  dedt = (error_v - error_prev) / d_T;
  // integral term
  eintegral = eintegral + (error_v * d_T);
  // compute control signal
  u = kp * error_v + ki * eintegral + kd * dedt;
  //control motor

  return (u);
}
