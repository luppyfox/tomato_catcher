class Motor
{
  private:
    int en, in1, in2;
    int d1, d2, pwm_out;
    float v_pid_L, v_pid_R;
  public:
    Motor();
    void init(int enIn, int in1In, int in2In);
    void rotate(int dig_in1, int dig_in2, int pwm_IN);
    float cal_pwm_L(float vel_pid_L);
    float pwm_L = 0;
    float cal_pwm_R(float vel_pid_R);
    float pwm_R = 0;
};

Motor::Motor() {}

void Motor::init(int enIn, int in1In, int in2In)
{
  en = enIn;
  in1 = in1In;
  in2 = in2In;

  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  rotate(0, 0, 0);
}

void Motor::rotate(int dig_in1, int dig_in2, int pwm_IN)
{
  d1 = dig_in1;
  d2 = dig_in2;
  pwm_out = pwm_IN;
  digitalWrite(in1, d1);
  digitalWrite(in2, d2);
  analogWrite(en, pwm_out);

}

float Motor::cal_pwm_L(float vel_pid_L)
{
  v_pid_L = vel_pid_L;
  pwm_L = ((0.0001 * v_pid_L * v_pid_L * v_pid_L) - (0.0064 * v_pid_L * v_pid_L) + (1.0741 * v_pid_L) + 4.8535); //wheel L
  return (pwm_L); // return pwm
}
float Motor::cal_pwm_R(float vel_pid_R)
{
  v_pid_R = vel_pid_R;
  pwm_R = ((0.0001 * v_pid_R * v_pid_R * v_pid_R) - (0.0106 * v_pid_R * v_pid_R) + (1.1394 * v_pid_R) + 4.4654); //wheel R
  return (pwm_R); // return pwm
}
