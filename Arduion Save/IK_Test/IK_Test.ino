void setup() {
  Serial.begin(9600);
  IKArmRun (250,-350,-90);
}

void loop() {

}

void IKArmRun (int xi, int yi, int QA)
  {
  float L1,L2,L3,x2,y2,QR0,QR1,QR2,QR3,QA0,QA1,QA2,QA3,C1,S1,C2,y3;    //C1 = Cos(QR1), C2 = Cos(QR2)
  L1 = 210;
  L2 = 150;
  L3 = 200;
  y3 = -yi;
  QA0 = -QA;
  QR0 = QA0*(M_PI/180);
  x2 = xi-(L3*cos(QR0));
  y2 = y3-(L3*sin(QR0));
  C2 = (((x2*x2)+(y2*y2)-(L1*L1)-(L2*L2))/(2*L1*L2));
  QR2 = acos(C2);
  C1 = (((L1+(L2*C2))*x2)+(L2*y2*sin(QR2)))/((x2*x2)+(y2*y2));
  S1 = (((L1+(L2*C2))*y2)-(L2*x2*sin(QR2)))/((x2*x2)+(y2*y2));
  QR1 = atan(S1/C1);
  QR3 = QR0-(QR1+QR2);
  QA1 = QR1*(-180/M_PI);
  QA2 = QR2*(180/M_PI);
  QA3 = QR3*(180/M_PI);
  Serial.print(QA1);
  Serial.print(",");
  Serial.print(QA2);
  Serial.print(",");
  Serial.println(QA3);
//  ArmRunToDeg(QA1,QA2,QA3,Speed,Acceleration);
  }
