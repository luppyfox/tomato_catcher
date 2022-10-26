#include "Motor.h"

#define MAX_PWM 100

Motor motorl;
Motor motorr;

float wheel_circumference = 68.5; //cm
const unsigned int WordLength = 8;
int getPWML;
int getPWMR;

void setup() {
  Serial.begin(115200);
  motorl.init(5, 7, 4, 150);
  motorr.init(6, 9, 8, 150);
}

void loop() {
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
      Serial.print("input = ");
      Serial.println(Command);
      int Data0 = (Command[0]);
      int Data1 = (Command[1] - '0') * 100;
      int Data2 = (Command[2] - '0') * 10;
      int Data3 = (Command[3] - '0');
      if (Data0 == 43)
      {
        getPWML = (Data1 + Data2 + Data3);
      }
      else if (Data0 == 45)
      {
        getPWML = -(Data1 + Data2 + Data3);
      }
      else
      {
        getPWML = 0;
      }
      int Data4 = (Command[4]);
      int Data5 = (Command[5] - '0') * 100;
      int Data6 = (Command[6] - '0') * 10;
      int Data7 = (Command[7] - '0');
      if (Data4 == 43)
      {
        getPWMR = -(Data5 + Data6 + Data7);
      }
      else if (Data4 == 45)
      {
        getPWMR = (Data5 + Data6 + Data7);
      }
      else
      {
        getPWMR = 0;
      }
//      Serial.print(getPWML);
//      Serial.print(" ");
//      Serial.println(getPWMR);
      motorl.rotate(getPWMR);
      motorr.rotate(getPWML);
    }
  }
}
