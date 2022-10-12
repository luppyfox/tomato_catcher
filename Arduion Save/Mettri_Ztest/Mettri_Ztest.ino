#include <AccelStepper.h>
AccelStepper stepper1(1, 3, 13);
//Set Default Speed
int SDSp = 10000;
//Set Default Acceleration
int SDAc = 5000;
const unsigned int WordLength = 12;

void setup()
{
  Serial.begin(9600);
  stepper1.disableOutputs();
  pinMode(A0,INPUT_PULLUP);
  set0();
  stepper1.setMaxSpeed(SDSp);
  stepper1.setAcceleration(SDAc);
  stepper1.setCurrentPosition(0);
  delay(1000);
  Serial.println("StandBy");
}

void loop()
{
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
      int Data3 = (Command[3] - '0') ;
      double DataZ;
      if (Data0 == 43)
      {
        DataZ = Data1 + Data2 + Data3;
      }
      else if (Data0 == 45)
      {
        DataZ = (-1) * (Data1 + Data2 + Data3);
      }
      Serial.print("Z = ");
      Serial.println(DataZ);
      float HigToStepRate = -186.776242;
      float Step1 = DataZ*HigToStepRate;
      stepper1.moveTo(Step1);
      while (stepper1.distanceToGo()!=0)
      {
        stepper1.run();
      }
      Serial.println("StandBy");
      WordPosition = 0;
    }
  }
}


void set0()
{
  stepper1.setMaxSpeed(2500);
  stepper1.setAcceleration(SDAc);
  stepper1.moveTo(1000000);
  while (1)
  {
    stepper1.run();
    if (digitalRead(A0) == LOW)
    {
      stepper1.setCurrentPosition(0);
      delay(1000);
      break;
    }
  }
}
