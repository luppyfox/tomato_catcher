#ifndef SENDMOVE_H
#define SENDMOVE_H

#include <Arduino.h>

class SendMove
{
  private:
    int D;
  public:
    int evalu(int getD);
    int sendD();
};
int SendMove::evalu(int getD)
{
  D = getD;
}
int SendMove::sendD()
{
  return D;
}

#endif
