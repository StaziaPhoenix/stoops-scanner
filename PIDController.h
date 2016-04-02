#ifndef PIDController_H
#define PIDController_H

#include "Arduino.h"

class PIDController {
  public:
    PIDController() {}
    int pid(float err, float dt);
    int pi(float err, float dt);
  private:
    const int kP = 1.2;
    const int kI = 0;
    const int kD = 0;
    int prev_err = 0;
    float I = 0;
};

#endif
