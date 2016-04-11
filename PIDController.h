#ifndef PIDController_H
#define PIDController_H

#include "Arduino.h"

class PIDController {
  public:
    PIDController() {}
    int pid(float err, float dt);
    void incP();
    void incI();
    void incD();
    void decP();
    void decI();
    void decD();
    float getP();
    float getI();
    float getD();
    int clamp(float pid);
  private:
    float kP = .7; 
    float kI = 0;
    float kD = 2.50;
    int prev_err = 0;
    float I = 0;
    byte outMax = 60;
};

#endif
