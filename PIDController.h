#ifndef PIDController_H
#define PIDController_H

#include "Arduino.h"

class PIDController {
  public:
    PIDController() {}
    int pid(float err);
    void incP();
    void incI();
    void incD();
    void decP();
    void decI();
    void decD();
    float getP() const;
    float getI() const;
    float getD() const;
    int clamp(float pid);
  private:
    float kP = .55; 
    float kI = 0;
    float kD = 2.80;
    int prev_err = 0;      
    float I = 0;
    const byte outMax = 60;
};

#endif
