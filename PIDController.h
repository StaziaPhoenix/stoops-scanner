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
    void setThreshold(float newVal);
    float getThreshold();
  private:
    float kP_t = 1.6; 
    float kD_t = 2;
    
    float kP_l = 0.8;
    float kD_l = 0.2;
    
    int prev_err = 0;
    float kI = 0;
    float I = 0;
    const byte outMax = 60;
    float threshold = 1.20;
};

#endif
