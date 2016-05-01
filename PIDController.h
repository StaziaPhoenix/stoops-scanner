#ifndef PIDController_H
#define PIDController_H

#include "Arduino.h"

class PIDController {
  public:
    PIDController() {}
    int pid(float err, byte & speedy);
    void incP_t();
    void incD_t();
    void decP_t();
    void decD_t();
    float getP_t() const;
    float getD_t() const;
    void incP_l();
    void incD_l();
    void decP_l();
    void decD_l();
    float getP_l() const;
    float getD_l() const;
    int clamp(float pid);
    void setThreshold(float newVal);
    float getThreshold();
    void setGo(byte state);
  private:
    float kP_t = 1.6; 
    float kD_t = 2;

    float kP_l = 0.8;
    float kD_l = 0.2;
    
    float prev_err = 0;
    float prev_change = 0;
    float kI = 0;
    float I = 0;
    const byte outMax = 63;
    float threshold = 1.10;
    byte go = 0;
};

#endif
