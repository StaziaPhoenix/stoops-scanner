#include "PIDController.h"

int PIDController::pid(float err, float dt) {
  float P = err*kP;
  float I = I + err*dt*kI;
  float D = (err-prev_err)/dt*kD;
  prev_err = err;
  return P+I+D;
}

int PIDController::pi(float err, float dt) {
  float P = kP*err;
  float I = I + kI*err*dt;
  prev_err = err;
  return (int) P+I;
}

