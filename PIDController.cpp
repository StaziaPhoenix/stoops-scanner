#include "PIDController.h"

int PIDController::pid(float err, float dt) {
  float P = err*kP;
  float I = I + err*dt*kI;
  float D = (err-prev_err)/dt*kD;
  prev_err = err;
  return clamp(P+I+D);
}

int PIDController::clamp (float pid) {
  if (pid > outMax) {
    return outMax;
  } else if (pid < -outMax) {
    return -outMax;
  } else {
    return pid;
  }
}
void PIDController::incP() {
  kP += 0.05;
}

void PIDController::incI() {
  kI += 0.05;
}

void PIDController::incD() {
  kD += 0.05;
}

void PIDController::decP() {
  kP -= 0.05;
}

void PIDController::decI() {
  kI -= 0.05;
}

void PIDController::decD() {
  kD -= 0.05;
}

float PIDController::getP() {
  return kP;
}

float PIDController::getI() {
  return kI;
}

float PIDController::getD() {
  return kD;
}

