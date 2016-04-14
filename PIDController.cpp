#include "PIDController.h"

int PIDController::pid(float err) {
  float P = err*kP;
  I = I + err*kI;
  float D = (err-prev_err)*kD;
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

float PIDController::getP() const {
  return kP;
}

float PIDController::getI() const {
  return kI;
}

float PIDController::getD() const {
  return kD;
}

