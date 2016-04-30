#include "PIDController.h"

int PIDController::pid(float err) {
  float change = err-prev_err;
  if (change > threshold) {
    float pid = err*kP_t+I + err*kI+(change)*kD_t;
    prev_err = err;
    return clamp(pid);
  } else {
    float pid = err*kP_l+I + err*kI +(change)*kD_l;
    prev_err = err;
    return clamp(pid);
  }
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
  kP_t += 0.01;
}

void PIDController::incI() {
  kI += 0.05;
}

void PIDController::incD() {
  kD_t += 0.05;
}

void PIDController::decP() {
  kP_t -= 0.01;
}

void PIDController::decI() {
  kI -= 0.05;
}

void PIDController::decD() {
  kD_t -= 0.05;
}

float PIDController::getP() const {
  return kP_t;
}

float PIDController::getI() const {
  return kI;
}

float PIDController::getD() const {
  return kD_t;
}

void PIDController::setThreshold(float newVal) {
  threshold = newVal;
}

float PIDController::getThreshold() {
  return threshold;
}

