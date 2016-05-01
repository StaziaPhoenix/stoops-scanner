#include "PIDController.h"

int PIDController::pid(float err, byte & speedy) {
  float change = err-prev_err;

  if (go && speedy > 5 && change > 0.8) {
    speedy = speedy - 5;
  }

  if (change > threshold) {
    float pid = err*kP_t+I + err*kI+(change)*kD_t;
    prev_err = err;
    return clamp(pid);
  } else {
    if (go && speedy < 90) {
      speedy = speedy + 1;
    }
    float pid = err*kP_l+I + err*kI +(change)*kD_l;
    prev_err = err;
    return clamp(pid);
  }
}

void PIDController::setGo(byte state) {
  go = state;
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
void PIDController::incP_t() {
  kP_t += 0.01;
}

void PIDController::incD_t() {
  kD_t += 0.05;
}

void PIDController::decP_t() {
  kP_t -= 0.01;
}

void PIDController::decD_t() {
  kD_t -= 0.05;
}

float PIDController::getP_t() const {
  return kP_t;
}

float PIDController::getD_t() const {
  return kD_t;
}

void PIDController::incP_l() {
  kP_l += 0.01;
}

void PIDController::incD_l() {
  kD_l += 0.01;
}

void PIDController::decP_l() {
  kP_l -= 0.01;
}

void PIDController::decD_l() {
  kD_l -= 0.01;
}

float PIDController::getP_l() const {
  return kP_l;
}

float PIDController::getD_l() const {
  return kD_l;
}

void PIDController::setThreshold(float newVal) {
  threshold = newVal;
}

float PIDController::getThreshold() {
  return threshold;
}

