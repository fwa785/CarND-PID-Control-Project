#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  prev_cte = 0;

  updated = false;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  if (updated) {
    d_error = (cte - prev_cte);
  } 
  i_error += cte;
  prev_cte = cte;
  updated = true;
}

double PID::TotalError() {
  return -(Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}
