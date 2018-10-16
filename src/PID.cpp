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

  iteration = 0;
  total_error = 0;
  best_error = 100000.0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  if (iteration > 0) {
    d_error = (cte - prev_cte);
  } 
  i_error += cte;
  prev_cte = cte;
  if (iteration < 100) {
    total_error = 0;
    iteration++;
  }
  else if (iteration < 2000) {
    total_error += (cte * cte);
    iteration++;
    if (best_error > total_error / (iteration - 100)) {
      best_error = total_error / (iteration - 100);
    }
  }
}

double PID::TotalError() {
  return total_error/(iteration-100);
}

double PID::GetAdjustment() {
  return -(Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}

