#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    
  //initialize errors
    i_error = 0;
    cte_old = 0;
  
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  
  //saturation
  if (i_error > 1/Ki)
  {
    i_error = 1/Ki;
  }
  else if (i_error < -1/Ki)
  {
    i_error = -1/Ki;
  }
  
  d_error = cte - cte_old;
  cte_old = cte;
  

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return i_error;
  
}

double PID::Output() {

  
  //return steering
  double raw_steer = -Kp * p_error - Ki*i_error - Kd * d_error;
  
  //saturation to -1 -> 1
  if (raw_steer > 1)
  {
    raw_steer = 1;
  }
  else if (raw_steer < -1)
  {
    raw_steer = -1;
  }
  
  return raw_steer;
  
  
}