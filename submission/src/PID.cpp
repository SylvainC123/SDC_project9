#include "PID.h"
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, 
			   double dv_Kp, double dv_Ki, double dv_Kd,
			   double thr_max, double thr_min, double cte_max) {
				   
    this->Kp0 = Kp;
    this->Ki0 = Ki;
    this->Kd0 = Kd;
    
    this->dv_Kp = dv_Kp;
    this->dv_Ki = dv_Ki;
    this->dv_Kd = dv_Kd;
    
    this->thr_max = thr_max;
    this->thr_min = thr_min;
    this->cte_max = cte_max;
    
    cte_sum = 0;
    cte_prev = 0;
}

void PID::UpdateCoefficients(double Kp, double Ki, double Kd, 
							double dv_Kp, double dv_Ki, double dv_Kd,
							double v) {
								
	this->Kp = Kp0 + dv_Kp * v;
    this->Ki = Ki0 + dv_Ki * v;
    this->Kd = Kd0 + dv_Ki * v;
}

void PID::UpdateError(double cte) {
    cte_sum = 0.95*cte_sum + cte;	//not used
    p_error = - Kp * cte;
    i_error = - Ki * cte_sum;
    d_error = - Kd * (cte - cte_prev);
    cte_prev = cte;  
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

double PID::Throtle(double cte) {
  double thr = thr_max;
  if (fabs(cte)> cte_max){
	thr = thr_min;
  }
  return thr;
}
