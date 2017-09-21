#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp0;   // initial parameters
  double Ki0;
  double Kd0;
  
  double Kp;	// live parameters
  double Ki;
  double Kd;
  
  double dv_Kp;	// parameter sensitivity to speed
  double dv_Ki;
  double dv_Kd;
  
  double thr_max;	// max throttle
  double thr_min;	// min throttle
  double cte_max;	// limit for deceleration
  
  double cte_sum;	// to computr I
  double cte_prev;	// to compute D

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd,
			double dv_Kp, double dv_Ki, double dv_Kd,
			double thr_max, double thr_min, double cte_max);

  /*
  * Update coefficients which vary with speed
  */
  void UpdateCoefficients(double Kp, double Ki, double Kd, 
			   double dv_Kp, double dv_Ki, double dv_Kd,
			   double v);
  
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
 
   /*
  * Calculate the throtle value.
  */ 
  double Throtle(double cte);
};

#endif /* PID_H */
