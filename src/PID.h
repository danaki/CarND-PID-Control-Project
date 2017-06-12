#include <deque>

#ifndef PID_H
#define PID_H

class PID {
public:
  double prev_cte;
  std::deque<double> cte_history;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double GetSteer();

  double SumHistoryCte();

  double SumHistoryCte2();
};

#endif /* PID_H */
