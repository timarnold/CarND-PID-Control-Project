#ifndef PID_H
#define PID_H

#import <iostream>

class PID
{
public:
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID with gain values and should_twiddle parameter
  */
  void Init(double Kp, double Ki, double Kd, bool should_twiddle);

  /*
  * Calculate a steering value for current coefficients
  */
  double CalculateSteering(double dt);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

private:
  /*
  * Errors
  */
  double errors[3];

  /*
  * PID Gain Coefficients
  */
  double params[3];

  /**
  * Use Twiddle method described in PID lectures to search
  * PID gain parameter space.
  */
  void Twiddle();

  /*
  * Twiddling Private Parameters
  */
  int p_i;
  int step;
  bool should_twiddle_;
  bool correcting_overshoot;
  int max_steps;
  double dp[3];
  double running_error;
  double best_error;
};

#endif /* PID_H */
