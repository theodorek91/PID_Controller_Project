#ifndef PID_H
#define PID_H
#include <string>
#include <vector>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  std::vector<double> Params;
  float p_inc;
  float d_inc;
  float i_inc;
  int loop_number;
  double temp_storage;
  std::vector<double> temp_storage2;
  int prev_param_index;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  std::vector<double> dp;
  int iteration;
  int total_error;
  double best_error;
  int index_param;
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
  void UpdateError(double Kp, double Ki, double Kd, double cte);

  /*
  * Calculate the total PID error.
  */
  
  
  double TotalError();
};

#endif /* PID_H */
