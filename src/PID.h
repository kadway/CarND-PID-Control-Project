#ifndef PID_H
#define PID_H
#include <vector>


#define MAX_TWIDDLE_RUNS 0

using std::vector;

class PID {
public:
    /**
     * Constructor
     */
    PID();
    
    /**
     * Destructor.
     */
    virtual ~PID();
    
    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp, double Ki, double Kd);
    
    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte);
    
    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError();
    
    /**
     * Update the gain parameters with twiddle.
     */  
    
    void Twiddle(double cte);
    
    
    double Speed(double cte);
    
private:
    /**
     * PID Errors
     */
    //double p_error;
    //double i_error;
    //double d_error;
    
    /**
     * PID Coefficients
     */ 
    //double Kp;
    //double Ki;
    //double Kd;
    int timestep, idx, runtime, twiddle_run;
    double best_err;
    vector <double> err;
    vector <double> gain;
    vector <double> factors;
};

#endif  // PID_H