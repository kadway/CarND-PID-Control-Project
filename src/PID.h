#ifndef PID_H
#define PID_H
#include <vector>

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
    int state, state_idx;
    double best_err;
    vector <double> err;
    vector <double> gain;
    vector <double> factors;
};

#endif  // PID_H