#include "PID.h"
#include <iostream>
#include <vector>
using std::vector;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    /**
     * TODO: Initialize PID coefficients (and errors, if needed)
     */
    //this->Kp = Kp_;
    //this->Ki = Ki_;
    //this->Kd = Kd_;
    //p_error = 0;
    //i_error = 0;
    //d_error = 0;
    this->state_idx = 0;
    this->state = 0;
    this->best_err = 999;
    this->gain.push_back(Kp);
    this->gain.push_back(Ki);
    this->gain.push_back(Kd);
    
    for(int j=0; j< 3; j++){
        this->err.push_back(0);
        this->factors.push_back(0.0001);
    }
}

void PID::UpdateError(double cte) {
    /**
     * TODO: Update PID errors based on cte.
     */
    err[2] = cte - err[0];
    err[0] = cte;
    err[1] += cte;
    
}

double PID::TotalError() {
    /**
     * TODO: Calculate and return the total error
     */
    double steer = 0;
    for(int j= 0; j< 3; j++){
        steer -= gain[j]*err[j];
    }
    
    return std::min(std::max(steer,-1.0),1.0);
}

void PID::Twiddle(double cte) {
    double factor_sum = 0;
    int i = this->state_idx;
    
    for (unsigned int j=0;j<factors.size();j++){
        factor_sum+=factors[j];
    }
    std::cout << "factor_sum: " << factor_sum << std::endl;
    if(factor_sum > 0.00001){
        switch (state) {
            
            //state 1:
            case 0:
                gain[i] += factors[i];
                this->state +=1;
                break;
                
            case 1:
                //state 2:
                if (cte < best_err){
                    best_err = cte;     
                    factors[i] *= 1.1;
                    
                    if(i==2){
                        this->state_idx=0;
                    }
                    else{
                        this->state_idx += 1;
                    }
                    this->state =0;
                }
                else{
                    gain[i] -= 2 * factors[i];
                    this->state +=1;
                }
                
                break;
                
            case 2:  
                //state 3:
                if (cte < best_err){
                    best_err = cte;
                    factors[i] *= 1.1;
                }
                else{
                    gain[i] += factors[i];
                    factors[i] *= 0.9;                 
                }
                
                this->state=0;
                if(i==2){
                    this->state_idx=0;
                }
                else{
                    this->state_idx += 1;
                }
                break;
        }       
    }
    
    std::cout << "Kp:" << this->gain[0] << " Ki:" << this->gain[1] << " Kd:" << this->gain[2] << std::endl;
}