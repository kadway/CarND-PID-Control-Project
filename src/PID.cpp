#include "PID.h"
#include <iostream>
#include <vector>
#include <math.h>  
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

    this->idx = 0;
    this->timestep = 0;
    this->best_err = 9999;
    this->runtime = 0;
    this->twiddle_run = 0;
    this->gain.push_back(Kp);
    this->gain.push_back(Ki);
    this->gain.push_back(Kd);
    
    //initialize errors and factors for each gain component
    for(int j=0; j< 3; j++){
        this->err.push_back(0);
        this->factors.push_back(0.01); //factor for Kp
        this->factors.push_back(0); //factor for Ki
        this->factors.push_back(0.01); //factor for Kd
    }
}

void PID::UpdateError(double cte) {
    /**
     * TODO: Update PID errors based on cte.
     */
    
    this->runtime++;
    
    err[2] = cte - err[0]; // differential
    err[0] = cte; //proportional
    err[1] += cte; //integral
    std::cout << "---------------------------" << std::endl;
    std::cout << "Runtime: " << this->runtime << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Updated error: Kp_err= " << err[0] << " Ki_err=" << err[1] << " Kd_err=" << err[2] << std::endl;
    
}

double PID::TotalError() {
    /**
     * TODO: Calculate and return the total error
     */
    double steer = 0;
    for(int j= 0; j< 3; j++){
        steer -= gain[j]*err[j];
    }
    
    //limit steering to the range of [-1, 1]
    return std::min(std::max(steer,-1.0),1.0);
}

void PID::Twiddle(double cte) {
    double factor_sum = 0;
 
    for (unsigned int j=0;j<factors.size();j++){
        factor_sum+=factors[j];
    }
    std::cout << "factor_sum: " << factor_sum << " twiddle run: "<< twiddle_run << std::endl;
    if(factor_sum < 0.05){
        
        factors[0]=0.01; //factor for Kp
        factors[1]=0; //factor for Ki
        factors[2]=0.1; //factor for Kd
        
        twiddle_run +=1;
        std::cout << "--------------------------------" << std::endl;        
        std::cout << "          Next twiddle RUN:" << twiddle_run << std::endl;
        std::cout << "--------------------------------" << std::endl;
        
    }
    std::cout << "Time step: " << timestep << " idx:" << idx << std::endl;
    if((twiddle_run < MAX_TWIDDLE_RUNS) | (timestep> 0)){
      
        switch (timestep) {
            //timestep 1: 
            case 0: // try adding the factor to the gain
                gain[idx] += factors[idx];
                timestep +=1;
                break;
            
            //timestep 2:
            case 1: 
                
                //check is error has decreased
                if (cte < best_err){
                    //error has decreased so increment the factor for next round
                    best_err = cte;     
                    factors[idx] *= 1.1;
                    // reset timestep count to test another gain component
                    timestep =0;
                    // go to next gain component index
                    idx = (idx+1)  % 3;
                }
                // error has not improved
                else{
                    //try subtracting the factor to the gain
                    gain[idx] -= 2 * factors[idx];
                    //go to next time step to check for the resulting error
                    timestep +=1;
                }
                break;
                
            //timestep 3    
            case 2:  
                //check is error has decreased
                if (cte < best_err){
                    //error has decreased so increment the factor for next round
                    best_err = cte;
                    factors[idx] *= 1.1;
                }
                
                // error has not improved set gain back to previous value, decrease the factor
                else{
                    gain[idx] += factors[idx];
                    factors[idx] *= 0.9;                 
                }
                
                //restart from first timestep and check next gain component
                timestep=0;
                idx = (idx+1)  % 3;
                break;
        }       
    }
    //std::cout << "Updated error: Kp_err= " << err[0] << " Ki_err=" << err[1] << " Kd_err=" << err[2] << std::endl;
    std::cout << "New gain Kp:" << this->gain[0] << " Ki:" << this->gain[1] << " Kd:" << this->gain[2] << std::endl;
    std::cout << "New factors F_Kp:" << this->factors[0] << " F_Ki:" << this->factors[1] << " F_Kd:" << this->factors[2] << std::endl;
}

double PID::Speed(double cte){
    double cte_square = cte*cte;
    return std::max((double)0.5-(sqrt(cte_square)/2), 0.1);
}