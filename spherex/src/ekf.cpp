/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "ekf.hpp"

void SphereXEKF::Fuser() {
    // We approximate the process noise using a small constant
    this->setQ(0, 0, .0001);
    this->setQ(1, 1, .0001);

    // Same for measurement noise
    this->setR(0, 0, .0001);
    this->setR(1, 1, .0001);
    this->setR(2, 2, .0001);
}

double* 

void SphereXEKF::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
    // Process model is f(x) = x
    
    double r[] =  {this->x[0], this->x[1], this->x[2]};
    double rdot[] = {this->x[3], this->x[4], this->x[5]};
    double q[] = {this->x[6], this->x[7], this->x[8], this->x[9]};
    double qdot[] = {this->x[10], this->x[11], this->x[12], this->x[13]};
    
    // from ravi
    // xdot = f(x)
    // f(r0 + rdot) = rdot_t+dt = f(r0) + df/dr|r=r0
    
    // f()
    
    
    
    fx[0] = this->x[0];
    fx[1] = this->x[1];
    
    // q
    for(int i=6; i<10; ++i){
        fx[i] = x[i] * 
    }
    fx[6];
    fx[7];
    fx[8];
    fx[9];
    
    // fx = state transition function
    // F = jacobian of fx
    // hx = sensor function
    // H = jacobian of hx
    
    // x_k = Ax_k-1 + Bu_k or
    // F = Afx + Buk
    //A = eye(7); A(1,2), A(2,3)...A(n-1, n) = deltaT
    //X_k = [x y z q0 q1 q2 q3]'
    //X_k = A * X_k-1
    
    // q_k = q_k-1 * deltat * q_k-1
    // q_k = q_k-1 * G^t * w * 1/2 * deltat
    
    // IFF a(z) <= g
    // let pos = [x y z]'
    // let thetadot = [thetax thetay thetaz]
    // pos_k = thetadot_k-1 * r + pos_k-1
    // x = r * theta + theta_0
    

    // So process model Jacobian is identity matrix
    F[0][0] = 1;
    F[1][1] = 1;

    // Measurement function simplifies the relationship between state and sensor readings for convenience.
    // A more realistic measurement function would distinguish between state value and measured value; e.g.:
    //   hx[0] = pow(this->x[0], 1.03);
    //   hx[1] = 1.005 * this->x[1];
    //   hx[2] = .9987 * this->x[1] + .001;
    hx[0] = this->x[0]; // Barometric pressure from previous state
    hx[1] = this->x[1]; // Baro temperature from previous state
    hx[2] = this->x[1]; // LM35 temperature from previous state

    // Jacobian of measurement function
    H[0][0] = 1; // Barometric pressure from previous state
    H[1][1] = 1; // Baro temperature from previous state
    H[2][1] = 1; // LM35 temperature from previous state
}