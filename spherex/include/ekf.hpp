/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ekf.hpp
 * Author: smorad
 *
 * Created on April 9, 2018, 6:55 PM
 */

#ifndef EKF_HPP
#define EKF_HPP

// number of states we use
// wx wy wz 
// q0 q1 q2 q3
// xdotdot ydotdot zdotdot
// number of states we care about
// q0 q1 q2 q3
// q0dot q1dot q2dot q3dot
// x y z
// xdot ydot zdot
#define Nsta 14

// number of sensors
// lidar
// imu
#define Mobs 2


#include "consts.h"
#include "TinyEKF.h"

class SphereXEKF : public TinyEKF {
    void Fuser();
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
};

#endif /* EKF_HPP */

