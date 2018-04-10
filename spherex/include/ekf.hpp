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

// number of states
// wx wy wz 
// q0 q1 q2 q3
// xdotdot ydotdot zdotdot
#define Nsta 10

// number of sensors
// lidar
// imu
# define Mobs 2

#include "TinyEKF.h"
//#include "tiny_ekf_struct.h"

#endif /* EKF_HPP */

