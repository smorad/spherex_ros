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

// r v q w
#define Nsta 13 // num states
// r v q w
#define Mobs 13   // num observations
// TODO set dt to actual value
#define radius 0.15

#include "consts.h"
#include "TinyEKF.h"

class SphereXEKF : public TinyEKF {
public:
    double dt;
    SphereXEKF();
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
};

#endif /* EKF_HPP */

