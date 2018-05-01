/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   spherex_kalman.h
 * Author: smorad
 *
 * Created on April 30, 2018, 2:28 PM
 */

#ifndef SPHEREX_KALMAN_H
#define SPHEREX_KALMAN_H

// r v q w
#define Nsta 13 // num states
// r v q w
#define Mobs 13   // num observations


#include <Eigen/Dense>
#include <TinyEKF.h>



//int states = 13;
//int measurements = 2;

// timestep
// TODO change to real value
double dt = 1.0;
double radius = 0.15; // m

class EKF : public TinyEKF {
    // quaternion shit
    // http://blogdugas.net/blog/2015/05/10/extended-kalman-filter-with-quaternions-for-attitude-estimation/

    Eigen::Matrix3d skew(Eigen::Vector4d q) {
        Eigen::Matrix3d result;
        result <<
         0, -q(3), q(2),
         q(3), 0, -q(1),
        -q(2), q(1), 0;
        return result;
    }
    

    double* q_dot(double q[]) {
        Eigen::Vector4d q_vect(q);
        Eigen::MatrixXd Gt(3, 4);
        Eigen::Vector4d qdot;
        //Eigen::MatrixXd result(4, 3);
        Gt << -q[1], -q[2], -q[3],
                q[0], q[3], -q[2],
                -q[3], q[0], q[1],
                q[2], -q[1], q[0];
        qdot = 0.5 * q_vect * Gt;
        double* qdot_arr = new double[4];
        for (int i = 0; i < 4; ++i) {
            qdot_arr[i] = qdot(i);
        }
        return qdot_arr;
    }

    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
        // measurements
        // r a q w
        for (int i = 0; i < Mobs; ++i) {
            // must integrate a to get v
            if (i > 2 && i < 6) {
                hx[i] = this->x[i] * dt;
            } else {
                hx[i] = this->x[i];
            }
        }
        // measurement jacobian is constant
        for (int i = 0; i < Mobs; ++i) {
            for (int j = 0; j < Mobs; ++j) {
                H[i][j] = 1;
            }
        }

        // process model
        // r 
        for (int i = 0; i < 3; ++i) {
            // x = x + wRdt
            fx[i] = this->x[i] + this->x[10 + i] * radius * dt;
        }
        // v
        for (int i = 3; i < 6; ++i) {
            // v = v
            fx[i] = this->x[i];
        }
        // q
        double q[4] = {x[6], x[7], x[8], x[9]};
        double* qdot = q_dot(q);
        for (int i = 6; i < 10; ++i) {
            // q = q*qdot
            fx[i] = this->x[i] * qdot[i - 6] * dt;
        }
        // TODO delete qdot here

        // w
        for (int i = 10; i < 13; ++i) {
            fx[i] = this->x[i];
        }

        Eigen::Quaterniond quat(qdot);
        // process model jacobian
        Eigen::MatrixXd pmodel;
        // r v q w
        pmodel <<
                // rx ry rz
                1, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ radius * dt, 0, 0,
                0, 1, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, radius * dt, 0,
                0, 0, 1, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, radius * dt,
                // v
                0, 0, 0, /**/ 1, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
                0, 0, 0, /**/ 0, 1, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
                0, 0, 0, /**/ 0, 0, 1, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
                // q TODO FIX ME
                0, 0, 0, /**/ 0, 0, 0, /**/ 1, 0, 0, 0, /**/ 0, 0, 0,
                0, 0, 0, /**/ 0, 0, 0, /**/ 0, 1, 0, 0, /**/ 0, 0, 0,
                0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 1, 0, /**/ 0, 0, 0,
                0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 1, /**/ 0, 0, 0,
                // w
                0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 1, 0, 0,
                0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 1, 0,
                0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 1;


    }
};


// Returns time derivative of quaterion
// q = q * 0.5 * G'w * dt
// q = 0.5 * dt * q * G'w




#endif /* SPHEREX_KALMAN_H */

