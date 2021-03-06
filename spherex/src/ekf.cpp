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



#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <math.h>

#include "ekf.hpp"


//class SphereXEKF : public TinyEKF {

SphereXEKF::SphereXEKF() {
    for (int i = 0; i < Nsta; ++i) {
        this->setQ(i, i, 0.0001);
    }
    for (int i = 0; i < Mobs; ++i) {
        this->setR(i, i, 0.0001);
    }
}
// quaternion shit
// http://blogdugas.net/blog/2015/05/10/extended-kalman-filter-with-quaternions-for-attitude-estimation/

Eigen::Matrix3d skew(Eigen::Vector3d q) {
    Eigen::Matrix3d result;
    result <<
            0, -q(2), q(1),
            q(2), 0, -q(0),
            -q(1), q(0), 0;
    return result;
}

double* q_dot(double q[], double w[]) {
    Eigen::Vector4d q_vect(q);
    Eigen::Vector3d w_vect(w);
    Eigen::MatrixXd Gt(4, 3);
    Eigen::Vector4d qdot;
    Gt << -q[1], -q[2], -q[3],
            q[0], q[3], -q[2],
            -q[3], q[0], q[1],
            q[2], -q[1], q[0];
    // 1x4 * 4x3
    qdot = 0.5 * Gt * w_vect;
    double* qdot_arr = new double[4];
    for (int i = 0; i < 4; ++i) {
        qdot_arr[i] = qdot(i);
    }
    return qdot_arr;
}

double* compute_q(double q[], double w[], double dt) {
    // Given q and w at t, computes q at t+1
    Eigen::MatrixXd Omega(4, 4);
    Eigen::Vector3d w_vect(w);
    Eigen::Vector4d q_vect(q);
    Eigen::Vector4d result;
    // Omega[omega] = 0.5 * [skew(w) w; -w' 0  ]
    Omega << skew(w_vect), w_vect,
            -w_vect.transpose(), 0;
    //result = exp(Omega * dt) * q_vect;
    result = (Omega * dt).exp() * q_vect;
    double* q_arr = new double[4];
    for (int i = 0; i < 4; ++i) {
        q_arr[i] = result(i);
    }
    return q_arr;
}

void SphereXEKF::debug(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Mobs]) {
    std::cerr << "dt " << dt << std::endl;

    std::cerr << "current state" << std::endl;
    for (int i = 0; i < Nsta; ++i) {
        std::cerr << this->getX(i) << " ";
    }
    std::cerr << std::endl;

    std::cerr << "process model" << std::endl;
    for (int j = 0; j < Nsta; ++j) {
        std::cerr << fx[j] << " ";
    }
    std::cerr << std::endl;

    std::cerr << "process jacobian" << std::endl;
    for (int i = 0; i < Nsta; ++i) {
        for (int j = 0; j < Nsta; ++j) {
            std::cerr << F[i][j] << "   ";
        }
        std::cerr << std::endl;
    }

    std::cerr << "sensor model" << std::endl;
    for (int j = 0; j < Mobs; ++j) {
        std::cerr << hx[j] << " ";
    }
    std::cerr << std::endl;

    std::cerr << "sensor jacobian" << std::endl;
    for (int i = 0; i < Mobs; ++i) {
        for (int j = 0; j < Mobs; ++j) {
            std::cerr << H[i][j] << "   ";
        }
        std::cerr << std::endl;
    }
}

void SphereXEKF::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
    std::cerr << "start model" << std::endl;
    // measurements
    // r v q w
    for (int i = 0; i < Mobs; ++i) {
        // must integrate a to get v
        if (i > 2 && i < 6) {
            hx[i] = this->x[i] * dt;
        } else {
            hx[i] = this->x[i];
        }
    }
    // measurement jacobian
    Eigen::MatrixXd smodel(Mobs, Mobs);
    smodel <<
            // r v q w
            // r
            1, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
            0, 1, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
            0, 0, 1, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
            // a
            0, 0, 0, /**/ dt, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, dt, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, dt, /**/ 0, 0, 0, 0, /**/ 0, 0, 0,
            // q
            0, 0, 0, /**/ 0, 0, 0, /**/ 1, 0, 0, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 1, 0, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 1, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 1, /**/ 0, 0, 0,
            // w
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 1, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 1, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 1;
    for (int i = 0; i < smodel.rows(); ++i) {
        for (int j = 0; j < smodel.cols(); ++j) {
            H[i][j] = smodel(i, j);
        }
    }
    /*
    for (int i = 0; i < Mobs; ++i) {
        for (int j = 0; j < Mobs; ++j) {
            H[i][j] = 1;
        }
    }*/

    std::cerr << "start process model" << std::endl;
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
    double w[3] = {x[10], x[11], x[12]};
    
    double* qnew = compute_q(q, w, dt);
    for (int i = 6; i < 10; ++i) {
        // q = q*qdot
        fx[i] = qnew[i - 6];
    }

    /*
    double q[4] = {x[6], x[7], x[8], x[9]};
    double w[3] = {x[10], x[11], x[12]};
    std::cerr << "computing qdot" << std::endl;
    double* qdot = q_dot(q, w);
    for (int i = 6; i < 10; ++i) {
        // q = q*qdot
        fx[i] = this->x[i] * qdot[i - 6] * dt;
    }
    std::cerr << "used qdot" << std::endl; */
    // TODO delete qdot here

    // w
    for (int i = 10; i < 13; ++i) {
        fx[i] = this->x[i];
    }
    std::cerr << "did w" << std::endl;

    // process model jacobian
    Eigen::MatrixXd pmodel(Nsta, Nsta);
    std::cerr << "create pmodel" << std::endl;
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
            // q TODO implement
            0, 0, 0, /**/ 0, 0, 0, /**/ 1, 0, 0, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 1, 0, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 1, 0, /**/ 0, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 1, /**/ 0, 0, 0,
            // w
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 1, 0, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 1, 0,
            0, 0, 0, /**/ 0, 0, 0, /**/ 0, 0, 0, 0, /**/ 0, 0, 1;
    for (int i = 0; i < pmodel.rows(); ++i) {
        for (int j = 0; j < pmodel.cols(); ++j) {
            F[i][j] = pmodel(i, j);
        }
    }
    debug(fx, F, hx, H);

}



