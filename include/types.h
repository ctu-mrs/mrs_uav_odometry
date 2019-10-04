#pragma once
#ifndef MRS_ODOMETRY_TYPES_H
#define MRS_ODOMETRY_TYPES_H

#include <Eigen/Eigen>

#define N_STATES 6
#define N_MEASUREMENTS 14
/* typedefs //{ */


typedef Eigen::Matrix<double, 1, 1> Mat1;
typedef Eigen::Matrix<double, N_STATES, N_STATES> LatMat;
typedef Eigen::Matrix<double, N_STATES, 1> LatState1D;
typedef Eigen::Matrix<double, 1, N_STATES> LatStateCol1D;
typedef Eigen::Matrix<double, N_STATES, 2> LatState2D;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, N_MEASUREMENTS, 1> LatVecMes;

//}

#endif
