#pragma once
#ifndef MRS_ODOMETRY_TYPES_H
#define MRS_ODOMETRY_TYPES_H

#include <Eigen/Eigen>

#include <mrs_lib/lkf.h>

/* defines //{ */

// lateral
#define LAT_N_STATES 3
#define LAT_N_MEASUREMENTS 14

// altitude
#define ALT_N_STATES 3
#define ALT_N_INPUTS 1
#define ALT_N_MEASUREMENTS 1

/*//}*/

/* typedefs //{ */

typedef Eigen::Matrix<double, 1, 1> Mat1;
typedef Eigen::Matrix<double, 2, 1> Vec2;

typedef Eigen::Matrix<double, LAT_N_STATES, LAT_N_STATES> LatMat;
typedef Eigen::Matrix<double, LAT_N_STATES, 1> LatState1D;
typedef Eigen::Matrix<double, 1, LAT_N_STATES> LatStateCol1D;
typedef Eigen::Matrix<double, LAT_N_STATES, 2> LatState2D;

typedef mrs_lib::LKF<ALT_N_STATES, ALT_N_INPUTS, ALT_N_MEASUREMENTS> lkf_alt_t;
typedef lkf_alt_t::statecov_t alt_statecov_t;
typedef lkf_alt_t::x_t alt_x_t; 
typedef lkf_alt_t::u_t alt_u_t; 
typedef lkf_alt_t::z_t alt_z_t; 
typedef lkf_alt_t::A_t alt_A_t; 
typedef lkf_alt_t::B_t alt_B_t; 
typedef lkf_alt_t::H_t alt_H_t; 
typedef lkf_alt_t::R_t alt_R_t; 
typedef lkf_alt_t::Q_t alt_Q_t; 
typedef lkf_alt_t::P_t alt_P_t; 

//}


#endif
