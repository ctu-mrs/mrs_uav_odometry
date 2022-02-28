#pragma once
#ifndef MRS_UAV_ODOMETRY_TYPES_H
#define MRS_UAV_ODOMETRY_TYPES_H

#include <Eigen/Eigen>

#include <mrs_lib/lkf.h>
#include <mrs_lib/jlkf.h>
#include <mrs_lib/repredictor.h>

/* defines //{ */

// trackers

#define NULL_TRACKER "NullTracker"
#define LANDOFF_TRACKER "LandoffTracker"

// lateral
#define LAT_N_STATES 3
#define LAT_M_INPUTS 1
#define LAT_P_MEASUREMENTS 1

// altitude
#define ALT_N_STATES 3
#define ALT_M_INPUTS 1
#define ALT_P_MEASUREMENTS 1

// heading
#define HDG_N_STATES 3
#define HDG_M_INPUTS 2
#define HDG_P_MEASUREMENTS 1

/*//}*/

/* typedefs //{ */

typedef Eigen::Matrix<double, 1, 1> Mat1;
typedef Eigen::Matrix<double, 2, 1> Vec2;

typedef Eigen::Matrix<double, LAT_N_STATES, LAT_N_STATES> LatMat;
typedef Eigen::Matrix<double, LAT_N_STATES, 1> LatState1D;
typedef Eigen::Matrix<double, 1, LAT_N_STATES> LatStateCol1D;
typedef Eigen::Matrix<double, LAT_N_STATES, 2> LatState2D;

// lateral
typedef mrs_lib::LKF<LAT_N_STATES, LAT_M_INPUTS, LAT_P_MEASUREMENTS> lkf_lat_t;
typedef lkf_lat_t::statecov_t lat_statecov_t;
typedef lkf_lat_t::x_t lat_x_t; 
typedef lkf_lat_t::u_t lat_u_t; 
typedef lkf_lat_t::z_t lat_z_t; 
typedef lkf_lat_t::A_t lat_A_t; 
typedef lkf_lat_t::B_t lat_B_t; 
typedef lkf_lat_t::H_t lat_H_t; 
typedef lkf_lat_t::R_t lat_R_t; 
typedef lkf_lat_t::Q_t lat_Q_t; 
typedef lkf_lat_t::P_t lat_P_t; 

// altitude
typedef mrs_lib::LKF<ALT_N_STATES, ALT_M_INPUTS, ALT_P_MEASUREMENTS> lkf_alt_t;
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

// altitude - repredictor
typedef mrs_lib::varstepLKF<ALT_N_STATES, ALT_M_INPUTS, ALT_P_MEASUREMENTS> var_lkf_alt_t;
typedef var_lkf_alt_t::statecov_t var_alt_statecov_t;
typedef var_lkf_alt_t::x_t var_alt_x_t; 
typedef var_lkf_alt_t::u_t var_alt_u_t; 
typedef var_lkf_alt_t::z_t var_alt_z_t; 
typedef var_lkf_alt_t::A_t var_alt_A_t; 
typedef var_lkf_alt_t::B_t var_alt_B_t; 
typedef var_lkf_alt_t::H_t var_alt_H_t; 
typedef var_lkf_alt_t::R_t var_alt_R_t; 
typedef var_lkf_alt_t::Q_t var_alt_Q_t; 
typedef var_lkf_alt_t::P_t var_alt_P_t; 

// altitude - ALOAMGARM
typedef mrs_lib::JLKF<6, 1, 1, 3> algarm_alt_t;
typedef algarm_alt_t::statecov_t algarm_alt_statecov_t;
typedef algarm_alt_t::x_t algarm_alt_x_t; 
typedef algarm_alt_t::u_t algarm_alt_u_t; 
typedef algarm_alt_t::z_t algarm_alt_z_t; 
typedef algarm_alt_t::A_t algarm_alt_A_t; 
typedef algarm_alt_t::B_t algarm_alt_B_t; 
typedef algarm_alt_t::H_t algarm_alt_H_t; 
typedef algarm_alt_t::R_t algarm_alt_R_t; 
typedef algarm_alt_t::Q_t algarm_alt_Q_t; 
typedef algarm_alt_t::P_t algarm_alt_P_t; 

// heading
typedef mrs_lib::LKF<HDG_N_STATES, HDG_M_INPUTS, HDG_P_MEASUREMENTS> lkf_hdg_t;
typedef lkf_hdg_t::statecov_t hdg_statecov_t;
typedef lkf_hdg_t::x_t hdg_x_t; 
typedef lkf_hdg_t::u_t hdg_u_t; 
typedef lkf_hdg_t::z_t hdg_z_t; 
typedef lkf_hdg_t::A_t hdg_A_t; 
typedef lkf_hdg_t::B_t hdg_B_t; 
typedef lkf_hdg_t::H_t hdg_H_t; 
typedef lkf_hdg_t::R_t hdg_R_t; 
typedef lkf_hdg_t::Q_t hdg_Q_t; 
typedef lkf_hdg_t::P_t hdg_P_t; 

// heading - variable LKF for repredictor
typedef mrs_lib::varstepLKF<HDG_N_STATES, HDG_M_INPUTS, HDG_P_MEASUREMENTS> var_lkf_hdg_t;
typedef lkf_hdg_t::statecov_t var_hdg_statecov_t;
typedef lkf_hdg_t::x_t var_hdg_x_t; 
typedef lkf_hdg_t::u_t var_hdg_u_t; 
typedef lkf_hdg_t::z_t var_hdg_z_t; 
typedef lkf_hdg_t::A_t var_hdg_A_t; 
typedef lkf_hdg_t::B_t var_hdg_B_t; 
typedef lkf_hdg_t::H_t var_hdg_H_t; 
typedef lkf_hdg_t::R_t var_hdg_R_t; 
typedef lkf_hdg_t::Q_t var_hdg_Q_t; 
typedef lkf_hdg_t::P_t var_hdg_P_t; 

typedef mrs_lib::Repredictor<var_lkf_alt_t> rep_t;
typedef mrs_lib::Repredictor<mrs_lib::LKF_MRS_odom> rep_lat_t;
typedef mrs_lib::Repredictor<var_lkf_hdg_t> rep_hdg_t;
typedef mrs_lib::Repredictor<algarm_alt_t> algarm_rep_t;

//}

#endif
