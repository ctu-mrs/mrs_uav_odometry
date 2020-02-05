#ifndef GPS_H_
#define GPS_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <lateral_estimator.h>

//}

#define N_STATES 6
#define N_INPUTS 1
#define N_MEASUREMENTS 6

namespace mrs_odometry
{

class Gps : public LateralEstimator {

  /* typedef //{ */

  typedef enum
  {

    POSITION,
    VELOCITY,
    ACCELERATION,
    ACCELERATION_INPUT,
    ACCELERATION_DISTURBANCE,
    TILT_MAVROS

  } StateId_t;

  //}

public:
  static const int n = N_STATES;
  static const int m = N_INPUTS;
  static const int p = N_MEASUREMENTS;

  typedef Eigen::Matrix<double, n, n> A_t;
  typedef Eigen::Matrix<double, n, m> B_t;
  typedef Eigen::Matrix<double, p, n> H_t;
  typedef Eigen::Matrix<double, n, p> K_t;
  typedef Eigen::Matrix<double, n, n> R_t;

  typedef Eigen::Matrix<double, 2, 1> State_t;
  typedef Eigen::Matrix<double, 2, n> States_t;
  typedef Eigen::Matrix<double, 2, 1> Input_t;
  typedef Eigen::Matrix<double, n, n> ProcessNoiseMatrix_t;
  typedef double                      Measurement_t;

public:
  virtual ~Gps(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string uav_name) const override;
  virtual bool start(void) const override;
  virtual bool pause(void) const override;
  virtual bool reset(void) const override;

  virtual std::string getName(void) const override;

  virtual State_t getState(const StateId_t &state_id_in) const override;
  virtual void    setState(const State_t &state_in) const override;

  virtual States_t getStates(void) const override;
  virtual void     setStates(const States_t &states_in) const override;

  virtual void setInput(const Input_t &input_in) const override;
  virtual void setMeasurement(const Measurement_t &measurement_in, const MeasurementId_t &measurement_id_in) const override;

  virtual ProcessNoiseMatrix_t getProcessNoise() const override;
  virtual void                 setProcessNoise(const ProcessNoiseMatrix_t &process_noise_in) const override;
  virtual double               getMeasurementNoise(void) const override;
  virtual void                 setMeasurementNoise(double covariance) const override;
};
}  // namespace mrs_odometry

#endif
