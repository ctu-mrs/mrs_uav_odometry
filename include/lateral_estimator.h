#ifndef LATERALESTIMATOR_H_
#define LATERALESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <lateral_measurement.h>

//}

namespace mrs_odometry
{

typedef enum
{

  UNINITIALIZED_STATE,
  INITIALIZED_STATE,
  READY_STATE,
  RUNNING_STATE,
  STOPPED_STATE,
  ERROR_STATE

} SMStates_t;

template <typename State, typename StateId, typename States, typename Input, typename Measurement, typename MeasurementId, typename ProcessNoiseMatrix>
class LateralEstimator {

public:
  virtual ~LateralEstimator(void) {
  }

private:
  SMStates_t previous_sm_state_;
  SMStates_t current_sm_state_;

public:
  virtual void initialize(const ros::NodeHandle &parent_nh, std::string name, const std::string uav_name) = 0;
  virtual bool start(void)                                                                                = 0;
  virtual bool pause(void)                                                                                = 0;
  virtual bool reset(void)                                                                                = 0;

  virtual std::string getName(void) = 0;

  virtual State getState(const StateId &state_id_in) = 0;
  virtual void  setState(const State &state_in)      = 0;

  virtual States getStates(void)                    = 0;
  virtual void   setStates(const States &states_in) = 0;

  virtual void setInput(const Input &input_in)                   = 0;
  virtual void setMeasurement(const Measurement &measurement_in) = 0;

  virtual ProcessNoiseMatrix getProcessNoise()                                           = 0;
  virtual void               setProcessNoise(const ProcessNoiseMatrix &process_noise_in) = 0;

  virtual double getMeasurementNoise(void)              = 0;
  virtual void   setMeasurementNoise(double covariance) = 0;

  // implemented
  void changeState(SMStates_t new_state);
};
}  // namespace mrs_odometry

#endif
