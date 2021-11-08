#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

//}

namespace mrs_odometry
{

template <typename State, typename StateId, typename States, typename Input, typename Measurement, typename MeasurementId, typename ProcessNoiseMatrix>
class Estimator {

public:
  virtual ~Estimator(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh) = 0;
  virtual bool start(void)                                                              = 0;
  virtual bool pause(void)                                                              = 0;
  virtual bool reset(void)                                                              = 0;

  virtual std::string getName(void) = 0;

  virtual State getState(const StateId &state_id_in) = 0;
  virtual void  setState(const State &state_in)      = 0;

  virtual States getStates(void)                    = 0;
  virtual void   setStates(const States &states_in) = 0;

  /* virtual void setInput(const Input &input_in)                                                           = 0; */
  /* virtual void setMeasurement(const Measurement &measurement_in, const MeasurementId &measurement_id_in) = 0; */

  /* virtual ProcessNoiseMatrix getProcessNoise()                                           = 0; */
  /* virtual void               setProcessNoise(const ProcessNoiseMatrix &process_noise_in) = 0; */
  /* virtual double             getMeasurementNoise(void)                                   = 0; */
  /* virtual void               setMeasurementNoise(double covariance)                      = 0; */
};
}  // namespace mrs_odometry

#endif
