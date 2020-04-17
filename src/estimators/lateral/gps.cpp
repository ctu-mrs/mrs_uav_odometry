/* includes //{ */

#include "estimators/lateral/gps.h"

//}

namespace mrs_odometry
{

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string uav_name) {

    nh_ = parent_nh;
    name_ = name;
    uav_name_ = uav_name;

    //TODO load parameters
    
    dt_ = 0.01;

    A_ << 
      1, 0, dt_, 0, dt^2/2, 0,
      0, 1, 0, dt_, 0, dt^2/2, 
      0, 0, 1, 0, dt_, 0, 
      0, 0, 0, 1, 0, dt; 
      0, 0, 0, 0, 1, 0, 
      0, 0, 0, 0, 0, 1; 

      B_ << 
        0, 0,
        0, 0,
        0, 0,
        0, 0,
        1, 0,
        0, 1;

      H_ <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;

        Q_ <<
          1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 1;

          R_ <<
            1, 0,
            0, 1;

    lkf_t lkf(A_, B_, H_);

  }

  bool start(void)                                                              = 0;
  bool pause(void)                                                              = 0;
  bool reset(void)                                                              = 0;

  std::string getName(void) = 0;

  State getState(const StateId &state_id_in) = 0;
  void  setState(const State &state_in)      = 0;

  States getStates(void)                    = 0;
  void   setStates(const States &states_in) = 0;

  void setInput(const Input &input_in)                                                           = 0;
  void setMeasurement(const Measurement &measurement_in, const MeasurementId_t &measurement_id_in) = 0;

  ProcessNoiseMatrix getProcessNoise()                                           = 0;
  void               setProcessNoise(const ProcessNoiseMatrix &process_noise_in) = 0;
  double             getMeasurementNoise(void)                                   = 0;
  void               setMeasurementNoise(double covariance)                      = 0;
};
}  // namespace mrs_odometry

#endif
