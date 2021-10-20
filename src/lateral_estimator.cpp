#include "lateral_estimator.h"

// | ---------------------- state machine --------------------- |

/* //{ changeState() */

void LateralEstimator::changeState(SMStates_t new_state) {

  previous_sm_state_ = current_sm_state_;
  current_sm_state_  = new_state;

  switch (current_sm_state_) {
    case UNINITIALIZED_STATE:
      break;
    case INITIALIZED_STATE:
      break;
    case READY_STATE:
      break;
    case RUNNING_STATE:
      break;
    case STOPPED_STATE:
      break;
  }

  // TODO use correct name of estimator
  ROS_INFO("[%s]: Switching sm state %s -> %s", getName().c_str(), sm_state_names[previous_sm_state_], sm_state_names[current_sm_state_]);
}

//}
