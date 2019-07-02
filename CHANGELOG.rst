^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2019-07-01)
------------------
* Switching heading estimator rotates lateral states
* Fixed max altitude
* Moved support functions to separate file
* + Brickflow estimator
* + Hector estimator
* Separate process covariance for optflow launch file
* Slower disturbance acceleration integration
* updated max optflow height
* Detecting VIO failures
* Tuned lateral GPS pos, vel covariances
* Calling failsafe when no fallback odometry available
* + monitor script
* changed rinfo frequency of disturbance force
* Fixed sign of target yaw body rate from mavros
* Printing disturbance force values to terminal
* Simplified configs
* Improved tilt fusion, disturbance acceleration
* ICP median filter
* Contributors: Matej Petrlik, Matej Petrlik (desktop), Matěj Petrlík, NAKI, Tomas Baca, Tomáš Báča, Vojtech Spurny, uav3, uav42, uav5, uav60

0.0.1 (2019-05-20)
------------------
