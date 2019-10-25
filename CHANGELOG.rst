^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* reset hector map after takeoff (tested in simulation)
* checking hector velocity
* hector reliable after switching estimator
* hector reliability tuning
* updated tracker_status topic to correct one
* fusing zero tilts on the ground
* odometry diag publishes availability of garmin (height_available)
* publishing height (detilted and filtered garmin range)
* publishing innovation
* fixed switching to non-active estimator
* fixed max_altitude = 0
* fixed covariance Q vs R bug
* fixed bug in correction
* updated estimator list for gps in simulation
* StateEstimator static Eigen matrices
* added publishing of pose to rtk_republisher
* fixed uninitialized variables
* hopefully fixed vslam jump bug
* vslam available in simulation
* VSLAM PoseStamped -> PoseWithCovarianceStamped
* vslam pose estimator
* 2nd rehaul of launchfiles
* rehauled launch files
* deleted almost all launchfiles
* fixed noise in velocity, preparing for vio in feedback
* fixed uninitialized variables
* fixed wrong hector corrections due to jumps in hector heading
* fixed measurement for sonar
* slow odom 1 hz
* in hector we trust less
* in hector we trust!
* faster disturbance integration
* Increased covariance of acceleration and velocity state
* sonar enabled
* increased covariance of sonar range
* remap ultrasound
* longer median fileter for sonar
* added missing parameters for simulation
* sonar added
* finished state spam removed
* removed terminal spam
* fixed utm origin initial coordinates
* zoh for hector pose
* running estimators can be now specified in config files
* utm_origin vs. local_origin is now decided based on takeoff estimator
* added missing hector pose remap
* brick estimator changes
* Work in progress on brick estimator
* Switching heading estimators now correctly rotates the lateral state
* Fixed a bug in mavros velocity calculation - RTK should work again
* added pixgarm if to odometry f550 launch file
* child_frame_id problem when switching heading estimator
* Contributors: Matej Petrlik, Matej Petrlik (desktop), Tomas Baca, UAV_44, Vojtech Spurny, uav43, uav5, uav61

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
