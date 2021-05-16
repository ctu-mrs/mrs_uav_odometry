^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_uav_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2021-05-16)
------------------
* version -> 1.0.1
* update liosam covariances after first flight testing
* update liosam covariances
* also allow to change liosam_origin to slam_origin
* updated ros::shutdown
* possibility to change aloam/liosam origin to slam_origin
* fixed rtk altitude intialization (tested on one rosbag only)
* fix in rtk saturate print
* fixed double -> bool variables
* updated initialization of gps_origin (update your world files!)
* added heading estimator to diagnostics msg
* increased repredictor buffer size, added AloamgarmDebug message publishing
* added remaining median filter config params for ALOAMGARM
* fixed Q matrix initilization in ALOAMGARM
* removed unnecessary if
* do not run repredictor without aloam data, fixes broken GPS flight
* throttled "longer than 10 ms prediction" warning
* aloamgarm update - separated aloam eigenvalue from odometry message, fixed dynamic reconfigure
* polishing aloamgarm
* removed commented out stuff from aloamgarm
* removed some garbage related to ALOAMGARM
* fixed aloam mapping tf
* modified aloamgarm duration debug publishing
* aloamgarm - added some debug publishers for processing duration and aloam_ok
* aloamgarm - added state for barometer bias
* increased toleration for aloamgarm median filter when close to the ground
* aloamgarm update
* aloamgarm - multiple values of difference for median filter
* trying ALOAM + garmin fusion
* passing zero input to estimators when attitude command is too old (to prevent repredictor warnings)
* disabled initialization of local odometry offset to current UAV height
* modified the rest of methods in HeadingEstimator to work with repredictor
* passing current time to prediction when no controller is active to prevent repredictor warnings
* added repredictor for StateEstimator and HeadingEstimator
* publishing ALOAM delay on a topic
* added repredictor reset
* altitude estimation working with repredictor (ALOAMREP estimator)
* repredictor for altitude estimation, it's broken
* trying repredictor for altitude estimation
* publishing debug topics, fixed switch to garmin
* aloamgarm - fusing garmin differentially
* trying aloam + garmin fusion for altitude estimation
* Contributors: Matej Petrlik, Pavel Petracek, Tomas Baca, Vaclav Pritzl

1.0.0 (2021-03-18)
------------------
* Major release

0.0.6 (2021-03-16)
------------------
* Major refactoring and overhaul
* c++ refactoring
* the use of AttitudeConverter
* heading-oriented estimation
* new estimators
* Noetic-compatible
* Contributors: Jan Bednar, Matej Petrlik, Pavel Petracek, Robert Penicka, Tomas Baca, Vaclav Pritzl

0.0.5 (2020-02-26)
------------------
* updated declinging invalid garmin measurement
* declining invladi garmin measurements
* service call routines, waiting for map reset before estimator reset
* checking if hector is running when switching to hector hdg
* reset hector service returns false when hector not running
* resetting acc state when reset hector
* Contributors: Matej Petrlik

0.0.4 (2020-02-18)
------------------
* shorter median filter
* plane cov 100->10 pls keep this value, higher breaks height estimator
* hiccup and failed servoing printing
* better mutexing of max_altitude, better diagnostic print
* Plane will not be set to unreliable ever
* not performing odometry switch if already active
* height estimate published in fcu_untilted frame
* fallback to optflow from gps
* lowered covariance of icp
* fixed taking off with plane estimator
* fixed brick heading deadlock
* updated dynamic reconfigure sequence
* removed covariances overrding default ones
* not fusing mavros velocity
* add icp2d to allowed state estimators for aloam
* gps, optflow, hector work reasonably well with se3 in simulation
* three states
* reset hector service
* diagonal matrices
* not switching estimators when already active
* triggering control update during brick reset only when brick in feedback
* simulation script switches worlds
* switching to garmin when plane height above 4 m
* add aloam_available to diagnostics msg
* fixed awiting for tf in callbackGramin, rmeove timeouts
* max heading brick jump
* hiccup detected is now a warning
* sanitized garmin callback
* better mutexing in mainTimer
* less uninitialized quaternion spam
* brick max jump
* checking nans and uninit quaternion in attitude cmd
* publishing imu in fcu untilted
* using attitude command from control manager instead of mavros target att
* detecting max brick jump
* saturating brick heading, slower fusion
* odom stable using transformer
* stable origin tf
* moved odom_stable transform after tf is publshed
* plane source for brick odometry
* separated local_origin and stable_origin
* brick timeout 0.5 s
* rinfo uppercase lateral estimator
* local_origin should work as intended wven with more active estimators
* fixed deadlock when switching to non-active state estimator
* correct baro offset when on the ground
* height in odom_gps is from the current alt estimator
* local_origin init
* fixed garmin remap
* added default value to GARMIN_FILTERED
* local_origin should start at correct height
* Change garmin input accoring to GARMIN_FILTERED
* moved spam to ros_debug
* local_origin is where the odometry node is started
* initialization of rtk estimator
* baro altitude estimator
* brick_origin uses plane altitude estimator
* service callback are now disabled by default
* Add ALOAM as lateral, heading and height estimator
* added service for toggling service callbacks
* odom_aux now published in main_timer
* cleaned up mainTimer
* improved intialization of local_origin frame
* Add altitude Q to config
* removed altitude_world
* added -Wall to check forgotten return types
* brick timeout
* max height for brick and plane height estimators
* brick altitude switching
* added brick_timeout to config
* updated garmin median filter
* saturating garmin corrections when toggled from off to on
* tuning altitude estimation (not tested with real UAV)
* tuned simulation covariances, publishing mavros odom
* Add covariances of aloam to dynamic reconfigure
* updated t650 mass in launch files
* mrs_rviz_interface in simulation.sh
* added world file resolving code to launch file
* added WORLD_NAME parameter to launch file
* Added missing dynamic reconfigure parameters
* swapped order of checks with garmin
* fixed untilted frame
* preparing for optflow optimized for low altitude
* Add tested version of AltitudeEstimator::ALOAM
* fixed untilted frame
* Add aloam_available to config
* latlon definition of local origin
* add aloam slam as new estimator
* unified parameters common to uav and simulation into one config
* fixed wrong brick height preventing postion fusion
* [TFConnectorDummy]: trees should be connected through GPS origin and not local origin
* [TFConnectorDummy]: added trycatch to lookuptransform to avoid crashes
* brickflow altitude
* added tf_connector_dummy for trivial connecting of TF trees between UAVs
* brickflow implemented, needs tuning
* prediction step triggered by main timer
* added utm tf publisher
* altitude estimator switch bugfix
* height estimator when brick becomes unreliable
* changed brick reliability check
* fixing height when brick unreliable
* fixed flying below 0 height with vio
* fixed wrong frame of gps_local_odom
* fix orientation in odom_stable
* constant prediction rate
* fixing BRICKFLOW estimator
* increased aux publisher rate
* no predictions when brick unreliable
* no more nans in tfs
* fixed for publishing local origin tf
* fixed checking name of estimator
* fcu tf moves again
* fixed altitude in aux estimators
* hopefully pass_rtk_as_odom works now
* fixed tf when using ground truth
* untilted is not unheadinged anymore
* publishing fcu_untilted transform
* fixed origin of GPS and RTK
* fixed pass_rtk_as_odom
* fixed gps origin
* added missing [Odometry] to ROS prints
* fixed measurement rotating bug
* brick and vio altitude estimators
* plane height estimator
* using correct function for fusing tilts
* fallback from BRICK cannot be BRICK
* removed disambiguate brick heading
* fixed inverse of tranform bug
* new reference frames
* fixed heading in odom_aux
* Updated VIO covariances
* fixed bug in angle unwrapping
* optflow in body frame
* rtk_local_odom now contains altitude above takeoff position
* publishing uav_state msg
* icp heading estimator in hector config
* changed namespace from local_origin to uavX/local_origin
* disable odometry callbacks before calling hover (safer)
* udpated hector map reset routine (hover + disable_callbacks)
* added missing parameters to uav config
* brick unreliable when detections stop coming
* updated brick topic
* change drone frame to uav_name/fcu
* implemented resetting routine for hector
* icp estimation runing
* New estimator based on ICP velocities
* Contributors: Jan Bednar, Matej Petrlik, Matej Petrlik (desktop), Matouš Vrba, Pavel Petracek, Pavel Petráček, Petr Stepan, Tomas Baca, Vit Kratky, Vojtech Spurny

0.0.3 (2019-10-25)
------------------
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
* Fixed sign of target heading body rate from mavros
* Printing disturbance force values to terminal
* Simplified configs
* Improved tilt fusion, disturbance acceleration
* ICP median filter
* Contributors: Matej Petrlik, Matej Petrlik (desktop), Matěj Petrlík, NAKI, Tomas Baca, Tomáš Báča, Vojtech Spurny, uav3, uav42, uav5, uav60

0.0.1 (2019-05-20)
------------------
