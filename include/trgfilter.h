#ifndef TRG_FILTER
#define TRG_FILTER

/* author: Daniel Hert */

#include <vector>
#include <ros/ros.h>

/**
 * @brief The TrgFilter class
 */
class TrgFilter {

public:

TrgFilter(int buffer_size, double car_height, bool suppress, double trg_max_valid_altitude, double trg_filter_max_difference);
double getValue(double input, double kalman_altitude, ros::Duration interval);

private:

double checkForCarSimple(double input, double kalman_altitude);
bool isValid(double input, ros::Duration interval);
std::vector<double> buffer;
int buffer_size;
int next;
bool above_car;
double car_height;
double trg_max_valid_altitude;
double trg_filter_max_difference;
bool suppress;
};


#endif

