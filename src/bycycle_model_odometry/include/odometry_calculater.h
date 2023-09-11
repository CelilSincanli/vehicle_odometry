#ifndef ODOMETRY_CALCULATER_H
#define ODOMETRY_CALCULATER_H

#include <ros/ros.h>
#include <tf/tf.h>

#include "vehicle_dynamic_msgs/DrivingParameters.h"

class odometry_calculater
{
private:
    
    double totalDistance = 0.0;
    double lastUpdateTime = 0.0;
    double wheel_base;
    double wheel_track_width;
    double maximum_wheel_angle;
    double drive_ratio;
    double steering_wheel_angle_deg;
    double steering_angle;
    
    ros::Publisher vehicle_odometry_pub;
    ros::Subscriber odometry_update_sub; 

    geometry_msgs::Pose currentPosition;
    
public:
    

    explicit odometry_calculater(ros::NodeHandle &nh);
    ~odometry_calculater();
    double calculateFrontWheelAngle(double steeringWheelAngleDeg, double wheelbase, double trackWidth, double driveRatio);
    void vehicleOdometryCallback(const vehicle_dynamic_msgs::DrivingParameters& msg);

};



#endif // ODOMETRY_CALCULATER_H