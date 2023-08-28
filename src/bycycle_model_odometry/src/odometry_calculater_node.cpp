#include <ros/ros.h>
#include <tf/tf.h>
#include "converter_messages/driving_parameters.h"

double totalDistance = 0.0;
double lastUpdateTime = 0.0;
double wheel_base;
double wheel_track_width;
double maximum_wheel_angle;
double steering_wheel_angle_deg;
double steering_angle;
double drive_ratio;
geometry_msgs::Pose currentPosition;

ros::Publisher vehicle_odometry_pub;

// Function to calculate front wheel steering angle based on steering wheel angle
double calculateFrontWheelAngle(double steeringWheelAngleDeg, double wheelbase, double trackWidth, double driveRatio)
{
    // Convert the steering wheel angle to radians
    double steeringWheelAngleRad = steeringWheelAngleDeg * M_PI / 180.0;

    // Calculate the desired angle of the front wheels
    double desiredFrontWheelAngleRad = atan(wheelbase / (wheelbase + (trackWidth / 2.0)) * tan(steeringWheelAngleRad / driveRatio));

    // Convert the angle back to degrees
    double desiredFrontWheelAngleDeg = desiredFrontWheelAngleRad * 180.0 / M_PI;

    return desiredFrontWheelAngleDeg;
}

void vehicleOdometryCallback(const converter_messages::driving_parameters& msg)
{
    double currentTime = ros::Time::now().toSec();
    double deltaTime = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    // Convert speed from km/h to m/s steer_angle_status
    double speed_mps = msg.vehicle_speed_status * 1000.0 / 3600.0;

    // Implement bicycle model for position and orientation updates
    double maxSteeringWheelAngleDeg = 900.0; // Maximum steering wheel angle in degrees
    steering_wheel_angle_deg = std::max(-maxSteeringWheelAngleDeg, std::min(msg.steer_angle_status, maxSteeringWheelAngleDeg));

    steering_angle = calculateFrontWheelAngle(steering_wheel_angle_deg, wheel_base, wheel_track_width, drive_ratio);

    double heading_dtheta = speed_mps * tan(steering_angle) / wheel_base;
    double position_dx = speed_mps * cos(heading_dtheta) * deltaTime;
    double position_dy = speed_mps * sin(heading_dtheta) * deltaTime;
    double orientation_dx = cos(heading_dtheta);
    double orientation_dy = sin(heading_dtheta);

    if (speed_mps != 0 )
    {
       totalDistance += sqrt(pow(((currentPosition.position.x + position_dx) - currentPosition.position.x),2) + pow(((currentPosition.position.y + position_dy) - currentPosition.position.y),2));
    }
    
    currentPosition.position.x += position_dx;
    currentPosition.position.y += position_dy;

    currentPosition.orientation.x = orientation_dx;
    currentPosition.orientation.y = orientation_dy;

    // ROS_WARN_STREAM(yaw);
    ROS_WARN_STREAM("currentPosition.position.x: " << currentPosition.position.x);
    ROS_WARN_STREAM("currentPosition.position.y: " << currentPosition.position.y);
    ROS_WARN_STREAM("totalDistance: " << totalDistance);
    vehicle_odometry_pub.publish(currentPosition);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_odomoetry");
    ros::NodeHandle nh("~");
    
    nh.getParam("wheel_base", wheel_base);
    nh.getParam("wheel_track_width", wheel_track_width);
    nh.getParam("maximum_wheel_angle", maximum_wheel_angle);
    nh.getParam("drive_ratio", drive_ratio);
    ROS_WARN_STREAM(wheel_base);
    // Create a subscriber for speed and steering data
    ros::Subscriber odometry_update_sub = nh.subscribe("/driving_parametes", 10, vehicleOdometryCallback);
    vehicle_odometry_pub = nh.advertise<geometry_msgs::Pose>("vehicle_odometry",10);
    ros::spin();
}