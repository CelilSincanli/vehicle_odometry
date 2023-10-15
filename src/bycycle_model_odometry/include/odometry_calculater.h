#ifndef ODOMETRY_CALCULATER_H
#define ODOMETRY_CALCULATER_H

#include <ros/ros.h>
#include <tf/tf.h>

#include "vehicle_dynamic_msgs/DrivingParameters.h"

// TODO : Get rid of magic numbers
#define KMH2MS 1000.0 / 3600.0
#define DEG2RAD M_PI / 180.0

class odometry_calculater {
 private:
  double totalDistance = 0.0;
  double lastUpdateTime = 0.0;
  double wheel_base = 0.0;
  double wheel_track_width = 0.0;
  double maximum_wheel_angle = 0.0;
  double drive_ratio = 0.0;
  double steering_wheel_angle_deg = 0.0;
  double delta_angle = 0.0;
  double yaw_angle = 0.0;
  double position_x = 0.0;
  double position_y = 0.0;
  double position_rear_x = 0.0;
  double position_rear_y = 0.0;
  double position_rear_x_old = 0.0;
  double position_rear_y_old = 0.0;

  ros::Publisher vehicle_odometry_pub;
  ros::Subscriber odometry_update_sub;

  geometry_msgs::Pose currentPosition;

 public:
  explicit odometry_calculater(ros::NodeHandle &nh);
  ~odometry_calculater();
  double calculateFrontWheelAngle(double steeringWheelAngleDeg,
                                  double wheelbase, double trackWidth,
                                  double driveRatio);

  void update(const double &vehicle_speed, const double &delta_angle,
              const double &deltaTime);
  void vehicleOdometryCallback(
      const vehicle_dynamic_msgs::DrivingParameters &msg);
};

#endif  // ODOMETRY_CALCULATER_H