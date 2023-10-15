#include "odometry_calculater.h"

// Function to calculate front wheel steering angle based on steering wheel
// angle
double odometry_calculater::calculateFrontWheelAngle(
    double steeringWheelAngleDeg, double wheelbase, double trackWidth,
    double driveRatio) {
  // Convert the steering wheel angle to radians
  double steeringWheelAngleRad = steeringWheelAngleDeg * M_PI / 180.0;

  // Calculate the desired angle of the front wheels
  double desiredFrontWheelAngleRad =
      atan(wheelbase / (wheelbase + (trackWidth / 2.0)) *
           tan(steeringWheelAngleRad / driveRatio));

  // Convert the angle back to degrees
  double desiredFrontWheelAngleDeg = desiredFrontWheelAngleRad * DEG2RAD;

  return desiredFrontWheelAngleDeg;
}

void odometry_calculater::vehicleOdometryCallback(
    const vehicle_dynamic_msgs::DrivingParameters &msg) {
  const double currentTime = ros::Time::now().toSec();
  const double deltaTime = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;

  // Convert speed from km/h to m/s steer_angle_status
  const double speed_mps = msg.vehicle_speed_status * KMH2MS;

  // Implement bicycle model for position and orientation updates
  const double max_steer_wheel_angle =
      900.0;  // Maximum steering wheel angle in degrees
  steering_wheel_angle_deg =
      std::max(-max_steer_wheel_angle,
               std::min(msg.steer_angle_status, max_steer_wheel_angle));

  delta_angle =
      steering_wheel_angle_deg * (maximum_wheel_angle / max_steer_wheel_angle);

  const double position_dx = speed_mps * cos(yaw_angle) * deltaTime;
  const double position_dy = speed_mps * sin(yaw_angle) * deltaTime;
  yaw_angle += (speed_mps / wheel_base) * tan(delta_angle) * deltaTime;
  const double orientation_dx = cos(yaw_angle);
  const double orientation_dy = sin(yaw_angle);

  if (speed_mps != 0) {
    update(speed_mps, delta_angle, deltaTime);
  }

  ROS_WARN_STREAM("totalDistance(m): " << totalDistance);
}

void odometry_calculater::update(const double &vehicle_speed,
                                 const double &delta_angle,
                                 const double &deltaTime) {
  position_x += vehicle_speed * cos(yaw_angle) * deltaTime;
  position_y += vehicle_speed * sin(yaw_angle) * deltaTime;
  yaw_angle += (vehicle_speed / wheel_base) * tan(delta_angle) * deltaTime;

  position_rear_x = position_x - ((wheel_base / 2) * cos(yaw_angle));
  position_rear_y = position_y - ((wheel_base / 2) * sin(yaw_angle));

  currentPosition.position.x += position_rear_x;
  currentPosition.position.y += position_rear_y;

  currentPosition.orientation.x = cos(yaw_angle);
  currentPosition.orientation.y = sin(yaw_angle);

  totalDistance += sqrt(pow((position_rear_x - position_rear_x_old), 2) +
                        pow((position_rear_y - position_rear_y_old), 2));

  position_rear_x_old = position_rear_x;
  position_rear_y_old = position_rear_y;

  vehicle_odometry_pub.publish(currentPosition);
}

odometry_calculater::odometry_calculater(ros::NodeHandle &nh) {
  nh.param<double>("wheel_base", wheel_base, 4.8);
  nh.param<double>("wheel_track_width", wheel_track_width, 2.17);
  nh.param<double>("maximum_wheel_angle", maximum_wheel_angle, 40.0);
  nh.param<double>("drive_ratio", drive_ratio, 5.13);

  ROS_WARN_STREAM("wheel_base: " << wheel_base);
  ROS_WARN_STREAM("wheel_track_width: " << wheel_track_width);
  ROS_WARN_STREAM("maximum_wheel_angle: " << maximum_wheel_angle);
  ROS_WARN_STREAM("drive_ratio: " << drive_ratio);

  // Create a subscriber for speed and steering data
  odometry_update_sub =
      nh.subscribe("/driving_parameters", 10,
                   &odometry_calculater::vehicleOdometryCallback, this);
  vehicle_odometry_pub =
      nh.advertise<geometry_msgs::Pose>("vehicle_odometry", 10);
}

odometry_calculater::~odometry_calculater() {}
