#include "odometry_visualization/rviz_visualization.hpp"

rviz_visualization::rviz_visualization(ros::NodeHandle &nh) {
  // Create publishers for visualization
  marker_pub =
      nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  // Create a subscriber for receiving Pose messages
  odometry_pose_sub = nh.subscribe("/vehicle_odomoetry/vehicle_odometry", 10,
                                   &rviz_visualization::poseCallback, this);

  // Create a publisher for publishing markers as a MarkerArray
  // marker_publisher_ =
  // nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",
  // 10);

  // Create a new marker for the received pose
  marker.header.frame_id = "map";
  marker.ns = "odometry_points";
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.points.reserve(POINT_VECT_SIZE);
}

void rviz_visualization::poseCallback(
    const geometry_msgs::Pose::ConstPtr &odometry_pose_msg) {
  geometry_msgs::Point odometry_point;
  odometry_point.x = odometry_pose_msg->position.x;
  odometry_point.y = odometry_pose_msg->position.y;
  odometry_point.z = odometry_pose_msg->position.z;

  ROS_WARN_STREAM("odometry_point: " << odometry_point);

  static int point_count = 0;

  if (point_count < POINT_VECT_SIZE) {
    marker.points.push_back(odometry_point);
  } else {
    marker.points.at(point_count % POINT_VECT_SIZE) = odometry_point;
  }

  point_count += 1;

  marker_pub.publish(marker);
}

rviz_visualization::~rviz_visualization() {}