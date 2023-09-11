#ifndef RVIZ_VISUALIZATION_H
#define RVIZ_VISUALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class rviz_visualization
{
private:


public:
    ros::Publisher marker_pub, marker_publisher_;
    ros::Subscriber odometry_pose_sub;
    geometry_msgs::PoseStamped pose_msg;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array_;
    explicit rviz_visualization(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::Pose::ConstPtr& odometry_pose_msg);
    ~rviz_visualization();
};


#endif // RVIZ_VISUALIZATION_H