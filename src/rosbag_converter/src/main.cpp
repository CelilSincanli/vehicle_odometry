#include <ros/ros.h>

#include "vehicle_dynamic_msgs/can_msg1.h"
#include "vehicle_dynamic_msgs/can_msg2.h"
#include "vehicle_dynamic_msgs/DrivingParameters.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Publisher vehicle_parameters_pub;

vehicle_dynamic_msgs::DrivingParameters vehicle_parameters;

void callback(const vehicle_dynamic_msgs::can_msg1::ConstPtr& steering_msg, const vehicle_dynamic_msgs::can_msg2::ConstPtr& speed_msg)
{
  vehicle_parameters.steer_angle_status     = steering_msg->steer_angle_status;
  vehicle_parameters.vehicle_speed_status   = speed_msg->vehicle_speed_status;
  vehicle_parameters.header.stamp = ros::Time::now();
  vehicle_parameters_pub.publish(vehicle_parameters);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_converter");
    ros::NodeHandle nh;

    vehicle_parameters_pub = nh.advertise<vehicle_dynamic_msgs::DrivingParameters>("driving_parametes", 1000);

    message_filters::Subscriber<vehicle_dynamic_msgs::can_msg1> steering_sub(nh, "/core_vehicle_interface/can_msg1/received_messages", 1);
    message_filters::Subscriber<vehicle_dynamic_msgs::can_msg2> speed_sub(nh, "/core_vehicle_interface/can_msg2/received_messages", 1);

    typedef message_filters::sync_policies::ApproximateTime<vehicle_dynamic_msgs::can_msg1, vehicle_dynamic_msgs::can_msg2> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), steering_sub, speed_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    
    return 0;
}