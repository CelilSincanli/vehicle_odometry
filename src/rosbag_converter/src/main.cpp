#include <ros/ros.h>

#include "vehicle_dynamic_msgs/CAN_55.h" //55
#include "vehicle_dynamic_msgs/CAN_56.h" //56
#include "vehicle_dynamic_msgs/DrivingParameters.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Publisher vehicle_parameters_pub;

vehicle_dynamic_msgs::DrivingParameters vehicle_parameters;

void callback(const vehicle_dynamic_msgs::CAN_55::ConstPtr& steering_msg, const vehicle_dynamic_msgs::CAN_56::ConstPtr& speed_msg)
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

    vehicle_parameters_pub = nh.advertise<vehicle_dynamic_msgs::DrivingParameters>("driving_parameters", 1000);

    message_filters::Subscriber<vehicle_dynamic_msgs::CAN_55> steering_sub(nh, "/core_vehicle_interface/CAN_55/received_messages", 1);
    message_filters::Subscriber<vehicle_dynamic_msgs::CAN_56> speed_sub(nh, "/core_vehicle_interface/CAN_56/received_messages", 1);

    typedef message_filters::sync_policies::ApproximateTime<vehicle_dynamic_msgs::CAN_55, vehicle_dynamic_msgs::CAN_56> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), steering_sub, speed_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    
    return 0;
}