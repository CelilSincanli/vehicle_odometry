#include "odometry_visualization/rviz_visualization.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_visualization");
    ros::NodeHandle nh("~");

    ros::Rate loop_rate(10);

    rviz_visualization visual(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}