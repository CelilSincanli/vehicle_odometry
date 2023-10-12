
#include "odometry_calculater.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vehicle_odomoetry",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  odometry_calculater vehicle_odometry(nh);

  ros::spin();
}