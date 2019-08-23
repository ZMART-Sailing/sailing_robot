#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator_gps");
  ros::NodeHandle nh;

  ros::Publisher sim_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("position", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    sensor_msgs::NavSatFix sim_gps_msg;
    sim_gps_msg.altitude = 0.0;
    sim_gps_msg.latitude = 0.0;
    sim_gps_msg.longitude = 0.0;
    sim_gps_pub.publish(sim_gps_msg);
    loop_rate.sleep();
  }
  return 0;
}
