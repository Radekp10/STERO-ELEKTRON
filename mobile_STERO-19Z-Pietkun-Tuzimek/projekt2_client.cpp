#include "ros/ros.h"
#include "stero_mobile_init/Projekt2.h"
#include <cstdlib>
#include "geometry_msgs/Pose.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "projekt2_client");
  if (argc != 3)
  {
    ROS_INFO("usage: projekt2_client pose");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("projekt2");
  stero_mobile_init::projekt2 srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}