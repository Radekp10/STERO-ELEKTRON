#include "ros/ros.h"
#include "stero_mobile_init/Projekt2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <global_planner/planner_core.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdio>
#include <ros/node_handle.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include "costmap_2d/layered_costmap.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include <string>
#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>

using std::string;

using costmap_2d::Costmap2DROS;
using global_planner::GlobalPlanner;
using geometry_msgs::TransformStamped;
using geometry_msgs::PoseStamped;



ros::Publisher chatter_pub;

costmap_2d::Costmap2DROS *global_costmap;
costmap_2d::Costmap2DROS *local_costmap;

global_planner::GlobalPlanner globalPlanner;

geometry_msgs::PoseStamped start;



// FUNKCJA PORUSZAJACA ROBOTEM
bool move(float v1, float v2){
	
	ros::Rate loop_rate(10);

	ros::spinOnce();

    geometry_msgs::Twist msg;

	msg.linear.x=v1;
	msg.linear.y=0.0;
	msg.linear.z=0.0;

	msg.angular.x=0.0;
	msg.angular.y=0.0;
	msg.angular.z=v2;



    chatter_pub.publish(msg);

    ros::spinOnce();
    	
  return true;
}



// FUNKCJA ROBIACA WSZYSTKO
bool integration(stero_mobile_init::Projekt2::Request  &goal,
         stero_mobile_init::Projekt2::Response &res)
{
	move(0.2,0.0);	

	//geometry_msgs::PoseStamped start;

	std::vector<geometry_msgs::PoseStamped> plan;

	PoseStamped target;
	target.header.frame_id = "map";
    target.pose = goal.goal;

	//bool wasSuccessful = globalPlanner.makePlan(start, goal.goal, plan);
	globalPlanner.makePlan(start, target, plan);
	
    char xd[20];
    //std::sprintf(xd, "%d elementow",plan.size());
 
    //ROS_INFO(xd);  

	ROS_INFO("Osiagnieto cel.");  
  return true;
}




//MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "projekt2_server");
  ros::NodeHandle n;

  chatter_pub = n.advertise<geometry_msgs::Twist>("mux_vel_nav/cmd_vel", 1000);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // COSTMAP - GLOBALNA I LOKALNA 
  global_costmap= new costmap_2d::Costmap2DROS("global_costmap", tfBuffer);
  local_costmap= new costmap_2d::Costmap2DROS("local_costmap", tfBuffer);

  // PLANER - GLOBALNY I LOKALNY
  globalPlanner.initialize("base_global_planner", global_costmap);

  TransformStamped currentTf = tfBuffer.lookupTransform("base_link", "map", ros::Time::now(), ros::Duration(0.1));
  start.header.frame_id = "map";
  start.pose.position.x = currentTf.transform.translation.x;
  start.pose.position.y = currentTf.transform.translation.y;
  start.pose.position.z = currentTf.transform.translation.z;
  start.pose.orientation = currentTf.transform.rotation; // std bind

  ros::Rate loop_rate(10);
  loop_rate.sleep();

  ros::ServiceServer service = n.advertiseService("projekt2", integration);// std bind

  ROS_INFO("Server dziala!");
  ros::spin();

  return 0;
}