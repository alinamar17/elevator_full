#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <stdio.h> 
#include <stdlib.h>  
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double distance=-1;
double new_distance=-2;
void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
int size = msg->ranges.size();
double distFront=msg->ranges[size/2];
if(new_distance==-2){
distance=distFront;
ROS_INFO("Distanse is %f",distFront);
}
else
new_distance=distFront;
}


int main(int argc, char **argv){
ros::init(argc, argv, "distance_measure_subscriber");
ros::NodeHandle node_obj;
ros::Subscriber distance_measure_subscribe=node_obj.subscribe("/scan",10,messageCallback);
while(distance==-1)
ros::spinOnce();
new_distance=-1;
ROS_INFO("first stage is over!");
while(new_distance==-1||abs(new_distance-distance)<1)
  ros::spinOnce();
ROS_INFO("elevetor is open!");
MoveBaseClient ac("move_base", true);
while(!ac.waitForServer(ros::Duration(5.0))){
 ROS_INFO("Waiting for the move_base action server to come up");
}
move_base_msgs::MoveBaseGoal goal;
goal.target_pose.header.frame_id = "base_link";
goal.target_pose.header.stamp = ros::Time::now();
goal.target_pose.pose.position.x = 1.0;
goal.target_pose.pose.orientation.w = 1.0;
ROS_INFO("Sending goal");
ac.sendGoal(goal);
ac.waitForResult();
if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("Hooray, the base moved 1 meter forward");
else
  ROS_INFO("The base failed to move forward 1 meter for some reason");

return 0;
}
