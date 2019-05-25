#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 10);//+
  std_msgs::String msg;//+
  std::string s;//+

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal firstgoal;

  // set up the frame parameters
  firstgoal.target_pose.header.frame_id = "map";
  firstgoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  firstgoal.target_pose.pose.position.x = 0;
  firstgoal.target_pose.pose.position.y = -2;
  firstgoal.target_pose.pose.position.z = 0;

  firstgoal.target_pose.pose.orientation.x = 0;
  firstgoal.target_pose.pose.orientation.y = 0;
  firstgoal.target_pose.pose.orientation.z = 0;
  firstgoal.target_pose.pose.orientation.w = 1;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending firstgoal");
  ac.sendGoal(firstgoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    s="pickup";//+
    msg.data = s;//+
    chatter_pub.publish(msg);//+
    ROS_INFO("Hooray, the robot reached to the pickup location, 5 seconds to go");
    ros::Duration(5.0).sleep();
  }
  else
  {
    ROS_INFO("The robot failed to reach to the pickup location, exiting");
    ros::Duration(5.0).sleep();
    return 0;
  }

  
  move_base_msgs::MoveBaseGoal secondgoal;

  // set up the frame parameters
  secondgoal.target_pose.header.frame_id = "map";
  secondgoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  secondgoal.target_pose.pose.position.x = 2;
  secondgoal.target_pose.pose.position.y = -5;
  secondgoal.target_pose.pose.position.z = 0;

  secondgoal.target_pose.pose.orientation.x = 0;
  secondgoal.target_pose.pose.orientation.y = 0;
  secondgoal.target_pose.pose.orientation.z = 0;
  secondgoal.target_pose.pose.orientation.w = 1;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending secondgoal");
  ac.sendGoal(secondgoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    s="drop";//+
    msg.data = s;//+
    chatter_pub.publish(msg);//+
    ROS_INFO("SUCCESS, the robot reached to the drop off location, exiting");
    //ros::Duration(5.0).sleep();
  }
  else
  {
    ROS_INFO("The robot failed to reach to the drop off location, exiting");
    //ros::Duration(5.0).sleep();
  }
  ros::Duration(15.0).sleep();
  return 0;
}
