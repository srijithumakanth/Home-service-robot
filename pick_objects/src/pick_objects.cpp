#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1;
  move_base_msgs::MoveBaseGoal goal2;

  // set up the frame parameters for goal 1
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  
  // set up the frame parameters for goal 2
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();

  // Define first position and orientation for the robot to reach
  goal1.target_pose.pose.position.x = 4.0;
  goal1.target_pose.pose.position.y = 2.0;
  goal1.target_pose.pose.orientation.w = 1.0;

  // Define second position and orientation for the robot to reach
  goal2.target_pose.pose.position.x = 4.0;
  goal2.target_pose.pose.position.y = 7.0;
  goal2.target_pose.pose.orientation.w = 1.0;

  // Send the first goal position and orientation for the robot to reach
  ROS_INFO("Sending goal 1");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, I reached zone 1, give me more!!");
  
  //Wait for 5 seconds before the second goal.
    sleep(5);
  }
  else
  {
    ROS_INFO("I'm sorry, I couldn't make it!");
    return 0;
  }
    
  
  // Send the second goal position and orientation for the robot to reach
  ROS_INFO("Sending goal 2");
  ac.sendGoal(goal2);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, I reached zone 2, give me more!!");
  else
  {
    ROS_INFO("I'm sorry, I couldn't make it!");
  }
    

  sleep(5);
  return 0;
  
}