#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool moveToGoal(move_base_msgs::MoveBaseGoal goal, MoveBaseClient * ac){
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, goal reached!");
    return true;
  } else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
    return false;
  }    
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Server Reached");

  move_base_msgs::MoveBaseGoal goal;
  float x[2] = {1,-1};
  float y[2] = {1,-1};

  for (int goalId = 0; goalId < 2; goalId++){
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = x[goalId];
    goal.target_pose.pose.position.y = y[goalId];
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Goal Set");
    bool goalReached = moveToGoal(goal, &ac);
    if (!goalReached){
      return 0; // Goal couldn't be reached for some reason, end the program
    } else{
      ros::Duration(5.0).sleep(); // Goal reached, wait 5 seconds
    }
  }
  ROS_INFO("All goals reached!");
  return 0;
}