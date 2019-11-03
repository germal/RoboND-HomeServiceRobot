#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float pickup_pose[3]   = {-1, 3.4, 1};
float drop_off_pose[3] = {-4, -3, 1};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  tf2::Quaternion quat_tf;
  move_base_msgs::MoveBaseGoal pickup_goal, drop_off_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup_goal.target_pose.pose.position.x = pickup_pose[0];
  pickup_goal.target_pose.pose.position.y = pickup_pose[1];
  quat_tf.setRPY( 0, 0, pickup_pose[2] );
  tf2::convert(quat_tf, pickup_goal.target_pose.pose.orientation);
   

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup_goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_WARN("The base failed to reach the pickup location for some reason");
    return -1;
  }

  ROS_INFO("The base reached the pickup location waiting 5 sec to load the package.");
  ros::Duration(5.0).sleep(); // sleep for 5 seconds


  // set up the frame parameters
  drop_off_goal.target_pose.header.frame_id = "map";
  drop_off_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  drop_off_goal.target_pose.pose.position.x = drop_off_pose[0];
  drop_off_goal.target_pose.pose.position.y = drop_off_pose[1];  
  quat_tf.setRPY( 0, 0, drop_off_pose[2] );
  tf2::convert(quat_tf, drop_off_goal.target_pose.pose.orientation);
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(drop_off_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_WARN("The base failed to reach drop off goal location for some reason");
    return -1;
  }
  ROS_INFO("Package succesfully dropped at the drop off goal ");

  return 0;
}
