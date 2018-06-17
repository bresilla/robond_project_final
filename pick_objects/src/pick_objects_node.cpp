#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

static const double PICK_X  = -6.8;
static const double PICK_Y  = -3.1;
static const double DROP_X  = -6.8;
static const double DROP_Y  = 4.55;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_objects");
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // PICK
  goal.target_pose.pose.position.x = PICK_X;
  goal.target_pose.pose.position.y = PICK_Y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("sending TARGET pick-up coordinates...");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Pick-up reached!");
    ROS_INFO("Wait few seconds to load...");
    ros::Duration(5.0).sleep();

    // DROP
    goal.target_pose.pose.position.x = DROP_X;
    goal.target_pose.pose.position.y = DROP_Y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending TARGET drop-off coordinates...");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Drop-off reached!");
    }
    else {
      ROS_INFO("Drop-off failed to reach!");
    }
  }
  else {
    ROS_INFO("Robot did not make it!");
  }
  return 0;
}
