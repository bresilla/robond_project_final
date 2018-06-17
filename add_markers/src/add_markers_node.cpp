#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

static const double THRESH  = 0.05;
static const double PICK_X  = -6.8;
static const double PICK_Y  = -3.1;
static const double DROP_X  = -6.8;
static const double DROP_Y  = 4.55;
static bool pickingUp       = false;
static ros::Publisher marker_pub;

void addMarker(double x, double y){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "marker";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}

void onAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  if(!pickingUp && abs(x - PICK_X) < THRESH && abs(y - PICK_Y) < THRESH){
    ROS_INFO("Target picked up ");
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "marker";
    marker.id = 0;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    pickingUp = true;
  }
  else if(pickingUp && abs(x - DROP_X) < THRESH && abs(y - DROP_Y) < THRESH){
    addMarker(DROP_X, DROP_Y);
    pickingUp = false;
    ROS_INFO("Target dropped off");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "add_marker");
  ros::NodeHandle n;

  ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, onAmclPose);
  marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){ return 0; }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    ros::Duration(1.0).sleep();
  }

  addMarker(PICK_X, PICK_Y);
  ros::spin();
  return 0;
}