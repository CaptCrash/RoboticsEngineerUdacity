#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>

float x = 0.8; //where are we trying to go?
float y = 0.8; //where are we trying to go?
int goalReached = 0; //Flag if we reached the current goal
float EPSILON = 0.25; //Within "a package" of the package

void posCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//Check if we're at the target
ROS_INFO("Pose recieved! x: %f, y:%f",msg->pose.pose.position.x,msg->pose.pose.position.y);
if (pow(msg->pose.pose.position.x-x,2) + pow(msg->pose.pose.position.y-y,2)<=pow(EPSILON,2)){
  //if target is reached, do something
  goalReached = 1;
  ROS_INFO_ONCE("Goal Reached");
}
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = n.subscribe("amcl_pose", 1, posCallback); //Only look at the most recent message
  bool finalProject;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("final",finalProject,bool(true));

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  float visible = 1.0;
  bool carrying = false;
  bool notDelivered = true;

  while (ros::ok() & notDelivered)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = visible;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO_ONCE("Publishing Marker!");
    marker_pub.publish(marker);
    if (goalReached > 0){
      if (!carrying){
        ROS_INFO("Package picked up!");
        visible = 0.0f;
        carrying = true;
      } else{
        sleep(5); //Simulate "Dropping" the package off, similar to pick_objects.cpp
        visible = 1.0f;
        marker.color.a = visible;
        ROS_INFO("Package delivered!");
        marker_pub.publish(marker);
        notDelivered = false;
        sleep(5);
      }
      x = -1;
      y = -1;
      goalReached = false;
    }
    if (finalProject){
      ROS_INFO_ONCE("Performing callbacks");
      ros::spinOnce(); //check our callbacks
    } else{
      ROS_INFO("CYCLE MODE");
      sleep(5); // We just need to go through the goals
      goalReached = true;
    }
    r.sleep();
  }
}