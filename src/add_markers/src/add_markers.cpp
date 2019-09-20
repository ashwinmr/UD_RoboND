#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
bool reached_1 = false;
bool reached_2 = false;
// Set goal positions
float pos_1_x = 0;
float pos_1_y = -1;
float pos_2_x = -1;
float pos_2_y = 0;
float wnd_size = 0.1;

// Callback to handle odom reading
void odomCallback(const nav_msgs::Odometry& msg)
{
  // Check if odometry is within 1st goal
  if(	(msg.pose.pose.position.x > pos_1_x - wnd_size )
     && (msg.pose.pose.position.x < pos_1_x + wnd_size )
     && (msg.pose.pose.position.y > pos_1_y - wnd_size )
     && (msg.pose.pose.position.y < pos_1_y + wnd_size )
     && (!reached_1)
     ){
    reached_1 = true;
    //Update marker
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    ROS_INFO("Reached goal 1");
    marker_pub.publish(marker);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  // Setup the marker
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();
  
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  
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
  
  // Show marker for first position
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.position.x = pos_1_x;
  marker.pose.position.y = pos_1_y;
  marker_pub.publish(marker);
  ROS_INFO("Marker ready");

  ros::spin();
}