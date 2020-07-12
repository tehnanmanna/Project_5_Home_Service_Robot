#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

double robot_a, robot_b;
double DISTANCE_THRESHOLD = 0.05;
double PICKUP_A = 2.0, PICKUP_B = 6.5;
double DROPOFF_A = -1.0, DROPOFF_B = 0.0;
int WAIT_TIME = 5;
int current_wait = 0;

void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_amcl)
{
  // Update robot position
  robot_a = msg_amcl->pose.pose.position.x;
  robot_b = msg_amcl->pose.pose.position.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  int state = 0;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set marker orientation
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.z = 0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.pose.position.z = 0;

  marker.lifetime = ros::Duration();

  // Subscribe to /amcl_pose
  ros::Subscriber sub1 = n.subscribe("/amcl_pose", 1000, robotPoseCallback);

  while (ros::ok())
  {

    // State transitions

    if (state == 0)
    {
      // Calculate distance
      double pickup_distance = abs(robot_a - PICKUP_A) + abs(robot_b - PICKUP_B);

      if (pickup_distance > DISTANCE_THRESHOLD)
      {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = PICKUP_A;
        marker.pose.position.y = PICKUP_B;
      }
      else
      {
        state = 1;
        marker.action = visualization_msgs::Marker::DELETE;
      }
    }

    else if (state == 1)
    {
      if (current_wait < WAIT_TIME)
      {
        current_wait += 5;
      }
      else
      {
        state = 2;
      }
    }

    else if (state == 2)
    {
      // Calculate distance
      double dropoff_distance = abs(robot_a - DROPOFF_A) + abs(robot_b - DROPOFF_B);

      if (dropoff_distance > DISTANCE_THRESHOLD)
      {
        marker.action = visualization_msgs::Marker::DELETE;
      }
      else
      {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = DROPOFF_A;
        marker.pose.position.y = DROPOFF_B;
      }
    }

    // Publish the Marker
    marker_pub.publish(marker);

    // Sleep for 1 seconds
    sleep(1);

    // Handle ROS communication events
    ros::spinOnce();
  }
}
