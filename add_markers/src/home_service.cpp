#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class home_service
{
private:
    //All the instances
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;
    visualization_msgs::Marker marker;
    double start_goal[2] = {4.0, 2.0}; //start goal (x,y)
    double end_goal[2] = {4.0, 7.0};  //end goal(x,y)
    
public:
    home_service()
    {
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        odom_sub = n.subscribe("/odom", &home_service::odomCallback, this);
        
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        //Initialized at the start goal position.
        marker.pose.position.x = start_goal[0]; 
        marker.pose.position.y = start_goal[1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);


    }
    
};



int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  
  ros::Rate r(1);
  
  bool done = false;


  while (ros::ok())
  {
  
     

    case 1:
      sleep(5);
      ROS_INFO_ONCE("Deleting object from pickup location");
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::DELETE;
      count += 1;
      break;
    
    case 2:
      sleep(5);
      ROS_INFO_ONCE("Object at dropoff");
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;  
      shape = visualization_msgs::Marker::CUBE;
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = 4.0;
      marker.pose.position.y = 7.0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      done = true;
      break;
    }  
  
  

  // Flag to see if the switch cases completed sucesfully.
  if(done)
  {
    sleep(3);
    return 0;
  }


    r.sleep();
  }
}