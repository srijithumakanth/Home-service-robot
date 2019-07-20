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
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        odom_sub = n.subscribe("/odom", 1, &home_service::odomCallback, this);
        
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

        //Odometry callback function definition -->
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            //Robot position
            float x_rob = msg->pose.pose.position.x;
            float y_rob = msg->pose.pose.position.y;
            
            //Marker scale values
            float scale_x = marker.scale.x;
            float scale_y = marker.scale.y;

            //Flags to check if robot reached the goal position
            bool start_reached = false;
            bool end_reached = false;

            //Logic to switch between the markers when robot reaches the goals.
            if ((x_rob < (start_goal[0] + scale_x)) && (x_rob > (start_goal[0] - scale_x)) && (y_rob < (start_goal[1] + scale_y)) && (y_rob > (start_goal[1] - scale_y)))
            {
                start_reached = true;
                ROS_INFO_ONCE("Start goal reached!");
                ROS_INFO_ONCE("Simulating Pickup!");
                //ros::Duration(5).sleep();

                marker.action = visualization_msgs::Marker::DELETE;
                ROS_INFO_ONCE("Deleting object");
                ros::Duration(5).sleep();
                marker_pub.publish(marker);
            }
            else if ((x_rob < (end_goal[0] + scale_x)) && (x_rob > (end_goal[0] - scale_x)) && (y_rob < (end_goal[1] + scale_y)) && (y_rob > (end_goal[1] - scale_y)))
            {
                end_reached = true;
                ROS_INFO_ONCE("End goal reached!");
                ROS_INFO_ONCE("Simulating Dropoff!");
                ros::Duration(5).sleep();

                marker.pose.position.x = end_goal[0];
                marker.pose.position.y = end_goal[1];
                marker.action = visualization_msgs::Marker::ADD;

                marker_pub.publish(marker);
            }
            

    }
    
};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "home_service");
  home_service home_service;
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  
  return 0;
}