#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_axis");
    ros::NodeHandle n;
    ros::Rate r(0.1);
    ros::Publisher axis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 4);
    uint32_t shape = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker x_axis;
    visualization_msgs::Marker y_axis;
    x_axis.header.frame_id = y_axis.header.frame_id = "map";
    x_axis.header.stamp = y_axis.header.stamp = ros::Time::now();
    x_axis.ns = y_axis.ns = "arrows";
    x_axis.id = 0;
    x_axis.id = 1;
    x_axis.type = y_axis.type = shape;
    x_axis.action = y_axis.action = visualization_msgs::Marker::ADD;
    
    x_axis.pose.position.x = 0;
    x_axis.pose.position.y = 0;
    x_axis.pose.position.z = 0;
    
    y_axis.pose.position.x = 0;
    y_axis.pose.position.y = 0;
    y_axis.pose.position.z = 0;
    
    x_axis.pose.orientation.x = 0.0;
    x_axis.pose.orientation.y = 0.0;
    x_axis.pose.orientation.z = 0.0;
    x_axis.pose.orientation.w = 1.0;
    
    y_axis.pose.orientation.x = 0.0;
    y_axis.pose.orientation.y = 0.0;
    y_axis.pose.orientation.z = 0.70710678;
    y_axis.pose.orientation.w = 0.70710678;
    
    x_axis.scale.x = 4.5;
    x_axis.scale.y = 0.1;
    x_axis.scale.z = 0.1;
    
    y_axis.scale.x = 3.0;
    y_axis.scale.y = 0.1;
    y_axis.scale.z = 0.1;
    
    x_axis.color.r = 1.0f;
    x_axis.color.g = 0.0f;
    x_axis.color.b = 0.0f;
    x_axis.color.a = 1.0f;
    
    y_axis.color.r = 0.0f;
    y_axis.color.g = 1.0f;
    y_axis.color.b = 0.0f;
    y_axis.color.a = 1.0f;
    
    x_axis.lifetime = y_axis.lifetime = ros::Duration();
    
    while (ros::ok())
    {
        axis_pub.publish(x_axis);
        axis_pub.publish(y_axis);
        r.sleep();   
    }
}
