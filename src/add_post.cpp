#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker gen_cube(geometry_msgs::Point center, bool vertical, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "post";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1.8;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    
    marker.pose.position = center;
    
    if (vertical)
    {
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    }
    else
    {
        marker.scale.z = 2.6;
        marker.pose.orientation.x = 0.70710678;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.70710678;
    }
    
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    
    marker.lifetime = ros::Duration();
    
    return marker;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_post");
    ros::NodeHandle n;
    ros::Rate r(0.5);
    ros::Publisher post_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    geometry_msgs::Point p1;
    p1.x = -4.5;
    p1.y = -1.3;
    p1.z = 0.9;
    visualization_msgs::Marker pl_1 = gen_cube(p1, true, 0);

    geometry_msgs::Point p2;
    p2.x = -4.5;
    p2.y = 0;
    p2.z = 1.8;
    visualization_msgs::Marker pl_2 = gen_cube(p2, false, 1);
    
    geometry_msgs::Point p3;
    p3.x = -4.5;
    p3.y = 1.3;
    p3.z = 0.9;
    visualization_msgs::Marker pl_3 = gen_cube(p3, true, 2);
    
    geometry_msgs::Point p4;
    p4.x = 4.5;
    p4.y = -1.3;
    p4.z = 0.9;
    visualization_msgs::Marker pr_1 = gen_cube(p4, true, 3);
    
    geometry_msgs::Point p5;
    p5.x = 4.5;
    p5.y = 0;
    p5.z = 1.8;
    visualization_msgs::Marker pr_2 = gen_cube(p5, false, 4);
    
    geometry_msgs::Point p6;
    p6.x = 4.5;
    p6.y = 1.3;
    p6.z = 0.9;
    visualization_msgs::Marker pr_3 = gen_cube(p6, true, 5);

    while (ros::ok())
    {
        post_pub.publish(pl_1);
        post_pub.publish(pl_2);
        post_pub.publish(pl_3);
        post_pub.publish(pr_1);
        post_pub.publish(pr_2);
        post_pub.publish(pr_3);
        r.sleep();
    }
}
