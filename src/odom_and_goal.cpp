#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_and_goal");
    ros::NodeHandle nh;

    ros::Publisher chatter_current_odom = nh.advertise<nav_msgs::Odometry>("current_odom", 1000);
    ros::Publisher chatter_goal_point = nh.advertise<geometry_msgs::PoseStamped>("goal_point", 1000);
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg;
        nav_msgs::Odometry current_odom;
        geometry_msgs::PoseStamped goal_point;

        /*
        current_odom.pose.pose.position.x = 0;
        current_odom.pose.pose.position.y = 0;
        goal_point.pose.position.x = 5;
        goal_point.pose.position.y = 15;
        */

        current_odom.pose.pose.position.x = 0;
        current_odom.pose.pose.position.y = 0;
        goal_point.pose.position.x = 5;
        goal_point.pose.position.y = 15;

        chatter_current_odom.publish(current_odom);
        chatter_goal_point.publish(goal_point);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
