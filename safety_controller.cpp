/*
 * Author Harry Terkanian
 * July 16, 2017
 *
 * ported from Python November 29, 2017
 *
 * most recent revision November 29, 2017
 *
 * safety controller.
 *
 * monitor the distance to objects in front of the car; if 
 * approaching minimum safe distance slow down, if at 
 * minimum safe distance stop.
 *
 * minimum safe distance is set by safe_distance
 *
 * at 3x minumum safe distance speed reduced from safe_speed to 0.0 
 * linearly as the minumum distance approaches the minimum safe distance
 * at minimum safe distance speed is set to 0.0
 *
 * possible enhancements: (1) change the arc being scanned for minimum distance 
 * based on the  current minimum distance since the car is 30 cm wide, 
 * (don't want to have a wheel clip an edge. . .);
 * and (2) monitor the steering angle to see whether to look one side of center 
 * as the car is turning. 
 */

#include <math.h>
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

bool  debug {true};
const float safe_distance {0.10};
const float lidar_offset {0.15};
float min_safe_distance {safe_distance + lidar_offset};
const float min_safe_distance_multiplier { 4.0};
float safety_dist {min_safe_distance_multiplier * min_safe_distance};
const float safe_speed {0.20};
float odom_velocity {0.0};

bool  slowdown_flag {false};
bool  first_scan {true};

int   begin_scan_arc {0};
int   end_scan_arc {0};

float min_distance {0.0};
const float PI = 3.14159265358979f;
float half_safety_arc {PI / 6.0f};

ackermann_msgs::AckermannDriveStamped   safety_msg;
nav_msgs::Odometry                      odom_msg;
sensor_msgs::LaserScan                  scan_msg;
ros::Publisher  cmd_pub;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_velocity = sqrt( msg->twist.twist.linear.x *
            msg->twist.twist.linear.x +
            msg->twist.twist.linear.y *
            msg->twist.twist.linear.y);
}


void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (first_scan)     //calculate some basic info on first scan
	{
	    first_scan = false;
        int scan_range_array_length = (int) (msg->angle_max - msg->angle_min) / msg->angle_increment;
        int scan_range_array_center =  scan_range_array_length / 2;
        int sixth_arc = (int) (half_safety_arc / msg->angle_increment);
        begin_scan_arc = scan_range_array_center - sixth_arc;
        end_scan_arc = scan_range_array_center + sixth_arc;
        if (debug)
        {
//            ROS_INFO("array length: [%d]; array center: [%d]; sixth arc: [%d]; begin arc: [%d]", 
//                    scan_range_array_length, 
//                    scan_range_array_center, 
//                    sixth_arc, 
//                    begin_scan_arc); 
        }
	}

    for (int i = begin_scan_arc; i < end_scan_arc + 1; ++i)
    {
        if (i == begin_scan_arc)
        {
            min_distance = msg->ranges[i];
        }
        else
        {
            min_distance = (min_distance < msg->ranges[i]) 
                    ? min_distance 
                    : msg->ranges[i];
        }
    }

    if (min_distance < safety_dist)
    {
        safety_msg.drive.speed = (odom_velocity *
                (min_safe_distance / safety_dist));
        slowdown_flag = true;
    }
    if (min_distance < min_safe_distance)
    {
        safety_msg.drive.speed = 0.0;
        slowdown_flag = true;
    }
    if (slowdown_flag)
    {
        slowdown_flag = false;
        if (odom_velocity > safety_msg.drive.speed)
        {
            cmd_pub.publish(safety_msg);
        }
    }
    if (debug)
    {
        ROS_INFO("Min distance: [%.2f]; velocity [%.2f]; safety speed: [%.2f]", 
                min_distance,
                odom_velocity,
                safety_msg.drive.speed);
    }

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "safety_controller");
    ros::NodeHandle n;
    ros::Subscriber scan_sub = n.subscribe("scan", 100, scan_callback);
    ros::Subscriber odom_sub = n.subscribe("odom", 100, odom_callback);
    cmd_pub  = 
            n.advertise<ackermann_msgs::AckermannDriveStamped>
            ("ackermann_cmd_mux/input/safety", 100);

	ros::spin();

	return 0;
}
