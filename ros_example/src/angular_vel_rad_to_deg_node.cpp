#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <tf/transform_datatypes.h>

ros::Publisher angular_vel_publisher;
ros::Subscriber quat_subscriber;

const float r2d = 57.29577951f;

void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

    geometry_msgs::Vector3 angular_vel;
    angular_vel.x = msg->angular_velocity.x*r2d;
    angular_vel.y = msg->angular_velocity.y*r2d;
    angular_vel.z = msg->angular_velocity.z*r2d;

    angular_vel_publisher.publish(angular_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener");
    ros::NodeHandle n;
    angular_vel_publisher = n.advertise<geometry_msgs::Vector3>("angular_vel_deg", 1000);
    quat_subscriber = n.subscribe("/imu/data", 1000, MsgCallback);

    ROS_INFO("waiting for imu data");
    ros::spin();
    return 0;
}
