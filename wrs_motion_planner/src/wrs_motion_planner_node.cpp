#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "kinematics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_mecanum_node");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/wrs/write", 10);

    sensor_msgs::JointState joint_msg;
    joint_msg.velocity.resize(4);

    Mecanum mecanum(50, 250, 270);
    mecanum.setYawAngle(0);
    float omega0 = 0, omega1 = 0, omega2 = 0, omega3 = 0;

    while (ros::ok())
    {
        mecanum.calcInvese(100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.velocity[0] = omega0;
        joint_msg.velocity[1] = omega1;
        joint_msg.velocity[2] = omega2;
        joint_msg.velocity[3] = omega3;
        joint_pub.publish(joint_msg);
        usleep(1000000);
        mecanum.calcInvese(-100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.velocity[0] = omega0;
        joint_msg.velocity[1] = omega1;
        joint_msg.velocity[2] = omega2;
        joint_msg.velocity[3] = omega3;
        joint_pub.publish(joint_msg);
        usleep(1000000);
    }
}