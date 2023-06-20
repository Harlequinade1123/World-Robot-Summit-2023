#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "kinematics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_motion_planner_node");
    ros::NodeHandle nh;
    ros::Publisher wheel_joint_pub = nh.advertise<sensor_msgs::JointState>("/wrs/wheel/write", 10);
    ros::Publisher arm_joint_pub   = nh.advertise<sensor_msgs::JointState>("/wrs/arm/write", 10);

    sensor_msgs::JointState wheel_joint_msg;
    wheel_joint_msg.velocity.resize(4);

    sensor_msgs::JointState arm_joint_msg;
    arm_joint_msg.position.resize(6);

    Mecanum mecanum(50, 250, 270);
    mecanum.setYawAngle(0);
    float omega0 = 0, omega1 = 0, omega2 = 0, omega3 = 0;

    float arm_lengths[8] = { 41.0, 64.0, 65.0, 185.0, 121.0, 129.0, 19.0, 24.0 };
    float arm_angles[8]  = { 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 };
    CraneX7 craneX7(arm_lengths, 7, 6);
    Eigen::VectorXd q(6);
    Eigen::Vector3d pos_vec;
    Eigen::Matrix3d ori_mat;
    q << 0.0, 1.0, 0.0, 1.0, 0.0, 1.0;
    craneX7.calcForward(q);
    double x, y, z;
    craneX7.getXYZ(x, y, z);
    pos_vec << x, y, z;
    craneX7.getR(ori_mat);
    craneX7.calcInvese(q, pos_vec, ori_mat);
    std::cout << q.transpose() << std::endl;
    ros::Rate rate(100);
    while (ros::ok())
    {
        /**
        craneX7.calcInvese(q, pos_vec, ori_mat);
        std::cout << q.transpose() << std::endl;
        arm_joint_msg.header.stamp = ros::Time::now();
        arm_joint_msg.position[0] = q(0);
        arm_joint_msg.position[1] = q(1);
        arm_joint_msg.position[2] = q(2);
        arm_joint_msg.position[3] = q(3);
        arm_joint_msg.position[4] = q(4);
        arm_joint_msg.position[5] = q(5);
        arm_joint_pub.publish(arm_joint_msg);
        **/

        //std::cout << "publish" << std::endl;
        //printf("%lf %lf %lf\n", pos_vec(0), pos_vec(1), pos_vec(2));
        //std::cout << ori_mat << std::endl;
        
        //std::cout << q.transpose() << std::endl;
        for (int i = 0; i < 50; i++)
        {
            pos_vec(1) -= 1;
            craneX7.calcInvese(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 100; i++)
        {
            pos_vec(0) -= 1;
            craneX7.calcInvese(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 100; i++)
        {
            pos_vec(1) += 1;
            craneX7.calcInvese(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 100; i++)
        {
            pos_vec(0) += 1;
            craneX7.calcInvese(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 50; i++)
        {
            pos_vec(1) -= 1;
            craneX7.calcInvese(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        /**
        mecanum.calcInvese(100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInvese(0, 100, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInvese(-100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInvese(0, -100, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        **/
        rate.sleep();
    }
}