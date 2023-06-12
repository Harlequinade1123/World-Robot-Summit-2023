#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include "psketch.h"
#include "kinematics.h"

class Sketch : public PSketch
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher  wheel_odom_pub_;
    ros::Publisher  arm_pose_pub_;
    ros::Subscriber wheel_joint_sub_;
    ros::Subscriber arm_joint_sub_;
    std::mutex wheel_mtx_;
    std::mutex arm_mtx_;
    sensor_msgs::JointState    arm_joint_msg_;
    geometry_msgs::PoseStamped arm_pose_msg_;
    nav_msgs::Odometry odom_msg_;
    ros::Time callback_time_;

    ros::Time ros_now = ros::Time::now();
    ros::Time ros_old = ros::Time::now();
    
    Mecanum mecanum;
    bool dxl_is_connected = true;
    uint8_t ids[4]  = { 14, 11, 12, 13 };
    float   vals[4] = { 0.0, 0.0, 0.0, 0.0 };

    float robot_x = 0.0;
    float robot_y = 0.0;
    float robot_yaw = 0.0;
    
    float robot_vx = 0.0;
    float robot_vy = 0.0;
    float robot_w  = 0.0;

    float robot_tread  = 270;
    float robot_depth  = 250;
    float robot_height = 410;
    float wheel_radius = 50;
    float wheel_width  = 40;

    float arm_size = 30;
    float arm_lengths[8] = { 41.0, 64.0, 65.0, 185.0, 121.0, 129.0, 19.0, 24.0 };
    float arm_angles[8]  = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };

    float old_MOUSEX = 0;
    float old_MOUSEY = 0;

    float camera_angle    =   0.0;
    float camera_distance = 600.0;
    float camera_height   = 250.0;

    public:
    Sketch();
    void setup();
    void draw();
    void drawRobot();
    void drawArm();
    void keyEvent(int key, int action);
    void mouseButtonEvent(int button, int action);
    void cursorPosEvent(double xpos, double ypos);
    void scrollEvent(double xoffset, double yoffset);
    void wheelJointCallback(const sensor_msgs::JointStateConstPtr &msg);
    void armJointCallback(const sensor_msgs::JointStateConstPtr &msg);
    void parallelTask1();
    void parallelTask2();
    void parallelTask3();
};