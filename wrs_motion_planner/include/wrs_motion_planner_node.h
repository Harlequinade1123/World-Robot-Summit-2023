#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <algorithm>
#include "kinematics.h"

class MotionPlanner
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MotionPlanner();
    ~MotionPlanner();
    void init();
    void final();
    void publishInitState();
    void wheelOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void armPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void publishTimerCallback(const ros::TimerEvent& e);
    void publishState();
    void moveMecanumAbsolute(double *odom);
    void moveMecanumRelative(double *odom);
    void moveArmAbsolute(double *pose);
    void moveArmRelative(double *pose);
    void moveArmAngle(double *angles);
    void moveArmInvPos();
    void moveArmInitPos1();
    void moveArmInitPos2();
    void waitForGoal(long timeout_sec);
    void targetOdomToJoint();
    void targetPoseToJoint();
    double getWheelOdomX();
    double getWheelOdomY();
    double getArmPoseX();
    double getArmPoseY();
    double getArmPoseZ();
    double getArmBasePositionX();
    double getArmBasePositionY();
    double getArmBasePositionZ();
    void endEffectorOn();
    void endEffectorOff();
    void setTranslate(double translate, double max_trans)
    {
        MAX_ARM_TRANSLATE_ = translate;
        pose_error_[0] = max_trans;
        pose_error_[1] = max_trans;
        pose_error_[2] = max_trans;
    }

    private:
    ros::NodeHandle nh_;
    nav_msgs::Odometry wheel_odom_msg_;
    geometry_msgs::PoseStamped arm_pose_msg_;
    double arm_pose_saved_data_[3];
    double arm_angles_saved_data_[6];
    double arm_angles_saved_data_inv_[6];
    bool arm_saved_data_is_set;
    bool arm_angles_saved_data_is_set;
    ros::Publisher wheel_joint_pub_;
    ros::Publisher arm_joint_pub_;
    ros::Subscriber wheel_odom_sub_;
    ros::Subscriber arm_pose_sub_;
    ros::Timer publish_timer_;
    std::mutex wheel_mtx_;
    std::mutex arm_mtx_;
    double target_odom_[3]; // x y yaw
    double target_pose_[6]; // x y z r p y
    double odom_error_[3];
    double pose_error_[3];
    double angle_error_[6];
    double MAX_WHEEL_VEL_;
    double MAX_WHEEL_YAW_VEL_;
    double MAX_ARM_TRANSLATE_;
    double MAX_ARM_ANGLE_;
    double MAX_ARM_ROTATION_;
    Eigen::VectorXd init_arm_angles_vec_;
    Eigen::VectorXd init_arm_angles_vec_inv_;
    Eigen::VectorXd arm_angles_vec_;
    double target_wheel_joint_[4];
    double target_arm_joint_[6];
    double target_angle_[6];
    bool   end_effector_is_on_;
    int    end_effector_cnt_;

    bool mecanum_is_moving_;
    bool arm_is_moving_;
    bool arm_is_moving_angle_;
    bool wheel_is_subscribed_;
    bool arm_is_subscribed_;

    bool set_init;

    double arm_base_position_[3];

    Mecanum mecanum;
    CraneX7 craneX7;
};