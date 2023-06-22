#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include "kinematics.h"

class MotionPlanner
{
    public:
    MotionPlanner();
    ~MotionPlanner();
    void setMecanumTarget();
    void setArmTarget();
    void wheelOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void armPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void armPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void publishtTimerCallback(const ros::TimerEvent& e);
    void moveMecanumAbsolute();
    void moveMecanumRelative();
    void moveArmAbsolute();
    void moveArmRelative();
    void moveMecanumAndArmAbsolute();
    void moveMecanumAndArmRelative();

    private:
    ros::NodeHandle nh_;
    nav_msgs::Odometry wheel_odom_msg_;
    geometry_msgs::PoseStamped arm_pose_msg_;
    ros::Publisher wheel_joint_pub_;
    ros::Publisher arm_joint_pub_;
    ros::Subscriber wheel_odom_sub_;
    ros::Subscriber arm_pose_sub_;
    std::mutex wheel_mtx_;
    std::mutex arm_mtx_;
    double target_odom_[3];
    double target_pose_[6];
    double target_wheel_joint_[4];
    double target_arm_joint_[6];
    bool   end_effector_is_on_;
    int    end_effector_cnt_;

    Mecanum mecanum;
    CraneX7 craneX7;
};

MotionPlanner::MotionPlanner()
{
    mecanum = Mecanum(50, 250, 270);
    mecanum.setYawAngle(0);
    float end_effector_length_ = 24.0;
    float arm_lengths[7] = { 41.0, 64.0, 65.0, 185.0, 121.0, 129.0, 19.0 + end_effector_length_ };
    craneX7 = CraneX7(arm_lengths, 7, 6);

    end_effector_is_on_             = false;
    end_effector_cnt_               = 0;
    ros::Subscriber wheel_odom_sub_ = nh_.subscribe("/wrs/wheel/odom", 10, &MotionPlanner::wheelOdomCallback, this);
    ros::Subscriber arm_pose_sub_   = nh_.subscribe("/wrs/arm/pose", 10, &MotionPlanner::armPoseCallback, this);
    ros::Publisher wheel_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/wrs/wheel/write", 10);
    ros::Publisher arm_joint_pub_   = nh_.advertise<sensor_msgs::JointState>("/wrs/arm/write", 10);
    ros::Timer publish_timer_       = nh_.createTimer(ros::Duration(0.05), &MotionPlanner::publishtTimerCallback, this);
}

void MotionPlanner::wheelOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    wheel_mtx_.lock();
    if (is_simulation_ || wheel_odom_msg_.header.stamp < msg->header.stamp)
    {
        wheel_odom_msg_.header.stamp = msg->header.stamp;
        wheel_odom_msg_.pose         = msg->pose;
        wheel_odom_msg_.twist        = msg->twist;
    }
    wheel_mtx_.unlock();
}

void MotionPlanner::armPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    arm_mtx_.lock();
    if (is_simulation_ || arm_pose_msg_.header.stamp < msg->header.stamp)
    {
        arm_pose_msg_.header.stamp = msg->header.stamp;
        arm_pose_msg_.pose         = msg->pose;
    }
    arm_mtx_.unlock();
}

void publishtTimerCallback(const ros::TimerEvent& e)
{
    sensor_msgs::JointState wheel_joint_msg;
    sensor_msgs::JointState arm_joint_msg;
    wheel_joint_msg.velocity.resize(4);
    arm_joint_msg.position.resize(6);
    arm_joint_msg.velocity.resize(1);
    wheel_mtx_.lock();
    for (int joint_i = 0; joint_i < 6; joint_i++)
    {
        wheel_joint_msg.velocity[joint_i] = target_wheel_joint_[joint_i];
    }
    wheel_joint_pub_.publish();
    wheel_mtx_.unlock();
    arm_mtx_.lock();
    for (int joint_i = 0; joint_i < 6; joint_i++)
    {
        wheel_joint_msg.position[joint_i] = target_arm_joint_[joint_i];
    }
    if (end_effector_is_on_)
    {
        end_effector_cnt_++
        end_effector_cnt_ /= 100;
        if (end_effector_cnt_ < 50)
        {
            wheel_joint_msg.velocity[0] = 10;
        }
        else
        {
            wheel_joint_msg.velocity[0] = -10;
        }
    }
    else
    {
        wheel_joint_msg.velocity[0] = 0;
    }
    arm_joint_pub_.publish();
    arm_mtx_.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_motion_planner_node");
    MotionPlanner motion_planner;
    motion_planner.init();
    
    motion_planner.final();
}


/**
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
    arm_joint_msg.velocity.resize(1);

    Mecanum mecanum(50, 250, 270);
    mecanum.setYawAngle(0);
    float omega0 = 0, omega1 = 0, omega2 = 0, omega3 = 0;

    float end_effector_length_ = 24.0;
    float arm_lengths[7] = { 41.0, 64.0, 65.0, 185.0, 121.0, 129.0, 19.0 + end_effector_length_ };
    float arm_angles[6]  = { 0.0, -1.0, 0.0, -1.0, 0.0, -1.0 };
    CraneX7 craneX7(arm_lengths, 7, 6);
    Eigen::VectorXd q(6);
    Eigen::Vector3d pos_vec;
    Eigen::Matrix3d ori_mat;
    q << 0.0, -1.0, 0.0, -1.0, 0.0, -1.0;
    ori_mat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, -1, 0));
    craneX7.calcForward(q);
    double x, y, z;
    craneX7.getXYZ(x, y, z);
    pos_vec << x, y, z;
    //craneX7.getR(ori_mat);
    craneX7.calcInverse(q, pos_vec, ori_mat);
    ros::Rate rate(100);
    while (ros::ok())
    {
        /**
        craneX7.calcInverse(q, pos_vec, ori_mat);
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
        /**
        for (int i = 0; i < 50; i++)
        {
            pos_vec(1) -= 1;
            ori_mat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, -1, 0));
            craneX7.calcInverse(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_msg.velocity[0] = 10.0;
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 100; i++)
        {
            pos_vec(0) -= 1;
            if (i < 50)
            {
                ori_mat = Eigen::AngleAxisd(M_PI + M_PI_2 / 50 * (i + 1), Eigen::Vector3d(0, -1, 0));
            }
            else
            {
                ori_mat = Eigen::AngleAxisd(M_PI + M_PI_2, Eigen::Vector3d(0, -1, 0));
            }
            craneX7.calcInverse(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_msg.velocity[0] = -10.0;
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 100; i++)
        {
            pos_vec(1) += 1;
            if (i < 50)
            {
                ori_mat = Eigen::AngleAxisd(M_PI + M_PI_2 - M_PI_2 / 50 * (i + 1), Eigen::Vector3d(0, -1, 0));
            }
            else
            {
                ori_mat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, -1, 0));
            }
            craneX7.calcInverse(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_msg.velocity[0] = 10.0;
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 100; i++)
        {
            pos_vec(0) += 1;
            if (i < 50)
            {
                ori_mat = Eigen::AngleAxisd(M_PI + M_PI_2 / 50 * (i + 1), Eigen::Vector3d(0, -1, 0));
            }
            else
            {
                ori_mat = Eigen::AngleAxisd(M_PI + M_PI_2, Eigen::Vector3d(0, -1, 0));
            }
            craneX7.calcInverse(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_msg.velocity[0] = -10.0;
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        for (int i = 0; i < 50; i++)
        {
            pos_vec(1) -= 1;
            ori_mat = Eigen::AngleAxisd(M_PI + M_PI_2 - M_PI_2 / 50 * (i + 1), Eigen::Vector3d(0, -1, 0));
            craneX7.calcInverse(q, pos_vec, ori_mat);
            arm_joint_msg.header.stamp = ros::Time::now();
            arm_joint_msg.position[0] = q(0);
            arm_joint_msg.position[1] = q(1);
            arm_joint_msg.position[2] = q(2);
            arm_joint_msg.position[3] = q(3);
            arm_joint_msg.position[4] = q(4);
            arm_joint_msg.position[5] = q(5);
            arm_joint_msg.velocity[0] = 10.0;
            arm_joint_pub.publish(arm_joint_msg);
            usleep(25000);
        }
        
        mecanum.calcInverse(100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInverse(0, 0, M_PI_2 / 3);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInverse(100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInverse(0, 0, -M_PI_2 / 3);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInverse(-100, 0, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        mecanum.calcInverse(0, -100, 0);
        mecanum.getRAD(omega0, omega1, omega2, omega3);
        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = omega0;
        wheel_joint_msg.velocity[1] = omega1;
        wheel_joint_msg.velocity[2] = omega2;
        wheel_joint_msg.velocity[3] = omega3;
        wheel_joint_pub.publish(wheel_joint_msg);
        usleep(3000000);
        

        wheel_joint_msg.header.stamp = ros::Time::now();
        wheel_joint_msg.velocity[0] = 0;
        wheel_joint_msg.velocity[1] = 0;
        wheel_joint_msg.velocity[2] = 0;
        wheel_joint_msg.velocity[3] = 0;
        wheel_joint_pub.publish(wheel_joint_msg);
    }
    wheel_joint_msg.header.stamp = ros::Time::now();
    wheel_joint_msg.velocity[0] = 0;
    wheel_joint_msg.velocity[1] = 0;
    wheel_joint_msg.velocity[2] = 0;
    wheel_joint_msg.velocity[3] = 0;
    wheel_joint_pub.publish(wheel_joint_msg);
}
**/