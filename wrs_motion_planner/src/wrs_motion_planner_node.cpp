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

MotionPlanner::MotionPlanner()
{
    MAX_WHEEL_VEL_     = 100;
    MAX_WHEEL_YAW_VEL_ = M_PI_2 / 2.5;
    MAX_ARM_TRANSLATE_ = 1;
    MAX_ARM_ANGLE_ = M_PI / 36;
    MAX_ARM_ROTATION_  = M_PI / 100;
    mecanum = Mecanum(50, 250, 270);
    mecanum.setYawAngle(0);
    float robot_tread  = 270;
    float robot_depth  = 250;
    float robot_height = 410;
    float wheel_radius = 50;
    arm_base_position_[0] = 180.0;
    arm_base_position_[1] = 0;
    arm_base_position_[2] = 320.0;
    //float end_effector_length_ = 24.0;
    float end_effector_length_ = 150.0;
    float arm_lengths[7] = { 41.0, 64.0, 65.0, 185.0, 121.0, 129.0, 19.0 + end_effector_length_ };
    craneX7 = CraneX7(arm_lengths, 7, 6);
    init_arm_angles_vec_ = Eigen::VectorXd(6);
    init_arm_angles_vec_ << 0.0, -M_PI / 12, 0.0, -5 * M_PI / 6, 0.0, -M_PI / 12;
    init_arm_angles_vec_inv_ = Eigen::VectorXd(6);
    init_arm_angles_vec_inv_ << 0.0, -M_PI / 12, 0.0, -5 * M_PI / 6, 0.0, -M_PI / 12;
    arm_angles_vec_ = Eigen::VectorXd(6);
    arm_angles_vec_ << 0.0, -M_PI / 12, 0.0, -5 * M_PI / 6, 0.0, -M_PI / 12;
    craneX7.calcForward(arm_angles_vec_);
    target_wheel_joint_[0] = 0.0;
    target_wheel_joint_[1] = 0.0;
    target_wheel_joint_[2] = 0.0;
    target_wheel_joint_[3] = 0.0;
    target_arm_joint_[0]   = 0.0;
    target_arm_joint_[1]   =-1.0;
    target_arm_joint_[2]   = 0.0;
    target_arm_joint_[3]   =-1.0;
    target_arm_joint_[4]   = 0.0;
    target_arm_joint_[5]   =-1.0;

    odom_error_[0] = 5;
    odom_error_[1] = 5;
    odom_error_[2] = M_PI / 72;
    pose_error_[0] = 2;
    pose_error_[1] = 2;
    pose_error_[2] = 2;
    angle_error_[0] = M_PI / 72;
    angle_error_[1] = M_PI / 72;
    angle_error_[2] = M_PI / 72;
    angle_error_[3] = M_PI / 72;
    angle_error_[4] = M_PI / 72;
    angle_error_[5] = M_PI / 72;

    end_effector_is_on_ = false;
    end_effector_cnt_   = 0;
    
    mecanum_is_moving_ = false;
    arm_is_moving_     = false;
    arm_is_moving_angle_= false;

    wheel_is_subscribed_ = false;
    arm_is_subscribed_   = false;

    arm_saved_data_is_set = false;
    arm_angles_saved_data_is_set = false;

    set_init = true;
}

MotionPlanner::~MotionPlanner()
{}

void MotionPlanner::init()
{
    wheel_odom_sub_  = nh_.subscribe("/wrs/wheel/odom", 10, &MotionPlanner::wheelOdomCallback, this);
    arm_pose_sub_    = nh_.subscribe("/wrs/arm/pose", 10, &MotionPlanner::armPoseCallback, this);
    wheel_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/wrs/wheel/write", 10);
    arm_joint_pub_   = nh_.advertise<sensor_msgs::JointState>("/wrs/arm/write", 10);
    //publish_timer_   = nh_.createTimer(ros::Duration(0.05), &MotionPlanner::publishTimerCallback, this);
    this->publishInitState();
    ROS_INFO("START\n");
}

void MotionPlanner::final()
{
    this->publishInitState();
    ROS_INFO("END\n");
}

void MotionPlanner::publishInitState()
{
    ROS_INFO("3...\n");
    usleep(1000000);
    sensor_msgs::JointState wheel_joint_msg;
    sensor_msgs::JointState arm_joint_msg;
    wheel_joint_msg.velocity.resize(4);
    arm_joint_msg.position.resize(6);
    arm_joint_msg.velocity.resize(1);
    wheel_joint_msg.header.stamp = ros::Time::now();
    for (int joint_i = 0; joint_i < 4; joint_i++)
    {
        wheel_joint_msg.velocity[joint_i] = 0;
    }
    wheel_joint_pub_.publish(wheel_joint_msg);
    for (int joint_i = 0; joint_i < 6; joint_i++)
    {
        arm_joint_msg.position[joint_i] = init_arm_angles_vec_(joint_i);
        arm_angles_saved_data_[joint_i] = init_arm_angles_vec_(joint_i);
        arm_angles_saved_data_inv_[joint_i] = init_arm_angles_vec_(joint_i);
    }
    arm_joint_msg.velocity[0] = 0;
    if (set_init)
    {
        for (int joint_i = 0; joint_i < 6; joint_i++)
        {
            arm_joint_msg.position[joint_i] = 0;
            arm_angles_saved_data_[joint_i] = 0;
        }
    }
    arm_joint_pub_.publish(arm_joint_msg);
    ROS_INFO("2...\n");
    arm_joint_pub_.publish(arm_joint_msg);
    usleep(1000000);
    ROS_INFO("1...\n");
    arm_joint_pub_.publish(arm_joint_msg);
    usleep(1000000);
}

void MotionPlanner::wheelOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    wheel_mtx_.lock();
    if (wheel_odom_msg_.header.stamp < msg->header.stamp)
    {
        wheel_odom_msg_.header.stamp = msg->header.stamp;
        wheel_odom_msg_.pose         = msg->pose;
        wheel_odom_msg_.twist        = msg->twist;
    }
    wheel_is_subscribed_ = true;
    wheel_mtx_.unlock();
}

void MotionPlanner::armPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    arm_mtx_.lock();
    if (arm_pose_msg_.header.stamp < msg->header.stamp)
    {
        arm_pose_msg_.header.stamp = msg->header.stamp;
        arm_pose_msg_.pose         = msg->pose;
    }
    arm_is_subscribed_ = true;
    arm_mtx_.unlock();
}

//つかわないほうがいいかも
void MotionPlanner::publishTimerCallback(const ros::TimerEvent& e)
{
    this->publishState();
}

void MotionPlanner::publishState()
{
    sensor_msgs::JointState wheel_joint_msg;
    sensor_msgs::JointState arm_joint_msg;
    wheel_joint_msg.velocity.resize(4);
    arm_joint_msg.position.resize(6);
    arm_joint_msg.velocity.resize(1);
    if (wheel_is_subscribed_)
    {
        wheel_mtx_.lock();
        if (mecanum_is_moving_)
        {
            this->targetOdomToJoint();
        }
        wheel_joint_msg.header.stamp = ros::Time::now();
        for (int joint_i = 0; joint_i < 4; joint_i++)
        {
            wheel_joint_msg.velocity[joint_i] = target_wheel_joint_[joint_i];
        }
        wheel_joint_pub_.publish(wheel_joint_msg);
        wheel_mtx_.unlock();
    }
    if (arm_is_subscribed_)
    {
        arm_mtx_.lock();
        if (arm_is_moving_ || arm_is_moving_angle_)
        {
            this->targetPoseToJoint();
        }
        arm_joint_msg.header.stamp = ros::Time::now();
        for (int joint_i = 0; joint_i < 6; joint_i++)
        {
            arm_joint_msg.position[joint_i] = target_arm_joint_[joint_i];
        }
        if (end_effector_is_on_)
        {
            end_effector_cnt_++;
            end_effector_cnt_ %= 150;
            //printf("%d\n", end_effector_cnt_);
            if (end_effector_cnt_ < 75)
            {
                arm_joint_msg.velocity[0] = 10;
            }
            else
            {
                arm_joint_msg.velocity[0] = -10;
            }
        }
        else
        {
            arm_joint_msg.velocity[0] = 0;
        }
        arm_joint_pub_.publish(arm_joint_msg);
        arm_mtx_.unlock();
    }
}

void MotionPlanner::moveMecanumAbsolute(double *odom)
{
    wheel_mtx_.lock();
    mecanum_is_moving_ = true;
    for (int odom_i = 0; odom_i < 3; odom_i++)
    {
        target_odom_[odom_i] = odom[odom_i];
    }
    wheel_mtx_.unlock();
}

void MotionPlanner::moveMecanumRelative(double *odom)
{
    wheel_mtx_.lock();
    mecanum_is_moving_ = true;
    target_odom_[0] = wheel_odom_msg_.pose.pose.position.x * 1000 + odom[0];
    target_odom_[1] = wheel_odom_msg_.pose.pose.position.y * 1000 + odom[1];
    double msg_angle = atan2(wheel_odom_msg_.pose.pose.orientation.z, wheel_odom_msg_.pose.pose.orientation.w) * 2.0;
    target_odom_[2] = msg_angle + odom[2];
    wheel_mtx_.unlock();
}

void MotionPlanner::moveArmAbsolute(double *pose)
{
    arm_mtx_.lock();
    arm_is_moving_ = true;
    if (!arm_saved_data_is_set)
    {
        arm_pose_saved_data_[0] = arm_pose_msg_.pose.position.x;
        arm_pose_saved_data_[1] = arm_pose_msg_.pose.position.y;
        arm_pose_saved_data_[2] = arm_pose_msg_.pose.position.z;
        arm_saved_data_is_set = true;
        std::cout << "GET" << arm_pose_saved_data_[0] << " " << arm_pose_saved_data_[1] << " " << arm_pose_saved_data_[2] << std::endl;
    }
    for (int pose_i = 0; pose_i < 6; pose_i++)
    {
        target_pose_[pose_i] = pose[pose_i];
    }
    arm_mtx_.unlock();
}

void MotionPlanner::moveArmRelative(double *pose)
{
    arm_mtx_.lock();
    arm_is_moving_ = true;
    if (!arm_saved_data_is_set)
    {
        arm_pose_saved_data_[0] = arm_pose_msg_.pose.position.x;
        arm_pose_saved_data_[1] = arm_pose_msg_.pose.position.y;
        arm_pose_saved_data_[2] = arm_pose_msg_.pose.position.z;
        std::cout << "GET" << arm_pose_saved_data_[0] << " " << arm_pose_saved_data_[1] << " " << arm_pose_saved_data_[2] << std::endl;
        arm_saved_data_is_set = true;
    }
    target_pose_[0] = arm_pose_saved_data_[0] * 1000 + pose[0];
    target_pose_[1] = arm_pose_saved_data_[1] * 1000 + pose[1];
    target_pose_[2] = arm_pose_saved_data_[2] * 1000 + pose[2];
    //TODO クォータニオン対応
    target_pose_[3] = pose[3];
    target_pose_[4] = pose[4];
    target_pose_[5] = pose[5];
    arm_mtx_.unlock();
}


void MotionPlanner::moveArmAngle(double *angles)
{
    arm_mtx_.lock();
    arm_is_moving_angle_ = true;
    for (int i = 0; i < 6; i++)
    {
        target_pose_[i] = angles[i];
    }
    arm_mtx_.unlock();
}

void MotionPlanner::moveArmInitPos1()
{
    arm_mtx_.lock();
    arm_is_moving_angle_ = true;
    target_angle_[0] = 0.0;
    target_angle_[1] = -M_PI / 12;
    target_angle_[2] = 0.0;
    target_angle_[3] = -5 * M_PI / 6;
    target_angle_[4] = 0.0;
    target_angle_[5] = -M_PI / 12;
    arm_mtx_.unlock();
}

void MotionPlanner::moveArmInitPos2()
{
    arm_mtx_.lock();
    arm_is_moving_angle_ = true;
    for (int i = 0; i < 6; i++)
    {
        target_angle_[i] = 0;
    }
    arm_mtx_.unlock();
}

void MotionPlanner::moveArmInvPos()
{
    arm_mtx_.lock();
    arm_is_moving_angle_ = true;
    for (int i = 0; i < 6; i++)
    {
        target_angle_[i] = arm_angles_saved_data_inv_[i];
    }
    arm_mtx_.unlock();
}

void MotionPlanner::waitForGoal(long timeout_sec)
{
    ros::Time begin_time = ros::Time::now();
    ros::Duration duration = ros::Time::now() - begin_time;
    bool is_goal = true;
    //ros::Rate rate(10);
    ros::Rate rate(100);
    while (ros::ok())
    {
        rate.sleep();
        is_goal = true;
        if (mecanum_is_moving_)
        {
            wheel_mtx_.lock();
            double msg_angle = atan2(wheel_odom_msg_.pose.pose.orientation.z, wheel_odom_msg_.pose.pose.orientation.w) * 2.0;
            if (abs(target_odom_[0] - wheel_odom_msg_.pose.pose.position.x * 1000) > odom_error_[0] ||
                abs(target_odom_[1] - wheel_odom_msg_.pose.pose.position.y * 1000) > odom_error_[1] ||
                abs(target_odom_[2] - msg_angle) > odom_error_[2])
            {
                is_goal = false;
            }
            wheel_mtx_.unlock();
        }
        if (arm_is_moving_)
        {
            arm_mtx_.lock();
            //std::cout << "x: " << abs(target_pose_[0] - arm_pose_saved_data_[0] * 1000) << std::endl;
            //std::cout << "y: " << abs(target_pose_[1] - arm_pose_saved_data_[1] * 1000) << std::endl;
            //std::cout << "z: " << abs(target_pose_[2] - arm_pose_saved_data_[2] * 1000) << std::endl;
            if (abs(target_pose_[0] - arm_pose_saved_data_[0] * 1000) > pose_error_[0] ||
                abs(target_pose_[1] - arm_pose_saved_data_[1] * 1000) > pose_error_[1] ||
                abs(target_pose_[2] - arm_pose_saved_data_[2] * 1000) > pose_error_[2])
            {
                is_goal = false;
            }
            arm_mtx_.unlock();
        }
        else if (arm_is_moving_angle_)
        {
            arm_mtx_.lock();
            bool check = false;
            for (int i = 0; i < 6; i++)
            {
                check = check || abs(target_angle_[i] - arm_angles_saved_data_[i]) > angle_error_[i];
            }
            if (check)
            {
                is_goal = false;
            }
            arm_mtx_.unlock();
        }
        if (is_goal)
        {
            ROS_INFO("GOAL");
            target_wheel_joint_[0] = 0;
            target_wheel_joint_[1] = 0;
            target_wheel_joint_[2] = 0;
            target_wheel_joint_[3] = 0;
            arm_is_moving_     = false;
            mecanum_is_moving_ = false;
            arm_is_moving_angle_ = false;
            break;
        }
        duration = ros::Time::now() - begin_time;
        if (timeout_sec < duration.sec)
        {
            ROS_INFO("TIME OUT");
            target_wheel_joint_[0] = 0;
            target_wheel_joint_[1] = 0;
            target_wheel_joint_[2] = 0;
            target_wheel_joint_[3] = 0;
            arm_is_moving_     = false;
            mecanum_is_moving_ = false;
            arm_is_moving_angle_ = false;
            break;
        }
        this->publishState();
        rate.sleep();
    }
}

void MotionPlanner::targetOdomToJoint()
{
    double target_vel_x, target_vel_y, target_vel_yaw;
    double msg_angle = atan2(wheel_odom_msg_.pose.pose.orientation.z, wheel_odom_msg_.pose.pose.orientation.w) * 2.0;
    double error_x   = target_odom_[0] - wheel_odom_msg_.pose.pose.position.x * 1000;
    double error_y   = target_odom_[1] - wheel_odom_msg_.pose.pose.position.y * 1000;
    double error_yaw = target_odom_[2] - msg_angle;
    double vel_gain = 100;
    double yaw_gain = 100;
    double wheel_vel = MAX_WHEEL_VEL_;//std::min(MAX_WHEEL_VEL_, MAX_WHEEL_VEL_ * vel_gain * (error_x * error_x + error_y * error_y));
    double max_wheel_yaw_vel = MAX_WHEEL_YAW_VEL_;//std::min(MAX_WHEEL_YAW_VEL_, MAX_WHEEL_YAW_VEL_ * yaw_gain * error_yaw * error_yaw);
    target_vel_x   = wheel_vel * error_x / sqrt(error_x * error_x + error_y * error_y);
    target_vel_y   = wheel_vel * error_y / sqrt(error_x * error_x + error_y * error_y);
    target_vel_yaw = max_wheel_yaw_vel;
    if (error_yaw < 0)
    {
        target_vel_yaw *= -1;
    }
    if (abs(error_x) < odom_error_[0])
    {
        target_vel_x = 0;
    }
    if (abs(error_y) < odom_error_[1])
    {
        target_vel_y = 0;
    }
    if (abs(error_yaw) < odom_error_[2])
    {
        target_vel_yaw = 0;
    }
    mecanum.setYawAngle(msg_angle);
    mecanum.calcInverse(target_vel_x, target_vel_y, target_vel_yaw);
    float omega0, omega1, omega2, omega3;
    mecanum.getRAD(omega0, omega1, omega2, omega3);
    target_wheel_joint_[0] = omega0;
    target_wheel_joint_[1] = omega1;
    target_wheel_joint_[2] = omega2;
    target_wheel_joint_[3] = omega3;
}

void MotionPlanner::targetPoseToJoint()
{
    if (arm_is_moving_angle_)
    {
        double error;
        double input_angle[6];
        for (int i = 0; i < 6; i++)
        {
            error = target_angle_[i] - arm_angles_saved_data_[i];
            if (0 < error)
            {
                input_angle[i] = arm_angles_saved_data_[i] + MAX_ARM_ANGLE_;
            }
            else
            {
                input_angle[i] = arm_angles_saved_data_[i] - MAX_ARM_ANGLE_;
            }
            if (abs(input_angle[i] - target_angle_[i]) < 1.5 * MAX_ARM_ANGLE_)
            {
                input_angle[i] = target_angle_[i];
            }
            target_arm_joint_[i] = input_angle[i];
            arm_angles_saved_data_[i] = input_angle[i];
        }
        return;
    }
    double input_pose[6];
    double error_x = target_pose_[0] - arm_pose_saved_data_[0] * 1000;
    double error_y = target_pose_[1] - arm_pose_saved_data_[1] * 1000;
    double error_z = target_pose_[2] - arm_pose_saved_data_[2] * 1000;

    
    if (0 < error_x)
    {
        input_pose[0] = arm_pose_saved_data_[0] * 1000 + MAX_ARM_TRANSLATE_;
    }
    else
    {
        input_pose[0] = arm_pose_saved_data_[0] * 1000 - MAX_ARM_TRANSLATE_;
    }
    if (0 < error_y)
    {
        input_pose[1] = arm_pose_saved_data_[1] * 1000 + MAX_ARM_TRANSLATE_;
    }
    else
    {
        input_pose[1] = arm_pose_saved_data_[1] * 1000 - MAX_ARM_TRANSLATE_;
    }
    if (0 < error_z)
    {
        input_pose[2] = arm_pose_saved_data_[2] * 1000 + MAX_ARM_TRANSLATE_;
    }
    else
    {
        input_pose[2] = arm_pose_saved_data_[2] * 1000 - MAX_ARM_TRANSLATE_;
    }
    arm_pose_saved_data_[0] = input_pose[0] / 1000;
    arm_pose_saved_data_[1] = input_pose[1] / 1000;
    arm_pose_saved_data_[2] = input_pose[2] / 1000;

    for (int pose_i = 0; pose_i < 3; pose_i++)
    {
        if (abs(input_pose[pose_i] - target_pose_[pose_i]) < MAX_ARM_TRANSLATE_)
        {
            input_pose[pose_i] = target_pose_[pose_i];
        }
    }
    Eigen::Vector3d pos_vec;
    Eigen::Matrix3d ori_mat;
    Eigen::Matrix3d rot_mat_r;
    Eigen::Matrix3d rot_mat_p;
    Eigen::Matrix3d rot_mat_y;
    rot_mat_r = Eigen::AngleAxisd(target_pose_[3], Eigen::Vector3d(1, 0, 0));
    rot_mat_p = Eigen::AngleAxisd(target_pose_[4], Eigen::Vector3d(0, 1, 0));
    rot_mat_y = Eigen::AngleAxisd(target_pose_[5], Eigen::Vector3d(0, 0, 1));
    ori_mat = rot_mat_r * rot_mat_p * rot_mat_y;
    pos_vec << input_pose[0], input_pose[1], input_pose[2];
    craneX7.calcInverse(arm_angles_vec_, pos_vec, ori_mat);
    for (int joint_i = 0; joint_i < 6; joint_i++)
    {
        target_arm_joint_[joint_i] = arm_angles_vec_(joint_i);
        arm_angles_saved_data_[joint_i] = arm_angles_vec_(joint_i);
        arm_angles_saved_data_inv_[joint_i] = arm_angles_vec_(joint_i);
    }
}

double MotionPlanner::getWheelOdomX()
{
    arm_mtx_.lock();
    double return_num = wheel_odom_msg_.pose.pose.position.x * 1000;
    arm_mtx_.unlock();
    return return_num;
}

double MotionPlanner::getWheelOdomY()
{
    arm_mtx_.lock();
    double return_num = wheel_odom_msg_.pose.pose.position.y * 1000;
    arm_mtx_.unlock();
    return return_num;
}

double MotionPlanner::getArmPoseX()
{
    arm_mtx_.lock();
    double return_num = arm_pose_msg_.pose.position.x * 1000;
    arm_mtx_.unlock();
    return return_num;
}

double MotionPlanner::getArmPoseY()
{
    arm_mtx_.lock();
    double return_num = arm_pose_msg_.pose.position.y * 1000;
    arm_mtx_.unlock();
    return return_num;
}

double MotionPlanner::getArmPoseZ()
{
    arm_mtx_.lock();
    double return_num = arm_pose_msg_.pose.position.z * 1000;
    arm_mtx_.unlock();
    return return_num;
}

double MotionPlanner::getArmBasePositionX()
{
    return arm_base_position_[0];
}

double MotionPlanner::getArmBasePositionY()
{
    return arm_base_position_[1];
}

double MotionPlanner::getArmBasePositionZ()
{
    return arm_base_position_[2];
}


void MotionPlanner::endEffectorOn()
{
    end_effector_is_on_ = true;;
}

void MotionPlanner::endEffectorOff()
{
    end_effector_is_on_ = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_motion_planner_node");
    MotionPlanner motion_planner;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    double odom[3] = {0, 0, 0};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    motion_planner.init();

    // 初期姿勢
    //pose[0] = 0.1;
    //pose[1] = 0.1;
    //pose[2] = 50.1;
    //pose[3] = 0;
    //pose[4] = -M_PI;
    //pose[5] = 0;
    //motion_planner.moveArmRelative(pose);
    //odom[0] = motion_planner.getWheelOdomX();
    ////odom[1] = motion_planner.getWheelOdomY() + 200;
    //odom[1] = motion_planner.getWheelOdomY();
    //motion_planner.moveMecanumAbsolute(odom);
    //motion_planner.waitForGoal(20);
    /**

    //トイレ横まで平行移動
    motion_planner.moveArmRelative(pose);
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY() + 100;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド右上まで前進
    motion_planner.moveArmRelative(pose);
    odom[0] = 600;
    odom[1] = motion_planner.getWheelOdomY();
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //90度旋回
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド左上（ゴミ箱）まで前進
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 350;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド右上まで戻る
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -500;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド右上まで戻る
    odom[0] = 500;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド左上（ゴミ箱）まで前進
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 350;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //少しバック
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 320;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //90度旋回
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド左下（ゴミ箱）まで前進
    odom[0] = -400;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド左上まで戻る
    odom[0] = 500;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド中央（トイレ前）まで移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);
    **/





    motion_planner.moveArmInvPos();
    motion_planner.waitForGoal(20);

    usleep(2000000);

    pose[0] = 0.1;
    pose[1] = 0.1;
    pose[2] = 0.1;
    pose[3] = 0;
    pose[4] = -M_PI;
    pose[5] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(20);

    
    motion_planner.endEffectorOn();

    pose[0] = motion_planner.getArmPoseX() + 200;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 400 - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    odom[0] = -50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);
    
    pose[0] = 0;
    pose[1] = 50;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 50;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = motion_planner.getArmPoseZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX() - 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = - motion_planner.getArmBasePositionZ() + 20;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY() + 100;
    pose[2] = - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY() - 200;
    pose[2] = - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY() + 100;
    pose[2] = - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 0;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 0;
    pose[2] = 10;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    motion_planner.endEffectorOff();

    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);
    /**




    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    pose[0] = -100.1;
    pose[1] = 0.1;
    pose[2] = -150.1;
    motion_planner.moveArmRelative(pose);
    motion_planner.moveMecanumAbsolute(odom);
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    pose[0] = motion_planner.getArmPoseX() - 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 0;
    motion_planner.moveArmAbsolute(pose);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -500;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = -600;
    odom[1] = -570;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);
    **/

    motion_planner.final();
}