#include "wrs_motion_planner_node.h"

MotionPlanner::MotionPlanner()
{
    MAX_WHEEL_VEL_     = 80;
    //MAX_WHEEL_YAW_VEL_ = M_PI_2 / 2.5;
    MAX_WHEEL_YAW_VEL_ = M_PI_2 / 8;
    MAX_ARM_TRANSLATE_ = 1;
    //MAX_ARM_ANGLE_ = M_PI / 72;
    MAX_ARM_ANGLE_ = M_PI / 180;
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
    //init_arm_angles_vec_ << 0.0, -M_PI / 12, 0.0, -5 * M_PI / 6, 0.0, -M_PI / 12;
    init_arm_angles_vec_ << 0.0, 0.0, 0.0, -M_PI_2 / 1.2, 0.0, -M_PI_2;
    init_arm_angles_vec_inv_ = Eigen::VectorXd(6);
    //init_arm_angles_vec_inv_ << 0.0, -M_PI / 12, 0.0, -5 * M_PI / 6, 0.0, -M_PI / 12; 
    init_arm_angles_vec_inv_ << 0.0, 0.0, 0.0, -M_PI_2 / 1.2, 0.0, -M_PI_2;
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
    //odom_error_[2] = M_PI / 72;
    odom_error_[2] = M_PI / 144;
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
        arm_joint_msg.position[0] = M_PI_2;
        arm_joint_msg.position[1] = 0.0;
        arm_joint_msg.position[2] = M_PI_2;
        arm_joint_msg.position[3] = -M_PI_2 / 2;
        arm_joint_msg.position[4] = 0.0;
        arm_joint_msg.position[5] = -M_PI_2;
        arm_angles_saved_data_[0] = M_PI_2;
        arm_angles_saved_data_[1] = 0.0;
        arm_angles_saved_data_[2] = M_PI_2;
        arm_angles_saved_data_[3] = -M_PI_2 / 2;
        arm_angles_saved_data_[4] = 0.0;
        arm_angles_saved_data_[5] = -M_PI_2;
        //for (int joint_i = 0; joint_i < 6; joint_i++)
        //{
        //    arm_joint_msg.position[joint_i] = 0;
        //    arm_angles_saved_data_[joint_i] = 0;
        //}
    }
    arm_joint_pub_.publish(arm_joint_msg);
    ROS_INFO("2...\n");
    arm_joint_pub_.publish(arm_joint_msg);
    usleep(2000000);
    ROS_INFO("1...\n");
    arm_joint_pub_.publish(arm_joint_msg);
    usleep(2000000);
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
    target_angle_[1] = 0.0;
    target_angle_[2] = 0.0;
    target_angle_[3] = -M_PI_2 / 2;
    target_angle_[4] = 0.0;
    target_angle_[5] = -M_PI_2;
    arm_mtx_.unlock();
}

void MotionPlanner::moveArmInitPos2()
{
    arm_mtx_.lock();
    arm_is_moving_angle_ = true;
    target_angle_[0] = M_PI_2;
    target_angle_[1] = 0.0;
    target_angle_[2] = M_PI_2;
    target_angle_[3] = -M_PI_2 / 2;
    target_angle_[4] = 0.0;
    target_angle_[5] = -M_PI_2;
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
        if (arm_is_moving_angle_)
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
        this->publishState();
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
        rate.sleep();
    }
}

void MotionPlanner::targetOdomToJoint()
{
    static double old_error_x   = 0;
    static double old_error_y   = 0;
    static double old_error_yaw = 0;
    static double int_error_yaw = 0;
    double target_vel_x, target_vel_y, target_vel_yaw;
    double msg_angle = atan2(wheel_odom_msg_.pose.pose.orientation.z, wheel_odom_msg_.pose.pose.orientation.w) * 2.0;
    double error_x   = target_odom_[0] - wheel_odom_msg_.pose.pose.position.x * 1000;
    double error_y   = target_odom_[1] - wheel_odom_msg_.pose.pose.position.y * 1000;
    double error_yaw = target_odom_[2] - msg_angle;
    if (M_PI_2 * 9 / 10 <= error_yaw)
    {
        old_error_x   = 0;
        old_error_y   = 0;
        old_error_yaw = 0;
        int_error_yaw = 0;
    }
    double dif_error_x = error_x - old_error_x;
    double dif_error_y = error_y - old_error_y;
    double dif_error_yaw = error_yaw - old_error_yaw;
    int_error_yaw += error_yaw + old_error_yaw;
    double vel_gain = 100;
    double yaw_gain = MAX_WHEEL_YAW_VEL_ * 0.6;
    double dif_yaw_gain = -0.1;
    double int_yaw_gain = 0.001;
    double wheel_vel = MAX_WHEEL_VEL_;//std::min(MAX_WHEEL_VEL_, MAX_WHEEL_VEL_ * vel_gain * (error_x * error_x + error_y * error_y));
    double max_wheel_yaw_vel = MAX_WHEEL_YAW_VEL_;//std::min(MAX_WHEEL_YAW_VEL_, abs(yaw_gain * error_yaw + dif_yaw_gain * dif_error_yaw + int_yaw_gain * int_error_yaw));
    //printf("%lf %lf\n", MAX_WHEEL_YAW_VEL_, abs(yaw_gain * error_yaw + dif_yaw_gain * dif_error_yaw + int_yaw_gain * int_error_yaw));
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

    old_error_x   = error_x;
    old_error_y   = error_y;
    old_error_yaw = error_yaw;
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