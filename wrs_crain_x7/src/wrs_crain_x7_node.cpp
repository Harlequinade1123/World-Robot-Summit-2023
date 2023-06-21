#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include "Dynamixel.h"

class CrainX7Node
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher  joint_pub_;
    ros::Subscriber joint_sub_;
    Dynamixel dxl_;
    int8_t  ids_[8]      = { 22, 23, 24, 25, 26, 27, 28, 29 };
    int32_t vals_[8]     = { 0, 0, 0, 0, 0, 0, 0, 0 };
    int32_t get_vals_[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    int angle_size_      = 8;
    int8_t end_effector_id_   = 28;
    int32_t end_effector_vel_ = 200;
    int32_t end_effector_dir_ = 0;
    sensor_msgs::JointState joint_msg_;
    std::mutex mtx_;
    ros::Time callback_time_;

    public:
    void init();
    void write();
    void read();
    void final();
    double map(double value, double start1, double stop1, double start2, double stop2);
    void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
};

void CrainX7Node::init()
{
    this->callback_time_ = ros::Time::now();
    this->joint_msg_.position.resize(8);
    this->joint_msg_.velocity.resize(1);
    this->joint_pub_ = this->nh_.advertise<sensor_msgs::JointState>("/wrs/arm/read", 10);
    this->joint_sub_ = this->nh_.subscribe("/wrs/arm/write", 10, &CrainX7Node::jointCallback, this);
    this->dxl_.begin("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5UJQQO-if00-port0", 1000000);
    this->dxl_.torqueOn(22);
    this->dxl_.torqueOn(23);
    this->dxl_.torqueOn(24);
    this->dxl_.torqueOn(25);
    this->dxl_.torqueOn(26);
    this->dxl_.torqueOn(27);
    this->dxl_.torqueOn(28);
    //this->dxl_.torqueOn(29);
    this->dxl_.readBulkPosition(this->ids_, this->vals_, this->angle_size_);
}

void CrainX7Node::write()
{
    //手先効果器の回転に対応
    int32_t vel = this->end_effector_dir_ * this->end_effector_vel_;
    this->dxl_.writeBulkPosAndOneVel(this->ids_, this->vals_, this->angle_size_, this->end_effector_id_, vel);
    //this->dxl_.writeBulkPosition(this->ids_, this->vals_, this->angle_size_);
}

void CrainX7Node::read()
{
    int32_t vel_data;
    if (this->dxl_.readBulkPosAndOneVel(this->ids_, this->get_vals_, this->angle_size_, this->end_effector_id_, vel_data))
    {
        this->joint_msg_.header.stamp = ros::Time::now();
        for (int i = 0; i < this->angle_size_; i++)
        {
            this->joint_msg_.position[i] = map(static_cast<double>(get_vals_[i]), 0, 4096, -M_PI, M_PI);
        }
        if (0 < vel_data)
        {
            this->joint_msg_.velocity[0] = 10;
        }
        else if (vel_data < 0)
        {
            this->joint_msg_.velocity[0] = -10;
        }
        else
        {
            this->joint_msg_.velocity[0] = 0;
        }
        this->joint_pub_.publish(this->joint_msg_);
    }
}

void CrainX7Node::final()
{
    this->dxl_.torqueOff(22);
    this->dxl_.torqueOff(23);
    this->dxl_.torqueOff(24);
    this->dxl_.torqueOff(25);
    this->dxl_.torqueOff(26);
    this->dxl_.torqueOff(27);
    this->dxl_.torqueOff(28);
    this->dxl_.torqueOff(29);
}

void CrainX7Node::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    if (callback_time_ < msg->header.stamp && 6 <= msg->position.size())
    {
        mtx_.lock();
        callback_time_ = msg->header.stamp;
        for (int i = 0; i < msg->position.size(); i++)
        {
            this->vals_[i] = map(msg->position[i], -M_PI, M_PI, 0, 4096);
        }
        this->angle_size_ = msg->position.size();
        
        if (0 < msg->velocity.size() && 0 < 1.0 <= msg->velocity[0])
        {
            this->end_effector_dir_ = 1;
        }
        else if (0 < msg->velocity.size() && msg->velocity[0] <= -1.0)
        {
            this->end_effector_dir_ = -1;
        }
        else
        {
            this->end_effector_dir_ = 0;
        }
        mtx_.unlock();
    }
}

double CrainX7Node::map(double value, double start1, double stop1, double start2, double stop2)
{
    if(start1 <= value && value <= stop1)
    {
        //return (stop2 - start2) / (stop1 - start1) * value;
        return (value - start1) * (stop2 - start2) / (stop1 - start1) + start2;
    }
    else
    {
        std::cerr << "ERROR::MAP::VALUE BOUNDARIES ARE NOT CORRECT" << std::endl;
        return 0;	
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_crain_x7_node");
    CrainX7Node node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    node.init();
    ros::Rate rate(50);
    ros::Time ros_now = ros::Time::now();
    ros::Time ros_old = ros::Time::now();
    while (ros::ok())
    {
        ros_now = ros::Time::now();
        ros::Duration ros_duration = ros_now - ros_old;
        node.write();
        node.read();
        printf("%u\n", ros_duration.nsec / 1000000);
        ros_old = ros_now;
        rate.sleep();
    }
    node.final();
}