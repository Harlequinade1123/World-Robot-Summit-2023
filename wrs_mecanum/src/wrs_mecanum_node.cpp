#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include "Dynamixel.h"

class MecanumNode
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher  joint_pub_;
    ros::Subscriber joint_sub_;
    Dynamixel dxl_;
    float   vals_[4] = { 0.0, 0.0, 0.0, 0.0 };
    int32_t get_vals_[4] = { 0, 0, 0, 0 };
    sensor_msgs::JointState joint_msg_;
    std::mutex mtx_;
    ros::Time callback_time_;

    public:
    void init();
    void write();
    void read();
    void final();
    void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
};

void MecanumNode::init()
{
    this->callback_time_ = ros::Time::now();
    this->joint_msg_.velocity.resize(4);
    this->joint_pub_ = this->nh_.advertise<sensor_msgs::JointState>("/wrs/wheel/read", 10);
    this->joint_sub_ = this->nh_.subscribe("/wrs/wheel/write", 10, &MecanumNode::jointCallback, this);
    this->dxl_.begin("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2N05OG-if00-port0", 1000000);
    this->dxl_.torqueOn(11);
    this->dxl_.torqueOn(12);
    this->dxl_.torqueOn(13);
    this->dxl_.torqueOn(14);
}

void MecanumNode::write()
{
    this->dxl_.writeBulkVelocity(static_cast<int32_t>(vals_[0]),
                                static_cast<int32_t> (vals_[1]),
                                static_cast<int32_t> (vals_[2]),
                                static_cast<int32_t> (vals_[3]));
}

void MecanumNode::read()
{
    if (this->dxl_.readBulkVelocity(get_vals_[0], get_vals_[1], get_vals_[2], get_vals_[3]))
    {
        this->joint_msg_.header.stamp = ros::Time::now();
        this->joint_msg_.velocity[0] = static_cast<double>(get_vals_[0]) * 0.229 * M_PI / 30.0;
        this->joint_msg_.velocity[1] = static_cast<double>(get_vals_[1]) * 0.229 * M_PI / 30.0;
        this->joint_msg_.velocity[2] = static_cast<double>(get_vals_[2]) * 0.229 * M_PI / 30.0;
        this->joint_msg_.velocity[3] = static_cast<double>(get_vals_[3]) * 0.229 * M_PI / 30.0;
        this->joint_pub_.publish(this->joint_msg_);
        printf("%d, %d, %d, %d : ", get_vals_[0], get_vals_[1], get_vals_[2], get_vals_[3]);
    }
}

void MecanumNode::final()
{
    this->dxl_.writeBulkVelocity(0, 0, 0, 0);
}

void MecanumNode::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    if (callback_time_ < msg->header.stamp && 4 <= msg->velocity.size())
    {
        mtx_.lock();
        callback_time_ = msg->header.stamp;
        this->vals_[0] = msg->velocity[0] * 30.0 / M_PI / 0.229;
        this->vals_[1] = msg->velocity[1] * 30.0 / M_PI / 0.229;
        this->vals_[2] = msg->velocity[2] * 30.0 / M_PI / 0.229;
        this->vals_[3] = msg->velocity[3] * 30.0 / M_PI / 0.229;
        mtx_.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_mecanum_node");
    MecanumNode node;
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