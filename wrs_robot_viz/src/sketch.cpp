#include "sketch.h"

Sketch::Sketch() : PSketch()
{
    this->mecanum = Mecanum(50, 250, 270);
    callback_time_   = ros::Time::now();
    this->odom_msg_.pose.pose.position.x    = 0.0;
    this->odom_msg_.pose.pose.position.y    = 0.0;
    this->odom_msg_.pose.pose.position.z    = 0.0;
    this->odom_msg_.pose.pose.orientation.x = 0.0;
    this->odom_msg_.pose.pose.orientation.y = 0.0;
    this->odom_msg_.pose.pose.orientation.z = 0.0;
    this->odom_msg_.pose.pose.orientation.w = 1.0;
    this->odom_msg_.twist.twist.linear.x    = 0.0;
    this->odom_msg_.twist.twist.linear.y    = 0.0;
    this->odom_msg_.twist.twist.linear.z    = 0.0;
    this->odom_msg_.twist.twist.angular.x   = 0.0;
    this->odom_msg_.twist.twist.angular.y   = 0.0;
    this->odom_msg_.twist.twist.angular.z   = 0.0;

    this->ros_now = ros::Time::now();
    this->ros_old = ros::Time::now();

    this->wheel_odom_pub_  = this->nh_.advertise<nav_msgs::Odometry>("/wrs/wheel/odom", 10);
    this->arm_pose_pub_    = this->nh_.advertise<geometry_msgs::PoseStamped>("/wrs/arm/pose", 10);
    this->wheel_joint_sub_ = this->nh_.subscribe("/wrs/wheel/read", 10, &Sketch::wheelJointCallback, this);
    this->arm_joint_sub_   = this->nh_.subscribe("/wrs/arm/read", 10, &Sketch::armJointCallback, this);

    size(800, 800, P3D);
}

void Sketch::wheelJointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    this->wheel_mtx_.lock();
    if (callback_time_ < msg->header.stamp && 4 <= msg->velocity.size())
    {
        callback_time_ = msg->header.stamp;
        this->mecanum.calcForward ( msg->velocity[0],
                                    msg->velocity[1],
                                    msg->velocity[2],
                                    msg->velocity[3] );
        float vx, vy, omega;
        this->mecanum.getVelocity (vx, vy, omega);
        this->odom_msg_.twist.twist.linear.x  = vx * 0.000001;
        this->odom_msg_.twist.twist.linear.y  = vy * 0.000001;
        this->odom_msg_.twist.twist.angular.z = omega;
    }
    this->wheel_mtx_.unlock();
}

void Sketch::armJointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    this->arm_mtx_.lock();
    if (callback_time_ < msg->header.stamp && 8 <= msg->position.size())
    {
        callback_time_ = msg->header.stamp;
        for (int i = 0; i < 8; i++)
        {
            arm_angles[i] = msg->position[i];
        }
    }
    this->arm_mtx_.unlock();
}


void Sketch::parallelTask1()
{
    ros::Rate rate(200);
    ros::Time ros_now = ros::Time::now();
    ros::Time ros_old = ros::Time::now();
    ros::Duration ros_duration;
    while (ros::ok())
    {
        ros_now = ros::Time::now();
        this->wheel_mtx_.lock();
        ros_duration = ros_now - ros_old;
        double delta_time = ros_duration.nsec / 1000000;
        double qz_tmp = 0;//this->odom_msg_.pose.pose.orientation.z;
        double qw_tmp = 1;//this->odom_msg_.pose.pose.orientation.w;
        this->odom_msg_.pose.pose.orientation.z += 0;//qw_tmp * delta_time * 0.5 * odom_msg_.twist.twist.angular.z;
        this->odom_msg_.pose.pose.orientation.w += 0;//qz_tmp * delta_time * 0.5 * odom_msg_.twist.twist.angular.z;
        this->odom_msg_.pose.pose.position.x += this->odom_msg_.twist.twist.linear.x * delta_time;
        this->odom_msg_.pose.pose.position.y += this->odom_msg_.twist.twist.linear.y * delta_time;
        this->wheel_odom_pub_.publish(this->odom_msg_);
        this->wheel_mtx_.unlock();
        ros_old = ros_now;
        rate.sleep();
    }
}

void Sketch::parallelTask2()
{}

void Sketch::parallelTask3()
{}

void Sketch::setup()
{
    stroke(255, 255, 255);
    frameRate(200);
}

void Sketch::draw()
{
    if (!ros::ok())
    {
        exit(1);
    }
    background(200);
    scale(0.5,0.5,0.5);
    strokeWeight(1.5);
    stroke(100);
    strokeCap(ROUND);
    for (int i = -8; i <= 8; i++)
    {
        line(-500 * 8, 500 * i, 500 * 8, 500 * i);
        line(500 * i, -500 * 8, 500 * i, 500 * 8);
    }
    
    double dt = 1.0 / 500.0;
    if (this->wheel_mtx_.try_lock())
    {
        this->robot_x   = odom_msg_.pose.pose.position.x * 1000;//[m] -> [mm]
        this->robot_y   = odom_msg_.pose.pose.position.y * 1000;//[m] -> [mm]
        this->robot_yaw = 0.0;
        this->wheel_mtx_.unlock();
    }
    
    this->drawRobot();
    translate(0.0, 0.0, this->robot_height / 2);
    this->drawArm();
}

void Sketch::keyEvent(int key, int action)
{
    if (action == GLFW_PRESS)
    {
        if (key == GLFW_KEY_SPACE)
            //std::cout << "Space Pressed" << std::endl;
        if (key == GLFW_KEY_RIGHT)
            rotateCamera(M_PI / 18);
        if (key == GLFW_KEY_LEFT)
            rotateCamera(-M_PI / 18);
    }
}

void Sketch::mouseButtonEvent(int button, int action)
{
    if (action == GLFW_PRESS)
    {
        //std::cout << "Button Pressed" << std::endl;
    }
}

void Sketch::cursorPosEvent(double xpos, double ypos)
{
    if (glfwGetMouseButton(this->window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        this->camera_angle -= (MOUSEX - old_MOUSEX) / 2 * M_PI / 180;
        if (this->camera_angle < -2 * M_PI)
        {
            this->camera_angle += 2 * M_PI;
        }
        else if (2 * M_PI < this->camera_angle)
        {
            this->camera_angle -= 2 * M_PI;
        }
        else
        {
            this->camera_distance -= old_MOUSEY - MOUSEY;
        }
        if (this->camera_distance < 0.1)
        {
            this->camera_distance = 0.1;
        }
        if (1000 < this->camera_distance)
        {
            this->camera_distance = 1000;
        }
        if (this->camera_height < 10)
        {
            this->camera_height = 10;
        }
        if (1000 < this->camera_height)
        {
            this->camera_height = 1000;
        }
        this->setCamera(this->camera_distance, this->camera_height, this->camera_angle);
    }
    old_MOUSEX = MOUSEX;
    old_MOUSEY = MOUSEY;
}

void Sketch::scrollEvent(double xoffset, double yoffset)
{
    this->camera_height += yoffset * 50;
    if (this->camera_height < 10)
    {
        this->camera_height = 10;
    }
    if (1000 < this->camera_height)
    {
        this->camera_height = 1000;
    }
    this->setCamera(this->camera_distance, this->camera_height, this->camera_angle);
}

void Sketch::drawRobot()
{
    stroke(50);
    translate(this->robot_x, this->robot_y, this->robot_height / 2 + this->wheel_radius);
    rotateZ(this->robot_yaw);
    fill(150);
    box(this->robot_depth, this->robot_tread - this->wheel_width, this->robot_height);
    pushMatrix();
    translate(0.0, 0.0, this->wheel_radius * 2 - this->robot_height / 2 - 30);
    box(this->robot_depth + this->wheel_radius * 2, this->robot_tread + this->wheel_width, 30);
    popMatrix();
    fill(100);
    pushMatrix();
    translate(this->robot_depth / 2, this->robot_tread / 2,-this->robot_height / 2);
    rotateX(M_PI_2);
    cylinder(this->wheel_radius, this->wheel_width);
    popMatrix();
    pushMatrix();
    translate(this->robot_depth / 2, -this->robot_tread / 2,-this->robot_height / 2);
    rotateX(M_PI_2);
    cylinder(this->wheel_radius, this->wheel_width);
    popMatrix();
    pushMatrix();
    translate(-this->robot_depth / 2, this->robot_tread / 2,-this->robot_height / 2);
    rotateX(M_PI_2);
    cylinder(this->wheel_radius, this->wheel_width);
    popMatrix();
    pushMatrix();
    translate(-this->robot_depth / 2, -this->robot_tread / 2,-this->robot_height / 2);
    rotateX(M_PI_2);
    cylinder(this->wheel_radius, this->wheel_width);
    popMatrix();
}

void Sketch::drawArm()
{
    fill(110);
    translate(0.0, 0.0, this->arm_lengths[0] / 2.0);
    box(this->arm_size * 2, this->arm_size * 2, this->arm_lengths[0]);
    translate(0.0, 0.0, this->arm_lengths[0] / 2.0);
    
    rotateZ(this->arm_angles[0]);

    translate(0.0, 0.0, this->arm_lengths[1] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[1]);
    translate(0.0, 0.0, this->arm_lengths[1] / 2.0);
    
    fill(130);
    rotateX(M_PI_2);
    cylinder(this->arm_size / 2.0 * 1.2, this->arm_size * 1.1);
    rotateX(-M_PI_2);
    fill(110);

    rotateY(this->arm_angles[1]);

    translate(0.0, 0.0, this->arm_lengths[2] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[2]);
    translate(0.0, 0.0, this->arm_lengths[2] / 2.0);

    rotateZ(this->arm_angles[2]);

    translate(0.0, 0.0, this->arm_lengths[3] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[3]);
    translate(0.0, 0.0, this->arm_lengths[3] / 2.0);

    fill(130);
    rotateX(M_PI_2);
    cylinder(this->arm_size / 2.0 * 1.2, this->arm_size * 1.1);
    rotateX(-M_PI_2);
    fill(110);

    rotateY(this->arm_angles[3]);

    translate(0.0, 0.0, this->arm_lengths[4] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[4]);
    translate(0.0, 0.0, this->arm_lengths[4] / 2.0);

    rotateZ(this->arm_angles[4]);

    translate(0.0, 0.0, this->arm_lengths[5] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[5]);
    translate(0.0, 0.0, this->arm_lengths[5] / 2.0);

    fill(130);
    rotateX(M_PI_2);
    cylinder(this->arm_size / 2.0 * 1.2, this->arm_size * 1.1);
    rotateX(-M_PI_2);
    fill(110);

    rotateY(this->arm_angles[5]);

    translate(0.0, 0.0, this->arm_lengths[6] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[6]);
    translate(0.0, 0.0, this->arm_lengths[6] / 2.0);

    rotateZ(this->arm_angles[6]);

    translate(0.0, 0.0, this->arm_lengths[7] / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[7]);
    translate(0.0, 0.0, this->arm_lengths[7] / 2.0);

}