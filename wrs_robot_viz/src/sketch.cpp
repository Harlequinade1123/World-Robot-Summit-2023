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

    this->odom_pub_  = this->nh_.advertise<nav_msgs::Odometry>("/wrs/odom", 10);
    this->joint_sub_ = this->nh_.subscribe("/wrs/joint", 10, &Sketch::jointCallback, this);

    size(800, 800, P3D);
}

void Sketch::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    this->mtx_.lock();
    if (callback_time_ < msg->header.stamp && 4 <= msg->velocity.size())
    {
        callback_time_ = msg->header.stamp;
        this->mecanum.calcForward ( msg->velocity[0],
                                    msg->velocity[1],
                                    msg->velocity[2],
                                    msg->velocity[3] );
        float vx, vy, omega;
        this->mecanum.getVelocity (vx, vy, omega);
        this->odom_msg_.twist.twist.linear.x  = vx;
        this->odom_msg_.twist.twist.linear.y  = vy;
        this->odom_msg_.twist.twist.angular.z = omega;
    }
    this->mtx_.unlock();
}

void Sketch::parallelTask1()
{
    ros::Rate rate(200);
    while (ros::ok())
    {
        this->mtx_.lock();
        double delta_time = 0;
        double qz_tmp = this->odom_msg_.pose.pose.orientation.z;
        double qw_tmp = this->odom_msg_.pose.pose.orientation.w;
        this->odom_msg_.pose.pose.orientation.z += qw_tmp * delta_time * 0.5 * odom_msg_.twist.twist.angular.z;
        this->odom_msg_.pose.pose.orientation.w += qz_tmp * delta_time * 0.5 * odom_msg_.twist.twist.angular.z;
        this->odom_msg_.pose.pose.position.x += this->odom_msg_.twist.twist.linear.x * delta_time;
        this->odom_msg_.pose.pose.position.y += this->odom_msg_.twist.twist.linear.y * delta_time;
        this->odom_pub_.publish(this->odom_msg_);
        this->mtx_.unlock();
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
    if (this->mtx_.try_lock())
    {
        this->robot_x   = odom_msg_.pose.pose.position.x/*[m]*/ * 1000;/*[mm]*/
        this->robot_y   = odom_msg_.pose.pose.position.y/*[m]*/ * 1000;/*[mm]*/
        this->robot_yaw = 2.0*atan2(this->odom_msg_.pose.pose.orientation.z,
                                    this->odom_msg_.pose.pose.orientation.w);
        this->mtx_.unlock();
    }
    
    this->drawRobot();

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
            this->camera_distance += old_MOUSEY - MOUSEY;
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
