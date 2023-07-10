#include "sketch.h"

Sketch::Sketch() : PSketch()
{
    std::random_device rnd;
    std::mt19937 mt_x(rnd());
    std::uniform_int_distribution<int> randx(-850, 850);
    std::mt19937 mt_y(rnd());
    std::uniform_int_distribution<int> randy(-750, 750);
    for (int item_i = 0; item_i < 5; item_i++)
    {
        bool x_check1, y_check1, x_check2, y_check2;
        do
        {
            this->item_xs[item_i] = randx(mt_x);
            this->item_ys[item_i] = randy(mt_y);
            x_check1 = -350 + 50 < this->item_xs[item_i] && this->item_xs[item_i] < 180 - 50;
            y_check1 = (-450 + 50 < this->item_ys[item_i] && this->item_ys[item_i] < -185 - 50);
            y_check1 = y_check1 || (185 + 50 < item_ys[item_i] && this->item_ys[item_i] < 450 - 50);
            x_check2 = 180 + 50 < this->item_xs[item_i] && this->item_xs[item_i] < 850 - 50;
            y_check2 = -450 + 50 < this->item_ys[item_i] && this->item_ys[item_i] < 450 - 50;
        }
        while (!((x_check1 && y_check1) || (x_check2 && y_check2)));
    }

    this->mecanum = Mecanum(50, 250, 270);
    this->craneX7 = CraneX7(this->arm_lengths, 7, 6);
    this->q_vec   = Eigen::VectorXd(this->axis_num);
    this->start_point_x = -850.0 + 200;//-850.0 + this->robot_tread / 2 + 50;
    this->start_point_y = -750.0 + 400;//-750.0 + this->robot_depth / 2 + 50;
    this->callback_time_                    = ros::Time::now();
    this->odom_msg_.pose.pose.position.x    = this->start_point_x * 0.001;
    this->odom_msg_.pose.pose.position.y    = this->start_point_y * 0.001;
    this->odom_msg_.pose.pose.position.z    = 0.0;
    this->odom_msg_.pose.pose.orientation.x = 0.0;
    this->odom_msg_.pose.pose.orientation.y = 0.0;
    this->odom_msg_.pose.pose.orientation.z = sin(this->robot_yaw / 2);
    this->odom_msg_.pose.pose.orientation.w = cos(this->robot_yaw / 2);
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
    this->arm_joint_sub_   = this->nh_.subscribe("/wrs/arm/write", 10, &Sketch::armJointCallback, this);

    size(800, 800, P3D);
}

void Sketch::wheelJointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    this->wheel_mtx_.lock();
    if ((is_simulation || callback_time_ < msg->header.stamp) && 4 <= msg->velocity.size())
    {
        callback_time_ = msg->header.stamp;
        this->mecanum.calcForward ( msg->velocity[0],
                                    msg->velocity[1],
                                    msg->velocity[2],
                                    msg->velocity[3] );
        float vx, vy, omega;
        this->mecanum.getVelocity (vx, vy, omega);
        this->odom_msg_.twist.twist.linear.x  = vx * 0.001;
        this->odom_msg_.twist.twist.linear.y  = vy * 0.001;
        this->odom_msg_.twist.twist.angular.z = omega;
    }
    this->wheel_mtx_.unlock();
}

void Sketch::armJointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    this->arm_mtx_.lock();
    if ((is_simulation || callback_time_ < msg->header.stamp) && this->axis_num <= msg->position.size())
    {
        callback_time_ = msg->header.stamp;
        for (int i = 0; i < this->axis_num; i++)
        {
            this->target_arm_angles[i] = msg->position[i];
        }
        if (0 < msg->velocity.size() && 0 < 1.0 <= msg->velocity[0])
        {
            this->end_effector_dir_ = 1.0;
        }
        else if (0 < msg->velocity.size() && msg->velocity[0] <= -1.0)
        {
            this->end_effector_dir_ = -1.0;
        }
        else
        {
            this->end_effector_dir_ = 0.0;
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
        double delta_time = ros_duration.nsec * 0.000000001;
        this->odom_msg_.header.stamp = ros_now;
        this->robot_yaw += odom_msg_.twist.twist.angular.z * delta_time;
        this->odom_msg_.pose.pose.orientation.z = sin(robot_yaw / 2);
        this->odom_msg_.pose.pose.orientation.w = cos(robot_yaw / 2);
        this->odom_msg_.pose.pose.position.x += cos(this->robot_yaw) * this->odom_msg_.twist.twist.linear.x * delta_time;
        this->odom_msg_.pose.pose.position.x -= sin(this->robot_yaw) * this->odom_msg_.twist.twist.linear.y * delta_time;
        this->odom_msg_.pose.pose.position.y += sin(this->robot_yaw) * this->odom_msg_.twist.twist.linear.x * delta_time;
        this->odom_msg_.pose.pose.position.y += cos(this->robot_yaw) * this->odom_msg_.twist.twist.linear.y * delta_time;
        this->wheel_odom_pub_.publish(this->odom_msg_);
        this->wheel_mtx_.unlock();
        this->arm_mtx_.lock();
        for (int i = 0; i < this->axis_num; i++)
        {
            this->q_vec(i) = this->arm_angles[i];
        }
        this->craneX7.calcForward(this->q_vec);
        double x, y, z;
        this->craneX7.getXYZ(x, y, z);
        this->arm_pose_msg_.header.stamp = ros_now;
        this->arm_pose_msg_.pose.position.x = x * 0.001;
        this->arm_pose_msg_.pose.position.y = y * 0.001;
        this->arm_pose_msg_.pose.position.z = z * 0.001;
        this->arm_pose_pub_.publish(this->arm_pose_msg_);
        this->arm_mtx_.unlock();
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
    
    
    double dt = 1.0 / 500.0;

    this->drawGrid();
    this->updateItems();
    this->drawItems();
    this->updateRobot(dt);
    this->drawRobot();
    translate(0.0, 0.0, -this->robot_height / 2 - this->wheel_radius);
    translate(this->arm_base_position_[0], this->arm_base_position_[1], this->arm_base_position_[2]);
    this->drawArm();
}

void Sketch::drawGrid()
{
    for (int i = -8; i <= 8; i++)
    {
        line(-500 * 8, 500 * i, 500 * 8, 500 * i);
        line(500 * i, -500 * 8, 500 * i, 500 * 8);
    }

    stroke(0, 200, 0);
    line(-850.0,  750.0,  850.0,  750.0);
    line(-850.0, -750.0,  850.0, -750.0);
    line( 850.0, -750.0,  850.0,  750.0);
    line(-850.0, -750.0, -850.0,  750.0);
    line(-350.0, -450.0, -350.0,  450.0);
    line(-350.0,  450.0,  850.0,  450.0);
    line(-350.0, -450.0,  850.0, -450.0);
    stroke(100);

    stroke(200, 0, 0);
    line(-845.0,  745.0,  845.0,  745.0);
    line(-845.0,  505.0,  845.0,  505.0);
    line( 845.0,  505.0,  845.0,  745.0);
    line(-845.0,  505.0, -845.0,  745.0);
    stroke(100);

    fill(230);
    pushMatrix();
    translate(-85.0, 0.0, 200.0);
    box(530.0, 370.0, 400.0);
    popMatrix();
}

void Sketch::drawItems()
{
    for (int item_i = 0; item_i < 5; item_i++)
    {
        pushMatrix();
        translate(this->item_xs[item_i], this->item_ys[item_i], this->item_size[item_i] / 2);
        box(this->item_size[item_i]);
        popMatrix();
    }
}

void Sketch::updateItems()
{
    float check_vecs[4][2];
    check_vecs[0][0] = this->robot_depth * 0.5 + this->wheel_radius; check_vecs[0][1] = 0.0;
    check_vecs[1][0] =-this->robot_depth * 0.5 - this->wheel_radius; check_vecs[1][1] = 0.0;
    check_vecs[2][0] = 0.0;                                          check_vecs[2][1] = this->robot_tread * 0.5 + 15;
    check_vecs[3][0] = 0.0;                                          check_vecs[3][1] =-this->robot_tread * 0.5 - 15;
    for (int item_i = 0; item_i < 5; item_i++)
    {
        float x = this->item_xs[item_i] - this->robot_x;
        float y = this->item_ys[item_i] - this->robot_y;
        float item_vec[2];
        item_vec[0] = x * cos(-this->robot_yaw) - y * sin(-this->robot_yaw);
        item_vec[1] = x * sin(-this->robot_yaw) + y * cos(-this->robot_yaw);
        if (this->robot_depth * 0.5 < std::min(abs(item_vec[0] - item_size[item_i]), std::abs(item_vec[0] + item_size[item_i])) ||
            this->robot_tread * 0.5 < std::min(abs(item_vec[1] - item_size[item_i]), std::abs(item_vec[1] + item_size[item_i])))
        {
            continue;
        }
        int   min_index  = 0;
        float min_length = 0;
        for (int vec_i = 0; vec_i < 4; vec_i++)
        {
            float vec_length = 0;
            if (check_vecs[vec_i][0] != 0.0)
            {
                vec_length = abs(item_vec[0] - check_vecs[vec_i][0]);
            }
            else
            {
                vec_length = abs(item_vec[1] - check_vecs[vec_i][1]);
            }
            if (vec_length < min_length)
            {
                min_length = vec_length;
                min_index  = vec_i;
            }
        }
        if (check_vecs[min_index][0] != 0.0)
        {
            item_vec[0] = check_vecs[min_index][0];
        }
        else
        {
            item_vec[1] = check_vecs[min_index][1];
        }
        this->item_xs[item_i] = item_vec[0] * cos(this->robot_yaw) - item_vec[1] * sin(this->robot_yaw) + this->robot_x;
        this->item_ys[item_i] = item_vec[0] * sin(this->robot_yaw) + item_vec[1] * cos(this->robot_yaw) + this->robot_y;
    }
}


void Sketch::updateRobot(float dt)
{
    if (this->wheel_mtx_.try_lock())
    {
        this->robot_x = odom_msg_.pose.pose.position.x * 1000;//[m] -> [mm]
        this->robot_y = odom_msg_.pose.pose.position.y * 1000;//[m] -> [mm]
        this->wheel_mtx_.unlock();
    }
    if (this->arm_mtx_.try_lock())
    {
        for (int i = 0; i < this->axis_num; i++)
        {
            float sign = 0.0;
            float angle_error = this->target_arm_angles[i] - this->arm_angles[i];
            if (angle_error > 0.01)
            {
                sign = 1.0;
                this->arm_angles[i] += sign * this->angle_rpm_ * M_PI / 30.0 * dt;
            }
            else if (angle_error < -0.01)
            {
                sign = -1.0;
                this->arm_angles[i] += sign * this->angle_rpm_ * M_PI / 30.0 * dt;
            }
            else
            {
                this->arm_angles[i] = this->target_arm_angles[i];
            }
            this->end_effector_angle_ += this->end_effector_dir_ * this->end_effector_vel_ * M_PI / 30.0 * dt;
        }
        this->arm_mtx_.unlock();
    }
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
    fill(80);
    translate(0.0, 0.0, -this->arm_lengths[0] / 2.0 + this->arm_lengths[0] / 8.5);
    box(this->arm_size * 3, this->arm_size * 3, this->arm_lengths[0] / 4);
    translate(0.0, 0.0, this->arm_lengths[0] / 2.0 - this->arm_lengths[0] / 8.5);
    fill(110);
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

    rotateY(-this->arm_angles[1]);

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

    rotateY(-this->arm_angles[3]);

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

    rotateY(-this->arm_angles[5]);

    translate(0.0, 0.0, (this->arm_lengths[6] - this->end_effector_length_) / 2.0);
    box(this->arm_size, this->arm_size, this->arm_lengths[6] - this->end_effector_length_);
    translate(0.0, 0.0, (this->arm_lengths[6] - this->end_effector_length_) / 2.0);

    rotateZ(this->end_effector_angle_);

    fill(200, 200, 0);
    translate(0.0, 0.0, this->end_effector_length_ / 2.0);
    box(this->arm_size * 1.5, this->arm_size * 1.5, this->end_effector_length_);
    translate(0.0, 0.0, this->end_effector_length_ / 2.0);
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
{}

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