#include "sketch.h"

Sketch::Sketch() : PSketch()
{
    this->mecanum = Mecanum(50, 250, 270);
    size(800, 800, P3D);
}

void Sketch::parallelTask1()
{

    while(!glfwWindowShouldClose(this->window))
    {
        for (int i = 0; i < 4; i++)
        {
            if (this->dxl_is_connected)
                this->dxl.writeRPM(this->ids[i], this->vals[i]);
            //this->dxl.writeVelocity(this->ids[i], 200);
            printf("%d ", (int)(this->vals[i] / 0.229));
        }
        printf("\n");
        usleep(5000);
    }
}

void Sketch::parallelTask2()
{
    //while(!glfwWindowShouldClose(this->window))
    //{
    //    for (int i = 0; i < 4; i++)
    //    {
    //        if (this->dxl_is_connected)
    //        {
    //            int vel = this->dxl.readVelocity(this->ids[i]);
    //            printf("vel%d:%d ", i, vel);
    //        }
    //    }
    //    printf("\n");
    //    usleep(50000);
    //}
}

void Sketch::parallelTask3()
{}

void Sketch::setup()
{
    if (this->dxl_is_connected)
    {
        this->dxl.begin("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2N05OG-if00-port0", 1000000);
        for (int i = 0; i < 4; i++)
        {
            dxl.torqueOn(this->ids[i]);
        }
    }
    stroke(255, 255, 255);
    frameRate(500);
}

void Sketch::draw()
{
    if (glfwGetKey(this->window, GLFW_KEY_W) == GLFW_PRESS)
    {
        robot_vx += 10.0;
        if (400 < robot_vx) robot_vx = 400;
    }
    if (glfwGetKey(this->window, GLFW_KEY_S) == GLFW_PRESS)
    {
        robot_vx -= 10.0;
        if (robot_vx < -400) robot_vx = -400;
    }
    if (glfwGetKey(this->window, GLFW_KEY_A) == GLFW_PRESS)
    {
        robot_vy += 10.0;
        if (400 < robot_vy) robot_vy = 400;
    }
    if (glfwGetKey(this->window, GLFW_KEY_D) == GLFW_PRESS)
    {
        robot_vy -= 10.0;
        if (robot_vy < -400) robot_vy = -400;
    }
    if (glfwGetKey(this->window, GLFW_KEY_E) == GLFW_PRESS)
    {
        robot_w -= M_PI / 36.0;
        if (robot_w < -M_PI) robot_w = -M_PI;
    }
    if (glfwGetKey(this->window, GLFW_KEY_Q) == GLFW_PRESS)
    {
        robot_w += M_PI / 36.0;
        if (M_PI < robot_w) robot_w = M_PI;
    }
    if (glfwGetKey(this->window, GLFW_KEY_R) == GLFW_PRESS)
    {
        robot_x   = 0.0;
        robot_y   = 0.0;
        robot_yaw = 0.0;
        robot_vx  = 0.0;
        robot_vy  = 0.0;
        robot_w   = 0.0;
    }
    mecanum.calcInvese(robot_vx, robot_vy, robot_w);
    mecanum.getRPM(this->vals[0], this->vals[1], this->vals[2], this->vals[3]);
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
    this->robot_x   += this->robot_vx * dt;
    this->robot_y   += this->robot_vy * dt;
    this->robot_yaw += this->robot_w  * dt;
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
