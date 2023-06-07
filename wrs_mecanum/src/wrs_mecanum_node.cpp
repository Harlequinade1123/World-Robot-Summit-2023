#include <ros/ros.h>
#include "kinematics.h"
#include "Dynamixel.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <iostream>
#include <chrono>

#include <termios.h>
#include <fcntl.h>

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}


uint8_t ids[4]  = { 14, 11, 12, 13 };
float   vals[4] = { 0.0, 0.0, 0.0, 0.0 };
int32_t get_vals[4] = { 0, 0, 0, 0 };

class MecanumNode
{
    private:
    ros::NodeHandle nh_;
    Dynamixel dxl_;
    uint8_t ids[4]  = { 14, 11, 12, 13 };
    float   vals[4] = { 0.0, 0.0, 0.0, 0.0 };
    int32_t get_vals[4] = { 0, 0, 0, 0 };

    public:
    void init();
    void write();
    void read();
};

void MecanumNode::init()
{
}

void MecanumNode::write()
{
}

void MecanumNode::read()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_mecanum_node");
    ros::NodeHandle nh;
    Mecanum mecanum(50, 250, 270);
    Dynamixel dxl;
    dxl.begin("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2N05OG-if00-port0", 1000000);
    for (int i = 0; i < 4; i++)
    {
        dxl.torqueOn(ids[i]);
    }

    ros::Rate rate(200);
    float robot_vx = 0, robot_vy = 0, robot_w = 0;
    ros::Time ros_now = ros::Time::now();
    ros::Time ros_old = ros::Time::now();
    while (ros::ok())
    {
        if (kbhit())
        {
            char key = getchar();
            if (key == 'q')
            {
                robot_vx = 100;
                robot_vy = 100;
            }
            else if (key == 'w')
            {
                robot_vx = 100;
                robot_vy = 0;
            }
            else if (key == 'e')
            {
                robot_vx = 100;
                robot_vy = -100;
            }
            else if (key == 'a')
            {
                robot_vx = 0;
                robot_vy = 100;
            }
            else if (key == 's')
            {
                robot_vx = 0;
                robot_vy = 0;
            }
            else if (key == 'd')
            {
                robot_vx = 0;
                robot_vy = -100;
            }
            else if (key == 'z')
            {
                robot_vx = -100;
                robot_vy = 100;
            }
            else if (key == 'x')
            {
                robot_vx = -100;
                robot_vy = 0;
            }
            else if (key == 'c')
            {
                robot_vx = -100;
                robot_vy = -100;
            }
        }
        mecanum.calcInvese(robot_vx, robot_vy, robot_w);
        mecanum.getRPM(vals[0], vals[1], vals[2], vals[3]);

        //for (int i = 0; i < 4; i++)
        //{
        //    dxl.writeVelocity(ids[i], vals[i] / 0.229);
        //}
        bool get_data = true;
        //for (int i = 0; i < 4; i++)
        //{
        //    get_data = get_data && dxl.readVelocity(ids[i], get_vals[i]);
        //}
        //if (get_data)
        //{
        //    
        //}
        dxl.writeBulkVelocity(vals[0] / 0.229, vals[1] / 0.229, vals[2] / 0.229, vals[3] / 0.229);
        if (dxl.readBulkVelocity(get_vals[0], get_vals[1], get_vals[2], get_vals[3]))
        {
            printf("%d, %d, %d, %d : ", get_vals[0], get_vals[1], get_vals[2], get_vals[3]);
        }
        ros_now = ros::Time::now();
        ros::Duration ros_duration = ros_now - ros_old;
        printf("%u\n", ros_duration.nsec / 1000000);
        ros_old = ros_now;

        rate.sleep();
    }
}