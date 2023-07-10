#include "wrs_motion_planner_node.h"

void motion1();
void motion2();
void motion3();
void cleaningMotion();
void turningMotion();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_motion_planner_init_node");
    MotionPlanner motion_planner;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    double odom[3] = {0, 0, 0};
    double pose[6] = {0, 0, 0, 0, 0, 0};
    motion_planner.init();


    // 初期姿勢
    pose[0] = 0.1;
    pose[1] = 0.1;
    pose[2] = 50.1;
    pose[3] = 0;
    pose[4] = -M_PI;
    pose[5] = 0;
    motion_planner.moveArmInitPos2();
    //motion_planner.moveArmRelative(pose);
    odom[0] = motion_planner.getWheelOdomX();
    ////odom[1] = motion_planner.getWheelOdomY() + 200;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);
}