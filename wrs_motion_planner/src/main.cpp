#include "wrs_motion_planner_node.h"

void motion1();
void motion2();
void motion3();
void cleaningMotion();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_motion_planner_node");
    //motion1();
    //motion2();
    //motion3();
    cleaningMotion();
}

void motion1()
{
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
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //トイレ横まで平行移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY() + 100;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //フィールド右上まで前進
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


    //トイレ清掃
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(20);
    usleep(2000000);
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

    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    
    motion_planner.endEffectorOn();

    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 430 - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    odom[0] = 50;
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
    
    odom[0] = -50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    usleep(1000000);

    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

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

    motion_planner.final();
}

void motion2()
{
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
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //トイレ横まで平行移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY() + 100;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //フィールド右上まで前進
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

    //フィールド中央（トイレ前）まで移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //90度旋回
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //トイレ清掃
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(20);
    usleep(2000000);
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

    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    
    motion_planner.endEffectorOn();

    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 430 - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);
    
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
    
    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    usleep(1000000);

    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

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

    motion_planner.final();
}

void motion3()
{}

void cleaningMotion()
{
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
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //トイレ清掃
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(20);
    usleep(2000000);
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

    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    
    motion_planner.endEffectorOn();

    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 430 - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);
    
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
    
    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    usleep(1000000);

    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);

    motion_planner.final();
}