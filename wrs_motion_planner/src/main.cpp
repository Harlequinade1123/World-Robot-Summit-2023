#include "wrs_motion_planner_node.h"

void motion1();
void motion2();
void motion2U();
void motion3();
void cleaningMotion();
void turningMotion();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_motion_planner_node");
    //motion1();
    //motion2();
    motion2U();
   // motion3();
    
    //cleaningMotion();
    //turningMotion();
}

//-----------------------------------------------------------------------------------------------------------------------------

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
    //odom[1] = 50;
    odom[1] = 20;
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

    //ちょっと下がる
    odom[0] = 200;
    odom[1] = 0;


    //便座清掃
    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 460 - motion_planner.getArmBasePositionZ();
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
    
    odom[0] = 100;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //床清掃
    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = - motion_planner.getArmBasePositionZ() + 20;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    odom[0] = -50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    //アームをしまう
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    //???
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    
    //追加分1
    //====================

    //ちょっと下がる
    odom[0] = -200;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //フィールド左上（ゴミ箱）まで平行移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //追加分2
    //====================
    //ちょっと下がる
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 350;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド左下（ゴミ箱）まで後退移動
    odom[0] = -200;
    odom[1] = 350;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //フィールド左上（ゴミ箱）まで前進移動
    odom[0] = 450;
    odom[1] = 350;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);
    //====================


    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //====================

    //元の位置に戻る
    odom[0] = -600;
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    motion_planner.final();
}

//-----------------------------------------------------------------------------------------------------------------------------

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

    //フィールド右上まで前進
    odom[0] = 700;
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
    //odom[1] = 50;
    odom[1] = 150;
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

    //ちょっと下がる
    odom[0] = 100;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //トイレ清掃開始
    motion_planner.endEffectorOn();

    //便座清掃
    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 440 - motion_planner.getArmBasePositionZ();
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
    
    odom[0] = 100;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //床清掃
    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = - motion_planner.getArmBasePositionZ() + 20;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    odom[0] = -50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    //アームをしまう
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    //???
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //追加分
    //====================

    //ちょっと下がる
    odom[0] = -250;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //フィールド左上（ゴミ箱）まで平行移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);


    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //====================

    //元の位置に戻る
    odom[0] = -620;
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    motion_planner.final();


    motion_planner.final();
}

void motion2U()
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
    //odom[1] = 350; 変更 motion2
    odom[1] = 500;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //========= 追加分
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 400;
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 430;
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = 150;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 580;
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = -250;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = 500;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY() - 100;
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    odom[0] = 600;
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);
    //=========

    //フィールド中央（トイレ前）まで移動
    odom[0] = motion_planner.getWheelOdomX();
    //odom[1] = 50;
    odom[1] = 50;
    odom[2] = M_PI;
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

    //ちょっと下がる
    odom[0] = 200;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //トイレ清掃開始
    motion_planner.endEffectorOn();

    //便座清掃
    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 435 - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 60;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -120;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 60;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //便座奥清掃
    pose[0] = 50;
    pose[1] = 0;
    pose[2] = 20;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //便座中清掃
    motion_planner.waitForGoal(10);
    
    pose[0] = -20;
    pose[1] = 0;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //アームを手前に戻す
    pose[0] = -30;
    pose[1] = 0;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = motion_planner.getArmPoseZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);
    
    odom[0] = 100;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //床清掃
    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = - motion_planner.getArmBasePositionZ() + 20;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    //近づく
    odom[0] = -80;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //左右
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //アームをちょっと前に出す
    pose[0] = 30;
    pose[1] = 0;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //トイレ側面の掃除
    /**
    pose[0] = 0;
    pose[1] = 0;
    pose[2] = 150;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = -20;
    pose[1] = 0;
    pose[2] = 150;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = -30 + 20;
    pose[1] = 0;
    pose[2] = -300;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    **/

    //遠ざかる
    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //左右
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
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

    //アームをしまう
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    //???
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //元の位置に戻る
    odom[0] = -630;
    odom[1] = -385;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    motion_planner.final();
}

//-----------------------------------------------------------------------------------------------------------------------------

void motion3()
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
    //odom[1] = 50;
    odom[1] = 50;
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

    //ちょっと下がる
    odom[0] = 200;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //トイレ清掃開始
    motion_planner.endEffectorOn();

    //便座清掃
    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 440 - motion_planner.getArmBasePositionZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 80;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -160;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 80;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //便座奥清掃
    pose[0] = 20;
    pose[1] = 0;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);
    
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //アームを手前に戻す
    pose[0] = -20;
    pose[1] = 0;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = motion_planner.getArmPoseZ();
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);
    
    odom[0] = 100;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //床清掃
    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = - motion_planner.getArmBasePositionZ() + 20;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    //近づく
    odom[0] = -50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //左右
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //アームをちょっと前に出す
    pose[0] = 30;
    pose[1] = 0;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //トイレ側面の掃除
    pose[0] = -50;
    pose[1] = 0;
    pose[2] = 200;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = -50;
    pose[1] = 0;
    pose[2] = 170;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = -30 + 100;
    pose[1] = 0;
    pose[2] = -370;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    //遠ざかる
    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //左右
    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = -200;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
    motion_planner.waitForGoal(10);

    pose[0] = 0;
    pose[1] = 100;
    pose[2] = 0;
    motion_planner.moveArmRelative(pose);
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

    //アームをしまう
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    //???
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //元の位置に戻る
    odom[0] = -620;
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    motion_planner.final();
}

//-----------------------------------------------------------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------------------------------------------------------

void turningMotion()
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

    usleep(500000);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI_2;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    usleep(500000);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = M_PI;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    usleep(500000);

    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = motion_planner.getWheelOdomY();
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);
}




// 7/11日　最高結果
/**
void motion3()
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
    //odom[1] = 50;
    odom[1] = 50;
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

    //ちょっと下がる
    odom[0] = 200;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //トイレ清掃開始
    motion_planner.endEffectorOn();

    //便座清掃
    pose[0] = motion_planner.getArmPoseX() + 50;
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = 440 - motion_planner.getArmBasePositionZ();
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
    
    odom[0] = 100;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

    //床清掃
    pose[0] = motion_planner.getArmPoseX();
    pose[1] = motion_planner.getArmPoseY();
    pose[2] = - motion_planner.getArmBasePositionZ() + 20;
    motion_planner.moveArmAbsolute(pose);
    motion_planner.waitForGoal(10);

    odom[0] = -50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    odom[0] = 50;
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumRelative(odom);
    motion_planner.waitForGoal(20);

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

    //アームをしまう
    motion_planner.moveArmInitPos1();
    motion_planner.waitForGoal(10);
    usleep(500000);
    motion_planner.moveArmInitPos2();
    motion_planner.waitForGoal(10);



    //???
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = 0;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //右端へ移動
    odom[0] = motion_planner.getWheelOdomX();
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    //元の位置に戻る
    odom[0] = -620;
    odom[1] = -400;
    odom[2] = 0;
    motion_planner.moveMecanumAbsolute(odom);
    motion_planner.waitForGoal(20);

    motion_planner.final();
}
*/