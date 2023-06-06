#pragma once

#include <math.h>

class Mecanum
{
    /* Mecanum
    robot:  l1: Axis1Length
            l2: Axis2Length
            -|##2##|          |##1##|
            ^   ##################              y
        l1  ¦   ##################              ^
            ¦   ##################   front      ¦
            v   ##################              ¦
            -|##3##|          |##0##|          -¦-----> x
                |       l2     |
                |<------------>|
*/
    public:
    Mecanum();
    Mecanum(float radius, float depth, float tread);
    ~Mecanum();
    void setYawAngle(float yaw);
    void calcInvese(float vx, float vy, float omega);
    void calcForward(float omega0, float omega1, float omega2, float omega3);
    void getVelocity(float &vx, float &vy, float &omega);
    void getRAD(float &omega0, float &omega1, float &omega2, float &omega3);
    void getRPM(float &omega0, float &omega1, float &omega2, float &omega3);

    private:
    float RADIUS_;
    float DEPTH_;
    float TREAD_;
    float robot_yaw_;

    float saved_vx_;
    float saved_vy_;
    float saved_omega_;
    
    float saved_omega0_rad_;
    float saved_omega1_rad_;
    float saved_omega2_rad_;
    float saved_omega3_rad_;
    float saved_omega0_rpm_;
    float saved_omega1_rpm_;
    float saved_omega2_rpm_;
    float saved_omega3_rpm_;
};