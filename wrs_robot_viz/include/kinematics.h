#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <iostream>
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
    void calcInverse(float vx, float vy, float omega);
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

class CraneX7
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CraneX7();
    CraneX7(const float links[8], const int link_num, const int axis_num);
    ~CraneX7();
    void calcForward(Eigen::VectorXd &angle_vec);
    void getXYZ(double &x, double &y, double &z);
    void getR(Eigen::Matrix3d& R);
    void calcJacobian();
    void calcErrorVec(const Eigen::Vector3d& pos_vec, const Eigen::Matrix3d& R);
    void calcAngleAxis(Eigen::Matrix3d &orientation_mat);
    void calcInverse(Eigen::VectorXd& q, const Eigen::Vector3d& pos_vec, const Eigen::Matrix3d& R);

    private:
    int LINK_NUM_;
    int AXIS_NUM_;
    double DELTA_;
    Eigen::MatrixXd k_mat_;
    Eigen::Vector3d INIT_LINK_VECS_[8];
    Eigen::Vector3d INIT_AXIS_VECS_[7];
    Eigen::Vector3d link_vecs_[8];
    Eigen::Vector3d axis_vecs_[7];
    Eigen::Vector3d pos_vec_FK_;
    Eigen::Matrix3d orientation_mat_FK_;
    Eigen::MatrixXd jacobian_;
    Eigen::Vector3d angle_axis_vec_;
    Eigen::VectorXd error_vec_;
};