#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
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

class CraneX7
{
    private:
    int link_num;
    Eigen::Vector3d init_link_vecs[8];
    Eigen::Vector3d init_axis_vecs[7];
    Eigen::Vector3d link_vecs[8];
    Eigen::Vector3d axis_vecs[7];

    Eigen::Vector3d pos_vec;
    Eigen::Vector3d ori_vec;
    Eigen::Vector3d angle_axis_vec;
    Eigen::Vector3d pos_error_vec;
    Eigen::Vector3d ori_error_vec;
    Eigen::Matrix3d rotation_mats[7];
    Eigen::MatrixXd jacobian;
    
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CraneX7();
    ~CraneX7();
    void setAngles();
    void calcJacobian();
    void calcForward();
    void calcInvese();
    void getAngles();
    void getXYZ();
};

CraneX7::CraneX7(const double (&links)[], const int num)
{
    link_num = num;
    for (int i = 0; i < link_num; i++)
    {
        init_link_vecs[i] = Eigen::Vector3d(0, 0, links[i]);
        if (i % 2 == 0)
        {
            init_axis_vecs[i] =  = Eigen::Vector3d(0, 0, 1);
        }
        else
        [
            init_axis_vecs[i] =  = Eigen::Vector3d(0, 1, 0);
        ]
    }
    jacobian = Eigen::MatrixXd(6, link_num);
}

CraneX7::~CraneX7()
{}

void CraneX7::setAngles(const double (&angles)[])
{
    for (int i = 0; i < 7; i++)
    {
        if (i % 2 == 0)
        {
            rotation_mats[i] = Eigen::AngleAxisd(angles[i], Eigen::Vector3d(0, 0, 1));
        }
        else
        [
            rotation_mats[i] = Eigen::AngleAxisd(angles[i], Eigen::Vector3d(0, 1, 0));
        ]
        Eigen::Matrix3d rotation_mat;
        rotation_mat = MatrixXd::Identity(3,3);
        for (int j = 0; j < i; j++)
        {
            rotation_mat = rotation_mat * rotation_mats[j];
        }
        link_vecs[i] = rotation_mat * init_link_vecs[i];
        axis_vecs[i] = rotation_mat * init_axis_vecs[i];
    }
}

void CraneX7::calcJacobian()
{
    for (int i = 0; i < 7; i++)
    {
        Eigen::Vector3d upper_vec = axis_vecs[i].cross(pos_vec - link_vecs[i]);
        for (int vec_i = 0; vec_i < 3; vec_i++)
        {
            this->jacobian(i, vec_i) = upper_vec(vec_i);
        }
        for (int vec_i = 0; vec_i < 3; vec_i++)
        {
            this->jacobian(i, vec_i + 3) = axis_vecs[i](vec_i);
        }
    }
}

void CraneX7::calcAngleAxis()
{
    angle_axis_vec << 0, 0, 0;
    Eigen::Matrix3d orientation_mat;
    Eigen::Vector3d angle_axis_vec;
    Eigen::Vector3d extended_angle_axis_vec;
    Eigen::Vector3d rot_vec;
    Eigen::Vector3d n_vec;
    rot_vec(0) = orientation_mat(2, 1) - orientation_mat(1, 2);
    rot_vec(1) = orientation_mat(0, 2) - orientation_mat(2, 0);
    rot_vec(2) = orientation_mat(1, 0) - orientation_mat(0, 1);
    double rot_vec_norm = rot_vec.norm();
    double orientation_trace = orientation_mat.trace();
    if (rot_vec_norm != 0)
    {
        double coefficient = atan2(rot_vec_norm, orientation_trace - 1.0) / rot_vec_norm;
        angle_axis_vec = coefficient * rot_vec;
    }
    else if (orientation_trace == 3)
    {
        angle_axis_vec = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (orientation_trace == -1)
    {
        angle_axis_vec = Eigen::Vector3d(0.0, 0.0, 0.0);
        if (orientation_mat(0, 1) >= 0.0 && orientation_mat(1, 2) >= 0.0 && orientation_mat(0, 2) >= 0.0)
        {
            n_vec(0) = sqrt((orientation_mat(0, 0) + 1.0) * 0.5);
            n_vec(1) = sqrt((orientation_mat(1, 1) + 1.0) * 0.5);
            n_vec(2) = sqrt((orientation_mat(2, 2) + 1.0) * 0.5);
        }
        else if (orientation_mat(0, 1) >= 0.0 && orientation_mat(1, 2) <  0.0 && orientation_mat(0, 2) <= 0.0)
        {
            n_vec(0) =  sqrt((orientation_mat(0, 0) + 1.0) * 0.5);
            n_vec(1) =  sqrt((orientation_mat(1, 1) + 1.0) * 0.5);
            n_vec(2) = -sqrt((orientation_mat(2, 2) + 1.0) * 0.5);
        }
        else if (orientation_mat(0, 1) <  0.0 && orientation_mat(1, 2) <= 0.0 && orientation_mat(0, 2) >= 0.0)
        {
            n_vec(0) =  sqrt((orientation_mat(0, 0) + 1.0) * 0.5);
            n_vec(1) = -sqrt((orientation_mat(1, 1) + 1.0) * 0.5);
            n_vec(2) =  sqrt((orientation_mat(2, 2) + 1.0) * 0.5);
        }
        else if (orientation_mat(0, 1) <= 0.0 && orientation_mat(1, 2) >= 0.0 && orientation_mat(0, 2) < 0.0)
        {
            n_vec(0) = -sqrt((orientation_mat(0, 0) + 1.0) * 0.5);
            n_vec(1) =  sqrt((orientation_mat(1, 1) + 1.0) * 0.5);
            n_vec(2) =  sqrt((orientation_mat(2, 2) + 1.0) * 0.5);
        }
        else
        {
            printf("n_vec error\n");
        }
        angle_axis_vec = M_PI * n_vec;
    }
    else
    {
        printf("orientation_trace error\n");
    }

    /**
    double angle_axis_vec_norm = angle_axis_vec.norm();
    double sign1 = (angle_axis_vec(0) > 0) - (angle_axis_vec(0) < 0);
    double sign2 = (angle_axis_vec(1) > 0) - (angle_axis_vec(1) < 0);
    double sign3 = (angle_axis_vec(2) > 0) - (angle_axis_vec(2) < 0);
    double theta1 = sign1 * dabs(angle_axis_vec(0));
    double theta2 = sign2 * dabs(angle_axis_vec(1));
    double theta3 = sign3 * dabs(angle_axis_vec(2));
    double abs_n_vec1 = dabs(angle_axis_vecc(0));
    double abs_n_vec2 = dabs(angle_axis_vecc(1));
    double abs_n_vec3 = dabs(angle_axis_vecc(2));
    if (angle_axis_vec_norm == 0.0)
    {
        abs_n_vec1 = 1.0;
        abs_n_vec2 = 1.0;
        abs_n_vec3 = 1.0;
    }
    else
    {
        abs_n_vec1 / angle_axis_vec_norm;
        abs_n_vec2 / angle_axis_vec_norm;
        abs_n_vec3 / angle_axis_vec_norm;
    }

    extended_angle_axis_vec = Eigen::Vector3d(0.0, 0.0, 0.0);
    **/
}

void calcErrorVec()
{
    this->calcForward();
    this->calcAngleAxis();
}

void CraneX7::calcForward()
{
    this->pos_vec << 0, 0, 0;
    for (int i = 0; i < 7; i++)
    {
        this->pos_vec += link_vecs[i];
    }
}

void CraneX7::calcInvese(const double x, const double y, const double z, const Eigen::Matrix3d& R)
{
    double old_energy = this->error_vec.transpose() * k_mat * this->error_vec * 0.5;
    double new_energy = 0.0;
    Eigen::Vector3d torque_vec;
    Eigen::Vector3d delta_q;
    Eigen::Vector3d q;
    Eigen::Matrix3d k_mat;
    Eigen::Matrix3d d_mat;
    double delta;
    setAngles(const double (&angles)[]);
    calcJacobian();
    calcErrorVec();
    new_energy = this->error_vec.transpose() * k_mat * this->error_vec * 0.5;
    while ((1.0 - 1e12) * old_energy <= new_energy)
    {
        old_energy = new_energy;
        torque_vec = this->jacobian.transpose() * k_mat * this->error_vec;
        d_mat = this->jacobian.transpose() * k_mat * this->jacobian + (torque_vec.norm() * 0.5 + delta) * MatrixXd::Identity(3, 3);
        Eigen::LLT<Eigen::Matrix3d> CholD(d_mat);
        Eigen::Matrix3d d_mat_inv= CholD.matrixL();
        delta_q = d_mat_inv * this->jacobian.transpose() * k_mat * this->error_vec;
        q = q + delta_q;

        setAngles(const double (&angles)[]);
        calcJacobian();
        calcErrorVec();
        new_energy = this->error_vec.transpose() * k_mat * this->error_vec * 0.5;
    }
    pos_vec(0) = q(0);
    pos_vec(1) = q(1);
    pos_vec(2) = q(2);
    ori_vec(0) = q(3);
    ori_vec(1) = q(4);
    ori_vec(2) = q(5);
}

void CraneX7::getAngles(double (&angles)[])
{
}

void CraneX7::getXYZ(double &x, double &y, double &z)
{
    x = pos_vec(0);
    y = pos_vec(1);
    z = pos_vec(2);
}