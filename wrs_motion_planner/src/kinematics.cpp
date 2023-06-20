#include "kinematics.h"

Mecanum::Mecanum()
{}

Mecanum::Mecanum(float radius, float depth, float tread)
    : RADIUS_(radius), DEPTH_(depth), TREAD_(tread)
{
    //100/2
    //250
    //270
    this->robot_yaw_        = 0;
    this->saved_vx_         = 0;
    this->saved_vy_         = 0;
    this->saved_omega_      = 0;
    this->saved_omega0_rad_ = 0;
    this->saved_omega1_rad_ = 0;
    this->saved_omega2_rad_ = 0;
    this->saved_omega3_rad_ = 0;
    this->saved_omega0_rpm_ = 0;
    this->saved_omega1_rpm_ = 0;
    this->saved_omega2_rpm_ = 0;
    this->saved_omega3_rpm_ = 0;
}

Mecanum::~Mecanum()
{}

void Mecanum::setYawAngle(float yaw)
{
    this->robot_yaw_ = yaw;
}

void Mecanum::calcInvese(float vx, float vy, float omega)
{
    float sin_yaw = sin(this->robot_yaw_);
    float cos_yaw = cos(this->robot_yaw_);
    this->saved_omega0_rad_ = (cos_yaw - sin_yaw) * vx + (sin_yaw + cos_yaw) * vy + (this->DEPTH_ + this->TREAD_) * 0.5 * omega;
    this->saved_omega1_rad_ = (cos_yaw + sin_yaw) * vx + (sin_yaw - cos_yaw) * vy - (this->DEPTH_ + this->TREAD_) * 0.5 * omega;
    this->saved_omega2_rad_ = (cos_yaw - sin_yaw) * vx + (sin_yaw + cos_yaw) * vy - (this->DEPTH_ + this->TREAD_) * 0.5 * omega;
    this->saved_omega3_rad_ = (cos_yaw + sin_yaw) * vx + (sin_yaw - cos_yaw) * vy + (this->DEPTH_ + this->TREAD_) * 0.5 * omega;
    this->saved_omega0_rad_ /= this->RADIUS_;
    this->saved_omega1_rad_ /= this->RADIUS_;
    this->saved_omega2_rad_ /= this->RADIUS_;
    this->saved_omega3_rad_ /= this->RADIUS_;
    this->saved_omega0_rpm_ = this->saved_omega0_rad_ * 30.0 / M_PI;
    this->saved_omega1_rpm_ = this->saved_omega1_rad_ * 30.0 / M_PI;
    this->saved_omega2_rpm_ = this->saved_omega2_rad_ * 30.0 / M_PI;
    this->saved_omega3_rpm_ = this->saved_omega3_rad_ * 30.0 / M_PI;
}

void Mecanum::calcForward(float omega0, float omega1, float omega2, float omega3)
{
    float sin_yaw = sin(this->robot_yaw_);
    float cos_yaw = cos(this->robot_yaw_);
    this->saved_vx_ = this->RADIUS_ * 0.25 * ((cos_yaw - sin_yaw) * omega0 +
                                              (cos_yaw + sin_yaw) * omega1 +
                                              (cos_yaw - sin_yaw) * omega2 +
                                              (cos_yaw + sin_yaw) * omega3);
    this->saved_vy_ = this->RADIUS_ * 0.25 * ((sin_yaw + cos_yaw) * omega0 +
                                              (sin_yaw - cos_yaw) * omega1 +
                                              (sin_yaw + cos_yaw) * omega2 +
                                              (sin_yaw - cos_yaw) * omega3);
    this->saved_omega_ = this->RADIUS_ * 2.0 * (this->DEPTH_ + this->TREAD_) * (omega0 - omega1 - omega2 + omega3);
}

void Mecanum::getVelocity(float &vx, float &vy, float &omega)
{
    vx    = this->saved_vx_;
    vy    = this->saved_vy_;
    omega = this->saved_omega_;
}

void Mecanum::getRAD(float &omega0, float &omega1, float &omega2, float &omega3)
{
    omega0 = this->saved_omega0_rad_;
    omega1 = this->saved_omega1_rad_;
    omega2 = this->saved_omega2_rad_;
    omega3 = this->saved_omega3_rad_;
}

void Mecanum::getRPM(float &omega0, float &omega1, float &omega2, float &omega3)
{
    omega0 = this->saved_omega0_rpm_;
    omega1 = this->saved_omega1_rpm_;
    omega2 = this->saved_omega2_rpm_;
    omega3 = this->saved_omega3_rpm_;
}


CraneX7::CraneX7()
{}

CraneX7::CraneX7(const float links[8], const int link_num, const int axis_num)
{
    this->LINK_NUM_ = link_num;
    this->AXIS_NUM_ = axis_num;
    double L = 0.0;
    for (int i = 0; i < LINK_NUM_; i++)
    {
        this->INIT_LINK_VECS_[i] = Eigen::Vector3d(0, 0, links[i]);
        L += links[i];
        if (i % 2 == 0)
        {
            this->INIT_AXIS_VECS_[i] = Eigen::Vector3d(0, 0, 1);
        }
        else
        {
            this->INIT_AXIS_VECS_[i] = Eigen::Vector3d(0, 1, 0);
        }
    }
    this->jacobian_ = Eigen::MatrixXd(6, this->AXIS_NUM_);
    this->k_mat_ = Eigen::MatrixXd(6, 6);
    this->k_mat_ << 1, 0, 0,  0,  0,  0,
                    0, 1, 0,  0,  0,  0,
                    0, 0, 1,  0,  0,  0,
                    0, 0, 0, L*L, 0,  0,
                    0, 0, 0,  0, L*L, 0,
                    0, 0, 0,  0,  0, L*L;
    this->DELTA_ = 0.0001 * (std::max(this->AXIS_NUM_, 6) - 5) * L * L;
    Eigen::VectorXd init_vec = Eigen::VectorXd::Zero(axis_num);
    this->calcForward(init_vec);
    this->error_vec_ = Eigen::VectorXd::Zero(6);
}

CraneX7::~CraneX7()
{}

void CraneX7::calcForward(Eigen::VectorXd &angle_vec)
{
    this->pos_vec_FK_ << 0, 0, 0;
    this->orientation_mat_FK_ <<1, 0, 0,
                                0, 1, 0,
                                0, 0, 1;
    Eigen::Matrix3d rotation_mat;
    this->link_vecs_[0] = INIT_LINK_VECS_[0];
    this->pos_vec_FK_ += link_vecs_[0];
    for (int i = 0; i < this->AXIS_NUM_; i++)
    {
        if (i % 2 == 0)
        {
            rotation_mat = Eigen::AngleAxisd(angle_vec(i), Eigen::Vector3d(0, 0, 1));
        }
        else
        {
            rotation_mat = Eigen::AngleAxisd(angle_vec(i), Eigen::Vector3d(0, 1, 0));
        }
        this->orientation_mat_FK_ = this->orientation_mat_FK_ * rotation_mat;
        link_vecs_[i + 1] = this->orientation_mat_FK_ * INIT_LINK_VECS_[i + 1];
        axis_vecs_[i] = this->orientation_mat_FK_ * INIT_AXIS_VECS_[i];
        this->pos_vec_FK_ += link_vecs_[i];
    }
}

void CraneX7::getXYZ(double &x, double &y, double &z)
{
    x = pos_vec_FK_(0);
    y = pos_vec_FK_(1);
    z = pos_vec_FK_(2);
}

void CraneX7::getR(Eigen::Matrix3d& R)
{
    R = orientation_mat_FK_;
}


void CraneX7::calcJacobian()
{
    Eigen::Vector3d pos_vec;
    pos_vec << 0.0, 0.0, 0.0;
    Eigen::Vector3d upper_vec;
    for (int i = 0; i < this->AXIS_NUM_; i++)
    {
        pos_vec  += this->link_vecs_[i];
        upper_vec = this->axis_vecs_[i].cross(pos_vec_FK_ - pos_vec);
        for (int vec_i = 0; vec_i < 3; vec_i++)
        {
            this->jacobian_(vec_i, i) = upper_vec(vec_i);
        }
        for (int vec_i = 0; vec_i < 3; vec_i++)
        {
            this->jacobian_(vec_i + 3, i) = this->axis_vecs_[i](vec_i);
        }
    }
}

void CraneX7::calcErrorVec(const Eigen::Vector3d& pos_vec, const Eigen::Matrix3d& R)
{
    Eigen::Vector3d pos_error_vec = pos_vec - this->pos_vec_FK_;
    Eigen::Matrix3d error_mat     = R * this->orientation_mat_FK_.transpose();
    this->calcAngleAxis(error_mat);
    this->error_vec_(0) = pos_error_vec(0);
    this->error_vec_(1) = pos_error_vec(1);
    this->error_vec_(2) = pos_error_vec(2);
    this->error_vec_(3) = this->angle_axis_vec_(0);
    this->error_vec_(4) = this->angle_axis_vec_(1);
    this->error_vec_(5) = this->angle_axis_vec_(2);
}

void CraneX7::calcAngleAxis(Eigen::Matrix3d &orientation_mat)
{
    this->angle_axis_vec_ << 0, 0, 0;
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
        this->angle_axis_vec_ = coefficient * rot_vec;
    }
    else if (orientation_trace == 3)
    {
        this->angle_axis_vec_ << 0.0, 0.0, 0.0;
    }
    else if (orientation_trace == -1)
    {
        this->angle_axis_vec_ = Eigen::Vector3d(0.0, 0.0, 0.0);
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
        this->angle_axis_vec_ = M_PI * n_vec;
    }
    else
    {
        printf("orientation_trace error\n");
    }
}

void CraneX7::calcInvese(Eigen::VectorXd& q, const Eigen::Vector3d& pos_vec, const Eigen::Matrix3d& R)
{
    double old_energy = 0.0;
    double new_energy = 0.0;
    Eigen::VectorXd delta_q(this->AXIS_NUM_);
    Eigen::MatrixXd d_mat;
    calcForward(q);
    calcJacobian();
    calcErrorVec(pos_vec, R);
    new_energy = this->error_vec_.dot(this->k_mat_ * this->error_vec_) * 0.5;
    old_energy = new_energy * 2;
    while ((1.0 - 0.001) * old_energy > new_energy)
    {
        old_energy = new_energy;
        Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(this->AXIS_NUM_, this->AXIS_NUM_);
        d_mat = this->jacobian_.transpose() * this->k_mat_ * this->jacobian_ + (old_energy * 0.5 + this->DELTA_) * eye;
        delta_q = d_mat.inverse() * this->jacobian_.transpose() * this->k_mat_ * this->error_vec_;
        q += delta_q;
        calcForward(q);
        calcJacobian();
        calcErrorVec(pos_vec, R);
        new_energy = this->error_vec_.dot(this->k_mat_ * this->error_vec_) * 0.5;
    }
}