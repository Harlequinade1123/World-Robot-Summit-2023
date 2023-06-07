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