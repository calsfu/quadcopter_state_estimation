#include "CExtendedKalmanFilter.h"

Vector10d CExtendedKalmanFilter::f(const Eigen::Vector3d& krAccel, const Eigen::Vector3d& krGyro)
{
    Vector10d xdot;

    // Velocity using accel
    xdot.block<3, 1>(3, 0) << x.block<3, 1>(3, 0) + krAccel * dt;

    // Position using velocity
    xdot.block<3, 1>(0, 0) << x.block<3, 1>(0, 0) + x.block<3, 1>(3, 0) * dt;

    // Quaternion using gyro
    Eigen::Quaterniond quat = Eigen::Quaterniond(x(6), x(7), x(8), x(9));
    quat = quat * Eigen::Quaterniond(Eigen::AngleAxisd(krGyro(0) * dt, Eigen::Vector3d::UnitX()));
    quat = quat * Eigen::Quaterniond(Eigen::AngleAxisd(krGyro(1) * dt, Eigen::Vector3d::UnitY()));
    quat = quat * Eigen::Quaterniond(Eigen::AngleAxisd(krGyro(2) * dt, Eigen::Vector3d::UnitZ()));
    xdot.block<4, 1>(6, 0) << quat.w(), quat.x(), quat.y(), quat.z();
    
}

void CExtendedKalmanFilter::predict(Eigen::Vector4d u, const Eigen::Vector3d& krAccel, const Eigen::Vector3d& krGyro) {
    // Predict state
    Vector10d x_predict = f(krAccel, krGyro);

    // Predict covariance
    Eigen::Matrix3d F;
    
}

void CExtendedKalmanFilter::update(Eigen::Vector4d z) {
    // Compute innovation

    // Compute innovation covariance

    // Compute Kalman gain

    // Update state

    // Update covariance
}