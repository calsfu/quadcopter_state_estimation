#include "CExtendedKalmanFilter.h"

State CExtendedKalmanFilter::f(const Eigen::Vector3d& krAccel, const Eigen::Vector3d& krGyro)
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
    State x_predict = f(krAccel, krGyro);

    // Predict covariance
    manif::SE_2_3d::Jacobian F, J_o_dx;
    X = X.rplus(manif::SE_2_3Tangentd(u), F, J_o_dx);

    // Make sure state is on manifold
    X.normalize();

    
}

void CExtendedKalmanFilter::update(Eigen::Vector3d z) {
    // jacobian
    Eigen::Matrix<double, 3, 10> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    // kalman gain
    Eigen::Matrix<double, 10, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // update state estimate
    x = x + K * (z - H * x);

    // update covariance
    P = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P;
}