#pragma once

#include <Eigen/Dense>
#include <manif/SE_2_3.h>

using Vector10d = Eigen::Matrix<double, 10, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;

class CExtendedKalmanFilter {
public:
    /// @brief    Constructor
    /// @param x0 Initial state
    /// @param P0 Initial covariance
    /// @param Q  Input noise covariance
    /// @param R  Measurement noise covariance
    /// @param dt Time step
    CExtendedKalmanFilter(Vector10d x0, Matrix12d P0, Matrix12d Q, Matrix12d R, double dt) 
        : x(x0), P(P0), Q(Q), R(R), dt(dt) {}
    
    Vector10d get_state() { return x; }

    /// @brief Computes the state transition function
    /// @param omega Angular velocity
    /// @param accel Acceleration
    /// @return Predicted state
    Vector10d f(const Eigen::Vector3d& krAccel, const Eigen::Vector3d& krGyro);

    /// @brief Predict the state estimate
    /// @param u Control input
    void predict(Eigen::Vector4d u, const Eigen::Vector3d& krAccel, const Eigen::Vector3d& krGyro);

    /// @brief Update the state estimate
    /// @param z Measurement
    void update(Eigen::Vector4d z);

    Vector10d x; //< State of [pos, vel, quat]
    Matrix12d P; //< Covariance
    Matrix12d Q; //< Process noise covariance
    Matrix12d R; //< Measurement noise covariance
    double dt;         //< Time step
};