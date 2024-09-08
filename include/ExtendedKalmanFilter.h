#pragma once

#include <Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(
        const Eigen::Matrix3d& procNoiseMat,
        const Eigen::Matrix3d& obvNoiseMatAccel,
        const Eigen::Matrix3d& obvNoiseMatGPS,
        const Eigen::Matrix3d& obvNoiseMatMag,
        double dt
    );
    void Predict();
    void Update();
private:
    void updateGps(const Eigen::Vector3d& orPos, const Eigen::Vector3d& orVel);
    void updateImu(const Eigen::Vector3d& orImu);
    void updateMag(const Eigen::Vector3d& orMag);
    
};