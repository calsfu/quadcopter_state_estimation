#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Compass.hpp>
#include <iostream>
#include <memory>
#include "simulation.h"
#include <random>

using namespace webots;

int main() {
  SimQuadcopter quad;

//   Controller ctrl;
  double x, y, z, yaw = 0.0;
  double dx, dy, dz = 0.0;

  while (true) {
    quad.step_sim();

    // robot teleop
    quad.keyboard_ctrl();

    auto omega = quad.get_gyro();
    auto a = quad.get_accel();
    auto gps_pos = quad.get_pos();
    auto gps_vel = quad.get_vel();
    auto mag = quad.get_mag();

    auto cheater_pos = quad.get_pos_true();
    auto cheater_vel = quad.get_vel_true();
    auto cheater_rot = quad.get_rot_true();

    std::cout << "Time: " << quad.get_time() << std::endl;
    std::cout << "Omega: " << omega.transpose() << std::endl;
    std::cout << "Accel: " << a.transpose() << std::endl;
    std::cout << "GPS Pos: " << gps_pos.transpose() << std::endl;
    std::cout << "GPS Vel: " << gps_vel.transpose() << std::endl;
    std::cout << "Mag: " << mag.transpose() << std::endl;
    std::cout << "Cheater Pos: " << cheater_pos.transpose() << std::endl;
    std::cout << "Cheater Vel: " << cheater_vel.transpose() << std::endl;
    std::cout << "Cheater Rot: " << cheater_rot.coeffs().transpose() << std::endl;
  }

  return 0;
}