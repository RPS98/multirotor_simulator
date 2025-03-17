/*!*******************************************************************************************
 *  \file       multirotor_simulator_traj_gen_test.cpp
 *  \brief      Class test
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"
#include "multirotor_simulator.hpp"

namespace multirotor {

using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
using DynamicWaypoint   = dynamic_traj_generator::DynamicWaypoint;

#define SIM_CONFIG_PATH "examples/simulation_config.yaml"
double floor_height = 0.0;

void quaternion_to_Euler(const Eigen::Quaterniond& _quaternion,
                         double& roll,
                         double& pitch,
                         double& yaw) {
  // roll (x-axis rotation)
  Eigen::Quaterniond quaternion = _quaternion.normalized();

  double sinr = 2.0 * (quaternion.w() * quaternion.x() + quaternion.y() * quaternion.z());
  double cosr = 1.0 - 2.0 * (quaternion.x() * quaternion.x() + quaternion.y() * quaternion.y());
  roll        = std::atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = 2.0 * (quaternion.w() * quaternion.y() - quaternion.z() * quaternion.x());
  if (std::abs(sinp) >= 1.0)
    pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny = 2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y());
  double cosy = 1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z());
  yaw         = std::atan2(siny, cosy);
}

class CsvLogger {
public:
  explicit CsvLogger(const std::string& file_name) : file_name_(file_name) {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time,x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,ax,ay,az,dwx,dwy,dwz,"
             "fx,fy,fz,tx,ty,tz,mw1,mw2,mw3,mw4,mdw1,mdw2,mdw3,mdw4,x_ref,y_ref,"
             "z_ref,yaw_ref,vx_ref,vy_ref,vz_ref,wx_ref,wy_ref,"
             "wz_ref,ax_ref,ay_ref,az_ref,dwx_ref,dwy_ref,dwz_ref,fx_ref,fy_ref,fz_ref,"
             "tx_ref,ty_ref,tz_ref,mw1_ref,mw2_ref,mw3_ref,mw4_ref,"
             "x_io,y_io,z_io,roll_io,pitch_io,yaw_io,vx_io,vy_io,vz_io,wx_io,wy_io,"
             "wz_io,ax_io,ay_io,az_io"
          << std::endl;
  }

  ~CsvLogger() { file_.close(); }

  void add_double(const double data) {
    // Check if data is nan
    if (std::isnan(data)) {
      // Throw exception
      std::invalid_argument("Data is nan");
      return;
    }
    file_ << data << ",";
  }

  void add_string(const std::string& data, const bool add_final_comma = true) {
    file_ << data;
    if (add_final_comma) file_ << ",";
  }

  void add_vector_row(const Eigen::Vector3d& data, const bool add_final_comma = true) {
    for (size_t i = 0; i < data.size(); ++i) {
      // Check if data is nan
      if (std::isnan(data[i])) {
        // Throw exception
        std::invalid_argument("Data is nan");
        return;
      }
      file_ << data[i];
      if (i < data.size() - 1) {
        file_ << ",";
      } else if (add_final_comma) {
        file_ << ",";
      }
    }
  }

  void add_vector_row(const Eigen::Vector4d& data, const bool add_final_comma = true) {
    for (size_t i = 0; i < data.size(); ++i) {
      // Check if data is nan
      if (std::isnan(data[i])) {
        // Throw exception
        std::invalid_argument("Data is nan");
        return;
      }
      file_ << data[i];
      if (i < data.size() - 1) {
        file_ << ",";
      } else if (add_final_comma) {
        file_ << ",";
      }
    }
  }

  void save(const double time, const Simulator<double, 4>& simulator) {
    add_double(time);

    // State ground truth
    const state::State<double, 4> state = simulator.get_dynamics_const().get_state();
    std::ostringstream state_stream;
    state_stream << state;
    std::string state_string = state_stream.str();
    add_string(state_string);

    // References
    // Reference trajectory generator
    add_vector_row(simulator.get_reference_position());  // Position
    add_double(simulator.get_reference_yaw());           // Yaw
    add_vector_row(simulator.get_reference_velocity());  // Velocity
    // Reference multirotor_controller
    add_vector_row(
        simulator.get_controller_const().get_desired_angular_velocity());  // Angular velocity
    add_vector_row(simulator.get_reference_acceleration());                // Linear acceleration
    add_vector_row(simulator.get_controller_const()
                       .get_indi_controller_const()
                       .get_desired_angular_acceleration());  // Angular acceleration
    // Force reference compensated with the gravity and in earth frame
    Eigen::Vector3d force_ref =
        simulator.get_dynamics_const().get_state().kinematics.orientation *
            simulator.get_controller_const().get_indi_controller_const().get_desired_thrust() +
        simulator.get_dynamics_const().get_model_const().get_gravity() *
            simulator.get_dynamics_const().get_model_const().get_mass();
    add_vector_row(force_ref);  // Force
    add_vector_row(simulator.get_controller_const()
                       .get_indi_controller_const()
                       .get_desired_torque());  // Torque

    add_vector_row(simulator.get_actuation_motors_angular_velocity());  // Motor angular velocity

    // State inertial odometry
    std::ostringstream io_stream;
    io_stream << simulator.get_inertial_odometry_const();
    std::string io_string = io_stream.str();
    add_string(io_string, false);

    // End line
    file_ << std::endl;
  }

  void close() { file_.close(); }

private:
  std::string file_name_;
  std::ofstream file_;
};

bool check_file_exits(const std::string& file_path = SIM_CONFIG_PATH) {
  // Check if file exists
  std::ifstream f(file_path.c_str());
  if (!f.good()) {
    std::string absolute_simulation_config_path = std::filesystem::absolute(file_path).string();
    std::cout << "File " << absolute_simulation_config_path << " does not exist." << std::endl;
    f.close();
    return false;
  }
  f.close();
  return true;
}

template <typename T>
bool read_yaml_param(const std::string& param_name,
                     T& value,
                     const std::string& yaml_path = SIM_CONFIG_PATH) {
  // Read params
  if (!check_file_exits(yaml_path)) {
    return false;
  }
  YAML::Node yaml_config_file = YAML::LoadFile(yaml_path);

  std::istringstream iss(param_name);
  std::string token;
  while (std::getline(iss, token, '.')) {
    // Check if param exists
    if (!yaml_config_file[token]) {
      std::cout << "Param " << param_name << " does not exist in file " << yaml_path << std::endl;
      return false;
    }
    yaml_config_file = yaml_config_file[token];
  }
  // Read param
  value = yaml_config_file.as<T>();
  return true;
}

SimulatorParams<double, 4> get_simulation_params_from_yaml(
    const std::string& simulation_config_path = SIM_CONFIG_PATH) {
  // Initialize simulator params
  SimulatorParams p;

  // Read params
  YAML::Node yaml_config_file = YAML::LoadFile(simulation_config_path);

  // Simulation parameters
  floor_height = yaml_config_file["sim_config"]["floor_height"].as<double>();

  // Dynamics state
  auto yml = yaml_config_file["dynamics"]["state"];

  p.dynamics_params.state.kinematics.position =
      Eigen::Vector3d(yml["position"][0].as<double>(), yml["position"][1].as<double>(),
                      yml["position"][2].as<double>());
  p.dynamics_params.state.kinematics.orientation =
      Eigen::Quaterniond(yml["orientation"][0].as<double>(), yml["orientation"][1].as<double>(),
                         yml["orientation"][2].as<double>(), yml["orientation"][3].as<double>());
  p.dynamics_params.state.kinematics.linear_velocity = Eigen::Vector3d(
      yml["linear_velocity"][0].as<double>(), yml["linear_velocity"][1].as<double>(),
      yml["linear_velocity"][2].as<double>());
  p.dynamics_params.state.kinematics.angular_velocity = Eigen::Vector3d(
      yml["angular_velocity"][0].as<double>(), yml["angular_velocity"][1].as<double>(),
      yml["angular_velocity"][2].as<double>());
  p.dynamics_params.state.kinematics.linear_acceleration = Eigen::Vector3d(
      yml["linear_acceleration"][0].as<double>(), yml["linear_acceleration"][1].as<double>(),
      yml["linear_acceleration"][2].as<double>());
  p.dynamics_params.state.kinematics.angular_acceleration = Eigen::Vector3d(
      yml["angular_acceleration"][0].as<double>(), yml["angular_acceleration"][1].as<double>(),
      yml["angular_acceleration"][2].as<double>());

  // Dynamics model
  yml = yaml_config_file["dynamics"]["model"];

  p.dynamics_params.model_params.gravity =
      Eigen::Vector3d(yml["gravity"][0].as<double>(), yml["gravity"][1].as<double>(),
                      yml["gravity"][2].as<double>());
  p.dynamics_params.model_params.vehicle_mass = yml["vehicle_mass"].as<double>();
  p.dynamics_params.model_params.vehicle_inertia =
      Eigen::Vector3d(yml["vehicle_inertia"][0].as<double>(),
                      yml["vehicle_inertia"][1].as<double>(),
                      yml["vehicle_inertia"][2].as<double>())
          .asDiagonal();
  p.dynamics_params.model_params.vehicle_drag_coefficient =
      yml["vehicle_drag_coefficient"].as<double>();
  p.dynamics_params.model_params.vehicle_aero_moment_coefficient =
      Eigen::Vector3d(yml["vehicle_aero_moment_coefficient"][0].as<double>(),
                      yml["vehicle_aero_moment_coefficient"][1].as<double>(),
                      yml["vehicle_aero_moment_coefficient"][2].as<double>())
          .asDiagonal();
  p.dynamics_params.model_params.force_process_noise_auto_correlation =
      yml["force_process_noise_auto_correlation"].as<double>();
  p.dynamics_params.model_params.moment_process_noise_auto_correlation =
      yml["moment_process_noise_auto_correlation"].as<double>();

  // Motors parameters
  yml = yaml_config_file["dynamics"]["model"]["motors_params"];

  double thrust_coefficient = yml["thrust_coefficient"].as<double>();
  double torque_coefficient = yml["torque_coefficient"].as<double>();
  double x_dist             = yml["x_dist"].as<double>();
  double y_dist             = yml["y_dist"].as<double>();
  double min_speed          = yml["min_speed"].as<double>();
  double max_speed          = yml["max_speed"].as<double>();
  double time_constant      = yml["time_constant"].as<double>();
  double rotational_inertia = yml["rotational_inertia"].as<double>();
  p.dynamics_params.model_params.motors_params =
      multirotor::model::Model<double, 4>::create_quadrotor_x_config(
          thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed,
          time_constant, rotational_inertia);

  // Controller params Indi
  yml = yaml_config_file["controller"]["indi"];

  p.controller_params.indi_controller_params.inertia =
      p.dynamics_params.model_params.vehicle_inertia;
  auto mixing_matrix_6D_4rotors = multirotor::model::Model<double, 4>::compute_mixer_matrix<6>(
      p.dynamics_params.model_params.motors_params);
  p.controller_params.indi_controller_params.mixer_matrix_inverse =
      indi_controller::compute_quadrotor_mixer_matrix_inverse(mixing_matrix_6D_4rotors);
  p.controller_params.indi_controller_params.pid_params.Kp_gains = Eigen::Vector3d(
      yml["Kp"][0].as<double>(), yml["Kp"][1].as<double>(), yml["Kp"][2].as<double>());
  p.controller_params.indi_controller_params.pid_params.Ki_gains = Eigen::Vector3d(
      yml["Ki"][0].as<double>(), yml["Ki"][1].as<double>(), yml["Ki"][2].as<double>());
  p.controller_params.indi_controller_params.pid_params.Kd_gains = Eigen::Vector3d(
      yml["Kd"][0].as<double>(), yml["Kd"][1].as<double>(), yml["Kd"][2].as<double>());
  p.controller_params.indi_controller_params.pid_params.alpha = Eigen::Vector3d(
      yml["alpha"][0].as<double>(), yml["alpha"][1].as<double>(), yml["alpha"][2].as<double>());
  p.controller_params.indi_controller_params.pid_params.antiwindup_cte =
      Eigen::Vector3d(yml["antiwindup_cte"][0].as<double>(), yml["antiwindup_cte"][1].as<double>(),
                      yml["antiwindup_cte"][2].as<double>());
  Eigen::Vector3d angular_acceleration_limit =
      Eigen::Vector3d(yml["angular_acceleration_limit"][0].as<double>(),
                      yml["angular_acceleration_limit"][1].as<double>(),
                      yml["angular_acceleration_limit"][2].as<double>());
  p.controller_params.indi_controller_params.pid_params.lower_output_saturation =
      -1.0 * angular_acceleration_limit;
  p.controller_params.indi_controller_params.pid_params.upper_output_saturation =
      angular_acceleration_limit;
  p.controller_params.indi_controller_params.pid_params.proportional_saturation_flag = true;

  // Controller params acro
  yml = yaml_config_file["controller"]["acro"];

  p.controller_params.acro_controller_params.gravity = p.dynamics_params.model_params.gravity;
  p.controller_params.acro_controller_params.vehicle_mass =
      p.dynamics_params.model_params.vehicle_mass;
  p.controller_params.acro_controller_params.kp_rot =
      Eigen::Vector3d(yml["kp_rot"][0].as<double>(), yml["kp_rot"][1].as<double>(),
                      yml["kp_rot"][2].as<double>())
          .asDiagonal();

  // Controller params trajectory
  yml = yaml_config_file["controller"]["trajectory"];

  p.controller_params.trajectory_controller_params.pid_params.Kp_gains = Eigen::Vector3d(
      yml["Kp"][0].as<double>(), yml["Kp"][1].as<double>(), yml["Kp"][2].as<double>());
  p.controller_params.trajectory_controller_params.pid_params.Ki_gains = Eigen::Vector3d(
      yml["Ki"][0].as<double>(), yml["Ki"][1].as<double>(), yml["Ki"][2].as<double>());
  p.controller_params.trajectory_controller_params.pid_params.Kd_gains = Eigen::Vector3d(
      yml["Kd"][0].as<double>(), yml["Kd"][1].as<double>(), yml["Kd"][2].as<double>());
  p.controller_params.trajectory_controller_params.pid_params.antiwindup_cte =
      Eigen::Vector3d(yml["antiwindup_cte"][0].as<double>(), yml["antiwindup_cte"][1].as<double>(),
                      yml["antiwindup_cte"][2].as<double>());
  Eigen::Vector3d linear_acceleration_limit =
      Eigen::Vector3d(yml["linear_acceleration_limit"][0].as<double>(),
                      yml["linear_acceleration_limit"][1].as<double>(),
                      yml["linear_acceleration_limit"][2].as<double>());
  p.controller_params.trajectory_controller_params.pid_params.lower_output_saturation =
      -1.0 * linear_acceleration_limit;
  p.controller_params.trajectory_controller_params.pid_params.upper_output_saturation =
      linear_acceleration_limit;
  p.controller_params.trajectory_controller_params.pid_params.proportional_saturation_flag = true;

  // Imu params
  yml = yaml_config_file["imu"];

  p.imu_params.gyro_noise_var                 = yml["gyro_noise_var"].as<double>();
  p.imu_params.accel_noise_var                = yml["accel_noise_var"].as<double>();
  p.imu_params.gyro_bias_noise_autocorr_time  = yml["gyro_bias_noise_autocorr_time"].as<double>();
  p.imu_params.accel_bias_noise_autocorr_time = yml["accel_bias_noise_autocorr_time"].as<double>();

  // Inertial odometry params
  yml = yaml_config_file["inertial_odometry"];

  p.inertial_odometry_params.alpha = yml["alpha"].as<double>();
  p.inertial_odometry_params.initial_world_orientation =
      p.dynamics_params.state.kinematics.orientation;
  return p;
}

SimulatorParams<double, 4> get_simulation_params_from_code() {
  // Initialize simulator params
  SimulatorParams p;

  // Initial state
  p.dynamics_params.state.kinematics.position             = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.linear_velocity      = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.linear_acceleration  = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.orientation          = Eigen::Quaterniond(1, 0, 0, 0);
  p.dynamics_params.state.kinematics.angular_velocity     = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.dynamics_params.state.kinematics.angular_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Vehicle parameters
  p.dynamics_params.model_params.gravity      = Eigen::Vector3d(0.0, 0.0, -9.81);
  p.dynamics_params.model_params.vehicle_mass = 1.0;
  p.dynamics_params.model_params.vehicle_inertia =
      Eigen::Vector3d(0.0049, 0.0049, 0.0069).asDiagonal();
  p.dynamics_params.model_params.vehicle_drag_coefficient = 0.1;
  p.dynamics_params.model_params.vehicle_aero_moment_coefficient =
      Eigen::Vector3d(0.003, 0.003, 0.003).asDiagonal();
  p.dynamics_params.model_params.force_process_noise_auto_correlation  = 0.0005;
  p.dynamics_params.model_params.moment_process_noise_auto_correlation = 1.25e-7;

  // Motors parameters
  const double thrust_coefficient = 1.91e-6;
  const double torque_coefficient = 2.6e-7;
  const double x_dist             = 0.08;
  const double y_dist             = 0.08;
  const double min_speed          = 0.0;
  const double max_speed          = 2200.0;
  const double time_constant      = 0.02;
  const double rotational_inertia = 6.62e-6;

  p.dynamics_params.model_params.motors_params =
      multirotor::model::Model<double, 4>::create_quadrotor_x_config(
          thrust_coefficient, torque_coefficient, x_dist, y_dist, min_speed, max_speed,
          time_constant, rotational_inertia);

  // Controller params Indi
  p.controller_params.indi_controller_params.inertia =
      p.dynamics_params.model_params.vehicle_inertia;
  auto mixing_matrix_6D_4rotors = multirotor::model::Model<double, 4>::compute_mixer_matrix<6>(
      p.dynamics_params.model_params.motors_params);
  p.controller_params.indi_controller_params.mixer_matrix_inverse =
      indi_controller::compute_quadrotor_mixer_matrix_inverse(mixing_matrix_6D_4rotors);
  p.controller_params.indi_controller_params.pid_params.Kp_gains = Eigen::Vector3d(9.0, 9.0, 9.0);
  p.controller_params.indi_controller_params.pid_params.Ki_gains = Eigen::Vector3d(3.0, 3.0, 3.0);
  p.controller_params.indi_controller_params.pid_params.Kd_gains = Eigen::Vector3d(0.3, 0.3, 0.3);
  p.controller_params.indi_controller_params.pid_params.alpha    = Eigen::Vector3d(0.6, 0.6, 0.6);
  p.controller_params.indi_controller_params.pid_params.antiwindup_cte =
      Eigen::Vector3d(1.0, 1.0, 1.0);

  // Controller params Acro
  p.controller_params.acro_controller_params.gravity = p.dynamics_params.model_params.gravity;
  p.controller_params.acro_controller_params.vehicle_mass =
      p.dynamics_params.model_params.vehicle_mass;
  p.controller_params.acro_controller_params.kp_rot = Eigen::Vector3d(20.0, 20.0, 8.0).asDiagonal();

  // Controller params Trajectory
  p.controller_params.trajectory_controller_params.pid_params.Kp_gains =
      Eigen::Vector3d(10.0, 10.0, 10.0);
  p.controller_params.trajectory_controller_params.pid_params.Ki_gains =
      Eigen::Vector3d(0.005, 0.005, 0.005);
  p.controller_params.trajectory_controller_params.pid_params.Kd_gains =
      Eigen::Vector3d(6.0, 6.0, 6.0);
  p.controller_params.trajectory_controller_params.pid_params.alpha =
      Eigen::Vector3d(0.0, 0.0, 0.0);
  p.controller_params.trajectory_controller_params.pid_params.antiwindup_cte =
      Eigen::Vector3d(1.0, 1.0, 1.0);

  // Controller params Speed
  p.controller_params.velocity_controller_params.pid_params.Kp_gains =
      Eigen::Vector3d(0.5, 0.5, 0.5);
  p.controller_params.velocity_controller_params.pid_params.Ki_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);
  p.controller_params.velocity_controller_params.pid_params.Kd_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);

  // Controller params Position
  p.controller_params.position_controller_params.pid_params.Kp_gains =
      Eigen::Vector3d(0.5, 0.5, 0.5);
  p.controller_params.position_controller_params.pid_params.Ki_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);
  p.controller_params.position_controller_params.pid_params.Kd_gains =
      Eigen::Vector3d(0.0, 0.0, 0.0);

  // Imu params
  p.imu_params.gyro_noise_var                 = 0.003;
  p.imu_params.accel_noise_var                = 0.005;
  p.imu_params.gyro_bias_noise_autocorr_time  = 1.0e-7;
  p.imu_params.accel_bias_noise_autocorr_time = 1.0e-7;

  // Inertial odometry params
  p.inertial_odometry_params.alpha = 0.9;

  return p;
}

SimulatorParams<double, 4> get_simulation_params() {
  // Check if file exists
  if (!check_file_exits()) {
    return get_simulation_params_from_code();
  }
  return get_simulation_params_from_yaml();
}

std::pair<double, double> get_sim_time(const std::string& file_path = SIM_CONFIG_PATH) {
  double sim_time = 10.0;
  double dt       = 0.001;

  // Read params
  if (!check_file_exits(file_path)) {
    return std::make_pair(sim_time, dt);
  }
  read_yaml_param("sim_config.sim_time", sim_time, file_path);
  read_yaml_param("sim_config.dt", dt, file_path);
  return std::make_pair(sim_time, dt);
}

double get_trajectory_generator_max_speed(const std::string& file_path = SIM_CONFIG_PATH) {
  double max_speed = 1.0;

  // Read params
  if (!check_file_exits(file_path)) {
    return max_speed;
  }
  read_yaml_param("sim_config.trajectory_generator_max_speed", max_speed, file_path);
  return max_speed;
}

std::vector<Eigen::Vector3d> get_trajectory_generator_waypoints(
    const std::string& file_path = SIM_CONFIG_PATH) {
  // trajectory_generator_waypoints:
  // - [0.0, 0.0, 1.0]
  // - [1.0, 0.0, 1.0]

  std::vector<Eigen::Vector3d> waypoints;
  if (!check_file_exits(file_path)) {
    waypoints = {Eigen::Vector3d(0.0, 0.0, 1.0)};
    return waypoints;
  }

  YAML::Node config = YAML::LoadFile(file_path);
  if (!config["sim_config"]["trajectory_generator_waypoints"]) {
    waypoints = {Eigen::Vector3d(0.0, 0.0, 1.0)};
    return waypoints;
  }

  for (auto waypoint : config["sim_config"]["trajectory_generator_waypoints"]) {
    waypoints.push_back(Eigen::Vector3d(waypoint[0].as<double>(), waypoint[1].as<double>(),
                                        waypoint[2].as<double>()));
  }
  return waypoints;
}

DynamicWaypoint::Vector eigen_vector_to_dynamic_waypoint_vector(
    const std::vector<Eigen::Vector3d>& vector_waypoints) {
  DynamicWaypoint::Vector vector_dynamic_waypoints;
  for (auto waypoint : vector_waypoints) {
    DynamicWaypoint dynamic_waypoint;
    dynamic_waypoint.resetWaypoint(waypoint);
    vector_dynamic_waypoints.push_back(dynamic_waypoint);
  }
  return vector_dynamic_waypoints;
}

Simulator<double, 4> get_simulator() {
  SimulatorParams simulator_params = get_simulation_params();
  Simulator simulator              = Simulator(simulator_params);
  simulator.enable_floor_collision(floor_height);
  simulator.arm();
  return simulator;
}

std::unique_ptr<Simulator<double, 4>> get_simulator_ptr() {
  SimulatorParams simulator_params = get_simulation_params();
  std::unique_ptr<Simulator<double, 4>> simulator =
      std::make_unique<Simulator<double, 4>>(simulator_params);
  simulator->enable_floor_collision(floor_height);
  simulator->arm();
  return simulator;
}

std::unique_ptr<DynamicTrajectory> get_trajectory_generator(
    const Eigen::Vector3d initial_position,
    const std::vector<Eigen::Vector3d>& waypoints = get_trajectory_generator_waypoints(),
    const double speed                            = get_trajectory_generator_max_speed()) {
  // Initialize dynamic trajectory generator
  std::unique_ptr<DynamicTrajectory> trajectory_generator = std::make_unique<DynamicTrajectory>();

  trajectory_generator->updateVehiclePosition(initial_position);
  trajectory_generator->setSpeed(speed);

  // Set waypoints
  DynamicWaypoint::Vector waypoints_to_set = eigen_vector_to_dynamic_waypoint_vector(waypoints);

  // Generate trajectory
  trajectory_generator->setWaypoints(waypoints_to_set);
  double max_time = trajectory_generator->getMaxTime();  // Block until trajectory is generated

  std::cout << "Trajectory generated with max time: " << max_time << std::endl;

  return trajectory_generator;
}

void test_arm(CsvLogger& logger,
              Simulator<double, 4>& simulator,
              const double sim_max_t,
              const double dt) {
  // Simulation
  double t = 0.0;  // seconds
  while (t < sim_max_t) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
    logger.save(t, simulator);
    t += dt;
  }
}

void test_motor_w_ref(CsvLogger& logger,
                      Simulator<double, 4>& simulator,
                      const double sim_max_t,
                      const double dt) {
  // Set control mode
  simulator.set_control_mode(ControlMode::MOTOR_W);
  Eigen::Vector4d motor_w_ref = Eigen::Vector4d::Ones() * 2200.0;
  simulator.set_refence_motors_angular_velocity(motor_w_ref);

  // Simulation
  double t = 0.0;  // seconds
  logger.save(t, simulator);
  while (t < sim_max_t) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
    logger.save(t, simulator);
    t += dt;
  }
  logger.close();
}

void test_indi_controller(CsvLogger& logger,
                          Simulator<double, 4>& simulator,
                          const double sim_max_t,
                          const double dt) {
  // Set control mode
  simulator.set_control_mode(ControlMode::ACRO);
  Eigen::Vector3d angular_velocity_ref = Eigen::Vector3d(0.0, 0.1, 0.0);
  simulator.set_reference_acro(15.0, angular_velocity_ref);

  // Simulation
  double t = 0.0;  // seconds
  logger.save(t, simulator);
  while (t < sim_max_t) {
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
    if (t > 2.0) {
      simulator.set_reference_acro(15.0, Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    logger.save(t, simulator);
    t += dt;
  }
  logger.close();
}

void test_geometric_controller(CsvLogger& logger,
                               Simulator<double, 4>& simulator,
                               const double sim_max_t,
                               const double dt) {
  // Set control mode
  simulator.set_control_mode(ControlMode::TRAJECTORY);

  // Initialize dynamic trajectory generator
  std::unique_ptr<DynamicTrajectory> trajectory_generator =
      get_trajectory_generator(simulator.get_dynamics_const().get_state().kinematics.position);
  double max_time = trajectory_generator->getMaxTime();

  // Initialize reference trajectory
  dynamic_traj_generator::References references;
  references.position = simulator.get_dynamics_const().get_state().kinematics.position;
  references.velocity = simulator.get_dynamics_const().get_state().kinematics.linear_velocity;
  references.acceleration =
      simulator.get_dynamics_const().get_state().kinematics.linear_acceleration;
  double roll_ref, pitch_ref, yaw_ref;
  quaternion_to_Euler(simulator.get_dynamics_const().get_state().kinematics.orientation, roll_ref,
                      pitch_ref, yaw_ref);

  simulator.set_reference_trajectory(references.position, references.velocity,
                                     references.acceleration);
  simulator.set_reference_yaw_angle(yaw_ref);

  // Simulation
  double t = 0.0;  // seconds
  logger.save(t, simulator);
  while (t < sim_max_t) {
    if (t < max_time) {
      trajectory_generator->evaluateTrajectory(t, references);
      simulator.set_reference_trajectory(references.position, references.velocity,
                                         references.acceleration);
      simulator.set_reference_yaw_angle(yaw_ref);
    }
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
    logger.save(t, simulator);
    t += dt;
  }
  logger.close();
}

double compute_yaw_angle_path_facing(double vx, double vy, Eigen::Quaterniond current_orientation) {
  if (sqrt(vx * vx + vy * vy) > 0.1) {
    return atan2f(vy, vx);
  }
  double roll, pitch, yaw;
  quaternion_to_Euler(current_orientation, roll, pitch, yaw);
  return yaw;
}

void test_trajectory_controller_with_takeoff(CsvLogger& logger,
                                             Simulator<double, 4>& simulator,
                                             const double sim_max_t,
                                             const double dt) {
  // Set control mode
  simulator.set_control_mode(ControlMode::TRAJECTORY);

  // Initialize dynamic trajectory generator for takeoff
  Eigen::Vector3d initial_position = simulator.get_dynamics_const().get_state().kinematics.position;
  Eigen::Vector3d takeoff_position = initial_position + Eigen::Vector3d(0.0, 0.0, 3.0);
  std::unique_ptr<DynamicTrajectory> trajectory_generator_takeoff =
      get_trajectory_generator(initial_position, {takeoff_position}, 1.0);
  double max_time_takeoff = trajectory_generator_takeoff->getMaxTime();

  // Initialize dynamic trajectory generator for trajectory
  std::unique_ptr<DynamicTrajectory> trajectory_generator =
      get_trajectory_generator(takeoff_position);
  double max_time = trajectory_generator->getMaxTime();

  // Initialize reference trajectory
  dynamic_traj_generator::References references;
  references.position = simulator.get_dynamics_const().get_state().kinematics.position;
  references.velocity = simulator.get_dynamics_const().get_state().kinematics.linear_velocity;
  references.acceleration =
      simulator.get_dynamics_const().get_state().kinematics.linear_acceleration;
  double roll_ref, pitch_ref, yaw_ref;
  quaternion_to_Euler(simulator.get_dynamics_const().get_state().kinematics.orientation, roll_ref,
                      pitch_ref, yaw_ref);

  simulator.set_reference_trajectory(references.position, references.velocity,
                                     references.acceleration);
  simulator.set_reference_yaw_angle(yaw_ref);

  // Simulation
  double t = 0.0;  // seconds
  logger.save(t, simulator);
  while (t < sim_max_t) {
    if (t < max_time_takeoff) {
      trajectory_generator_takeoff->evaluateTrajectory(t, references);
      simulator.set_reference_trajectory(references.position, references.velocity,
                                         references.acceleration);
      simulator.set_reference_yaw_angle(yaw_ref);
    } else if (t >= max_time_takeoff && t < max_time_takeoff + max_time) {
      trajectory_generator->evaluateTrajectory(t - max_time_takeoff, references);
      simulator.set_reference_trajectory(references.position, references.velocity,
                                         references.acceleration);
      simulator.set_reference_yaw_angle(compute_yaw_angle_path_facing(
          references.velocity.x(), references.velocity.y(),
          simulator.get_dynamics_const().get_state().kinematics.orientation));
    }
    simulator.update_controller(dt);
    simulator.update_dynamics(dt);
    simulator.update_imu(dt);
    simulator.update_inertial_odometry(dt);
    logger.save(t, simulator);
    t += dt;
  }
  logger.close();
}

}  // namespace multirotor

int main(int argc, char** argv) {
  using Simulator = multirotor::Simulator<double, 4>;
  using CsvLogger = multirotor::CsvLogger;

  // Logger
  std::string file_name = "multirotor_log.csv";
  CsvLogger logger(file_name);

  // Initialize simulator
  Simulator simulator = multirotor::get_simulator();

  // Pair of std::make_pair(max_time, dt)
  std::pair<double, double> sim_time = multirotor::get_sim_time();
  double max_time                    = sim_time.first;
  double dt                          = sim_time.second;

  // test_arm(logger, simulator, max_time, dt);
  // test_motor_w_ref(logger, simulator, max_time, dt);
  // test_indi_controller(logger, simulator, max_time, dt);
  // test_geometric_controller(logger, simulator, max_time, dt);
  multirotor::test_trajectory_controller_with_takeoff(logger, simulator, max_time, dt);
  return 0;
}
