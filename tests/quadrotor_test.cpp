#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <memory>

#include "quadrotor_model/common/actuation.hpp"
#include "quadrotor_model/common/model.hpp"
#include "quadrotor_model/common/state.hpp"
#include "quadrotor_model/dynamics/dynamics.hpp"
#include "quadrotor_model/flight_controller/flight_controller.hpp"
#include "quadrotor_model/imu/imu.hpp"
#include "quadrotor_model/quadrotor.hpp"

#define LOG(x) std::cout << x << std::endl

using namespace quadrotor;

void quaternion_to_Euler(const Eigen::Quaternionf& _quaternion,
                         float& roll,
                         float& pitch,
                         float& yaw) {
  // roll (x-axis rotation)
  Eigen::Quaternionf quaternion = _quaternion.normalized();

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
  CsvLogger(const std::string& file_name) : file_name_(file_name) {
    LOG("Saving to file: " << file_name);
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "Time,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,w1,w2,w3,w4,w1r,w2r,w3r,w4r,tx,ty,tz,txr,"
             "tyz,tzr"
          << std::endl;
  }

  ~CsvLogger() { file_.close(); }

  void save(float time, const std::vector<float>& data) {
    file_ << time << ",";
    for (size_t i = 0; i < data.size(); ++i) {
      file_ << data[i] << ",";
    }
    file_ << std::endl;
  }

  void save(float time, const Eigen::Vector3f& data) {
    file_ << time << ",";
    for (size_t i = 0; i < data.size(); ++i) {
      file_ << data[i] << ",";
    }
    file_ << std::endl;
  }

  void save(float time, const Eigen::Vector3f& data_p, const Eigen::Vector3f& data_q) {
    file_ << time << ",";
    for (size_t i = 0; i < data_p.size(); ++i) {
      file_ << data_p[i] << ",";
    }
    for (size_t i = 0; i < data_q.size(); ++i) {
      file_ << data_q[i] << ",";
    }
    file_ << std::endl;
  }

  void save(float time,
            const Eigen::Vector3f& data_p,
            const Eigen::Vector3f& data_q,
            const Eigen::Vector3f& data_v,
            const Eigen::Vector3f& data_w,
            const Eigen::Vector4f& data_mw,
            const Eigen::Vector4f& data_mw_ref,
            const Eigen::Vector3f& data_torque,
            const Eigen::Vector3f& data_torque_ref) {
    file_ << time << ",";
    for (size_t i = 0; i < data_p.size(); ++i) {
      file_ << data_p[i] << ",";
    }
    for (size_t i = 0; i < data_q.size(); ++i) {
      file_ << data_q[i] << ",";
    }
    for (size_t i = 0; i < data_v.size(); ++i) {
      file_ << data_v[i] << ",";
    }
    for (size_t i = 0; i < data_w.size(); ++i) {
      file_ << data_w[i] << ",";
    }
    for (size_t i = 0; i < data_mw.size(); ++i) {
      file_ << data_mw[i] << ",";
    }
    for (size_t i = 0; i < data_mw_ref.size(); ++i) {
      file_ << data_mw_ref[i] << ",";
    }
    for (size_t i = 0; i < data_torque.size(); ++i) {
      file_ << data_torque[i] << ",";
    }
    for (size_t i = 0; i < data_torque_ref.size(); ++i) {
      file_ << data_torque_ref[i] << ",";
    }
    file_ << std::endl;
  }

  void save(float time,
            const quadrotor::State& data,
            const Eigen::Vector4f motor_w_ref,
            const Eigen::Vector3f torque_ref) {
    float roll, pitch, yaw;
    quaternion_to_Euler(data.kinematics.orientation, roll, pitch, yaw);
    save(time, data.kinematics.position, Eigen::Vector3f(roll, pitch, yaw),
         data.kinematics.linear_velocity, data.kinematics.angular_velocity,
         data.actuators.motor_angular_velocity, motor_w_ref, data.dynamics.torque, torque_ref);
  }

  void close() { file_.close(); }

private:
  std::string file_name_;
  std::ofstream file_;
};

void print_ground_truth(const State& state) {
  LOG("GT Pose: "
      << "x: " << state.kinematics.position.x() << ", y: " << state.kinematics.position.y()
      << ", z: " << state.kinematics.position.z() << ", w: " << state.kinematics.orientation.w()
      << ", qx: " << state.kinematics.orientation.x() << ", qy: "
      << state.kinematics.orientation.y() << ", qz: " << state.kinematics.orientation.z());

  LOG("GT Twist: "
      << "vx: " << state.kinematics.linear_velocity.x() << ", vy: "
      << state.kinematics.linear_velocity.y() << ", vz: " << state.kinematics.linear_velocity.z()
      << ", wx: " << state.kinematics.angular_velocity.x()
      << ", wy: " << state.kinematics.angular_velocity.y()
      << ", wz: " << state.kinematics.angular_velocity.z());
}

void print_odometry(const state::Kinematics& state_kinematics) {
  LOG("Odometry: "
      << "x: " << state_kinematics.position.x() << ", y: " << state_kinematics.position.y()
      << ", z: " << state_kinematics.position.z() << ", w: " << state_kinematics.orientation.w()
      << ", qx: " << state_kinematics.orientation.x() << ", qy: "
      << state_kinematics.orientation.y() << ", qz: " << state_kinematics.orientation.z()
      << ", vx: " << state_kinematics.linear_velocity.x() << ", vy: "
      << state_kinematics.linear_velocity.y() << ", vz: " << state_kinematics.linear_velocity.z()
      << ", wx: " << state_kinematics.angular_velocity.x()
      << ", wy: " << state_kinematics.angular_velocity.y()
      << ", wz: " << state_kinematics.angular_velocity.z());
}

void print_imu(const Eigen::Vector3f& gyroscope_measurement,
               const Eigen::Vector3f accelerometer_measurement) {
  LOG("IMU: "
      << "ax: " << accelerometer_measurement.x() << ", ay: " << accelerometer_measurement.y()
      << ", az: " << accelerometer_measurement.z() << ", wx: " << gyroscope_measurement.x()
      << ", wy: " << gyroscope_measurement.y() << ", wz: " << gyroscope_measurement.z());
}

void print_state(const State& state,
                 const state::Kinematics& state_kinematics,
                 const Eigen::Vector3f& gyroscope_measurement,
                 const Eigen::Vector3f& accelerometer_measurement) {
  // LOG("STATE");
  print_ground_truth(state);
  // print_odometry(state_kinematics);
  // print_imu(gyroscope_measurement, accelerometer_measurement);
  LOG("");
  return;
}

void test_quadrotor() {
  quadrotor_params params;
  State initial_state;

  params.floor_height                    = -10.0f;
  params.motor_thrust_coefficient        = 1.91e-6f;
  params.motor_torque_coefficient        = 2.6e-7f;
  params.motor_dx                        = 0.08f;
  params.motor_dy                        = 0.08f;
  params.motor_min_speed                 = 0.0f;
  params.motor_max_speed                 = 2200.0f;
  params.motor_time_constant             = 0.02f;
  params.motor_rotational_inertia        = 6.62e-6f;
  params.motors_frame_type               = 1;  // 0: +-Configuration, 1: X-Configuration
  params.vehicle_mass                    = 1.0f;
  params.vehicle_inertia                 = Eigen::Vector3f(0.0049f, 0.0049f, 0.0069f).asDiagonal();
  params.vehicle_drag_coefficient        = 0.1f;
  params.vehicle_aero_moment_coefficient = Eigen::Vector3f(0.003f, 0.003f, 0.003f).asDiagonal();
  params.gravity                         = Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  params.moment_process_noise_auto_correlation = 1.25e-7f;
  params.force_process_noise_auto_correlation  = 0.0005f;
  params.initial_state                         = initial_state;
  params.kp                                    = Eigen::Vector3f(9.0f, 9.0f, 9.0f);
  params.ki                                    = Eigen::Vector3f(3.0f, 3.0f, 3.0f);
  params.kd                                    = Eigen::Vector3f(0.3f, 0.3f, 0.3f);
  params.gyro_noise_var                        = 0.003f;
  params.accel_noise_var                       = 0.005f;
  params.gyro_bias_noise_autocorr_time         = 1.0e-7f;
  params.accel_bias_noise_autocorr_time        = 1.0e-7f;
  params.imu_orientation                       = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);

  Quadrotor quadrotor(params);

  // State
  State state;
  state::Kinematics state_kinematics;
  Eigen::Vector3f gyroscope_measurement;
  Eigen::Vector3f accelerometer_measurement;

  // Logger
  // std::string filename =
  //     "quadrotor_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count())
  //     +
  //     ".csv";
  std::string filename = "quadrotor.csv";
  CsvLogger logger(filename);

  // For 100 iterations, simulate the quadrotor with a constant thrust and angular velocity
  quadrotor::actuation::Acro acro;
  acro.thrust           = 1.0f;
  acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

  float dt   = 0.0001f;
  float time = 0.0f;
  // print_state(quadrotor);
  int iterations = 30000;

  Eigen::Vector4f motor_speeds_ref_;
  Eigen::Vector3f torque_ref_;

  for (int i = 0; i < iterations; i++) {
    if (i == 20000) {
      acro.angular_velocity.y() = 0.01f;
      LOG("Applying angular velocity");
    }

    // quadrotor.update(dt, acro);
    quadrotor.apply_floor_force();
    quadrotor.acro_to_motor_speeds(acro);
    motor_speeds_ref_ = quadrotor.actuation_motor_w_.angular_velocity;
    torque_ref_       = quadrotor.flight_controller_->get_torque_desired();
    quadrotor.process_euler_explicit(dt);
    quadrotor.apply_floor_limit();
    quadrotor.update_imu(dt);

    // Update State
    quadrotor.get_state(state);
    quadrotor.get_imu_measurement(state_kinematics);
    quadrotor.get_imu_measurement(gyroscope_measurement, accelerometer_measurement);

    // print_state(state, state_kinematics, gyroscope_measurement, accelerometer_measurement);
    logger.save(time, state, motor_speeds_ref_, torque_ref_);
    time += dt;
  }
  logger.close();
}

int main(int argc, char* argv[]) {
  test_quadrotor();
  return 0;
}
