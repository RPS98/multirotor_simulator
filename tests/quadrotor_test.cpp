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
    file_ << "Time,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,wxr,wyr,wzr,w1,w2,w3,w4,w1r,w2r,w3r,w4r,"
             "tx,ty,tz,txr,tyz,tzr,thrust,thrustr"
          << std::endl;
  }

  ~CsvLogger() { file_.close(); }

  void add_time_row(float time) { file_ << time << ","; }

  void add_float(float data) { file_ << data << ","; }

  void add_vector_row(const Eigen::Vector3f& data) {
    for (size_t i = 0; i < data.size(); ++i) {
      file_ << data[i] << ",";
    }
  }

  void add_vector_row(const Eigen::Vector4f& data) {
    for (size_t i = 0; i < data.size(); ++i) {
      file_ << data[i] << ",";
    }
  }

  void save(float time, const quadrotor::Quadrotor& quadrotor, quadrotor::actuation::Acro acro) {
    add_time_row(time);

    // Position
    add_vector_row(quadrotor.get_state()->kinematics.position);

    // Orientation
    float roll, pitch, yaw;
    quaternion_to_Euler(quadrotor.get_state()->kinematics.orientation, roll, pitch, yaw);
    add_vector_row(Eigen::Vector3f(roll, pitch, yaw));

    // Velocity
    add_vector_row(quadrotor.get_state()->kinematics.linear_velocity);

    // Angular velocity
    add_vector_row(quadrotor.get_state()->kinematics.angular_velocity);

    // Reference angular velocity
    add_vector_row(acro.angular_velocity);

    // Motor speeds
    add_vector_row(quadrotor.get_state()->actuators.motor_angular_velocity);

    // Reference motor speeds
    add_vector_row(quadrotor.get_flight_controller()->get_motor_speeds_angular_velocity());

    // Vehicle torque
    add_vector_row(quadrotor.get_state()->dynamics.torque);

    // Reference vehicle torque
    add_vector_row(quadrotor.get_flight_controller()->get_torque_desired());

    // Vehicle thrust
    add_float(Model::get_thrust_force(quadrotor.get_model()->get_motors_thrust_coefficient(),
                                      quadrotor.get_state()->actuators.motor_angular_velocity)
                  .z());

    // Reference vehicle thrust
    add_float(acro.thrust);

    // End line
    file_ << std::endl;
  }

  void close() { file_.close(); }

private:
  std::string file_name_;
  std::ofstream file_;
};

Quadrotor quadrotor_initialize() {
  quadrotor_params params;
  State initial_state;
  initial_state.actuators.motor_angular_velocity =
      Eigen::Vector4f(1000.0f, 1000.0f, 1000.0f, 1000.0f);

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
  params.moment_process_noise_auto_correlation = 0.0f;  // 1.25e-7f;
  params.force_process_noise_auto_correlation  = 0.0f;  // 5.00e-5f;
  params.initial_state                         = initial_state;
  float kp                                     = 10.0f;
  float ki                                     = 6.0f;
  float kd                                     = 0.02f;
  params.kp                                    = Eigen::Vector3f(kp, kp, kp);
  params.ki                                    = Eigen::Vector3f(ki, ki, ki);
  params.kd                                    = Eigen::Vector3f(kd, kd, kd);
  params.antiwindup_cte                        = 1500.0f;
  params.alpha                                 = 0.4f;
  params.gyro_noise_var                        = 0.003f;
  params.accel_noise_var                       = 0.005f;
  params.gyro_bias_noise_autocorr_time         = 1.0e-7f;
  params.accel_bias_noise_autocorr_time        = 1.0e-7f;
  params.imu_orientation                       = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);

  Quadrotor quadrotor(params);

  return quadrotor;
}

void test_quadrotor_input_step(std::string log_filename = "quadrotor.csv") {
  // Initialize quadrotor
  Quadrotor quadrotor = quadrotor_initialize();

  // Logger
  CsvLogger logger(log_filename);

  // For 100 iterations, simulate the quadrotor with a constant thrust and angular velocity
  quadrotor::actuation::Acro acro;

  float dt              = 0.001f;  // Delta time in seconds
  float time            = 0.0f;    // Time in seconds
  float simulation_time = 1.0f;    // Time in seconds of simulation
  float time_step       = 0.5f;    // Time in seconds of step
  float thrust          = 10.0f;   // Thrust in Newtons
  float step            = 1.0f;    // Step in rad/s

  acro.thrust           = thrust;
  acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  for (float time = 0.0f; time < simulation_time; time += dt) {
    // If time is equal to time_step, step the angular velocity
    if ((time - time_step) > -0.5f * dt && (time - time_step) < 0.5f * dt) {
      LOG("Step");
      acro.angular_velocity = Eigen::Vector3f(0.0f, step, 0.0f);
    }
    quadrotor.update(dt, acro);
    logger.save(time, quadrotor, acro);
  }
  logger.close();
}

void test_quadrotor_input_delta(std::string log_filename = "quadrotor.csv") {
  // Initialize quadrotor
  Quadrotor quadrotor = quadrotor_initialize();

  // Logger
  CsvLogger logger(log_filename);

  // For 100 iterations, simulate the quadrotor with a constant thrust and angular velocity
  quadrotor::actuation::Acro acro;

  float dt              = 0.001f;  // Delta time in seconds
  float time            = 0.0f;    // Time in seconds
  float simulation_time = 1.0f;    // Time in seconds of simulation
  float time_delta      = 0.5f;    // Time in seconds of delta
  float duration_delta  = 0.1f;    // Time in seconds of delta
  float thrust          = 10.0f;   // Thrust in Newtons
  float step            = 1.0f;    // Step in rad/s

  acro.thrust           = thrust;
  acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  for (float time = 0.0f; time < simulation_time; time += dt) {
    // If time is equal to time_step, step the angular velocity
    if ((time - time_delta) > -0.5f * dt && (time - time_delta) < 0.5f * dt) {
      LOG("Delta rise");
      acro.angular_velocity = Eigen::Vector3f(0.0f, step, 0.0f);
    } else if ((time - time_delta - duration_delta) > -0.5f * dt &&
               (time - time_delta - duration_delta) < 0.5f * dt) {
      LOG("Step fall");
      acro.angular_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    }
    quadrotor.update(dt, acro);
    logger.save(time, quadrotor, acro);
  }
  logger.close();
}

int main(int argc, char* argv[]) {
  test_quadrotor_input_step();
  test_quadrotor_input_delta();
  return 0;
}
