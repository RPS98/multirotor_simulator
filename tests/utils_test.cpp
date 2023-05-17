#include "quadrotor_model/common/utils.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

void test_clamp_vector() {
  Eigen::Vector4f vector;
  vector << 1, 2, 3, 4;
  float min = 2;
  float max = 3;
  quadrotor::utils::clamp_vector(vector, min, max);
  assert(vector(0) == 2);
  assert(vector(1) == 2);
  assert(vector(2) == 3);
  assert(vector(3) == 3);
}

int main(int argc, char* argv[]) {
  test_clamp_vector();
  return 0;
}
