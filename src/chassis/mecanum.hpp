#ifndef MECANUM_HPP
#define MECANUM_HPP

#include <array>
#include <cmath>

#include "coordinate/coordinate.hpp"

// constexpr int motor_amount = 4;

namespace chassis {

class Mecanum {
 public:
  Mecanum();
  Mecanum(const std::array<coordinate::CoordinatePolar, 4>& pos) {
    for (int i = 0; i < 4; ++i) {
      wheel_pos[i] = pos[i];
    }
  }
  Mecanum(const std::array<coordinate::Coordinate, 4>& pos) {
    for (int i = 0; i < 4; ++i) {
      wheel_pos[i] = static_cast<coordinate::CoordinatePolar>(pos[i]);
    }
  }
  Mecanum(const float radius) {
    for (int i = 0; i < 4; ++i) {
      constexpr float ofs = 2 * M_PI / 4;
      wheel_pos[i] = coordinate::CoordinatePolar(radius, ofs * i + M_PI / 4);
    }
  }

  void calc(const coordinate::Velocity& vel, float* result) {
    for (int i = 0; i < 4; ++i) {
      constexpr float ofs = 2 * M_PI / 4;
      const float vx =
          vel.x + wheel_pos[i].r * vel.ang * cos(wheel_pos[i].theta + M_PI / 2);
      const float vy =
          vel.y + wheel_pos[i].r * vel.ang * sin(wheel_pos[i].theta + M_PI / 2);
      result[i] = vy - tan(ofs * i + M_PI / 4) * vx;
    }
  }

 private:
  std::array<coordinate::CoordinatePolar, 4> wheel_pos;
};

}  // namespace chassis
#endif