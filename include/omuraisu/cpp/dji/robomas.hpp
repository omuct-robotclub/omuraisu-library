#ifndef OMURAISU_CPP_DJI_ROBOMAS_HPP_
#define OMURAISU_CPP_DJI_ROBOMAS_HPP_
#include "can/can_interface.hpp"
#include "robomas_core.hpp"

namespace omuraisu {
namespace dji {
class Robomas {
 public:
  Robomas(can::ICanBus& bus) noexcept;
  Robomas(const Robomas&) = delete;
  Robomas& operator=(const Robomas&) = delete;

  void set_max_output(int16_t max);
  int read();
  bool write();
  int parse(uint32_t id, const uint8_t data[8]);
  void set_output(int16_t current, int id);
  void set_output_percent(float percent, int id);
  void get_output(uint8_t out[2][8]) const;
  void get_output_group(uint8_t out[8], unsigned int group) const;
  int16_t get_current(int id) const;
  uint16_t get_angle(int id) const;
  int32_t get_rotation_count(int id) const;
  float get_total_angle(int id) const;
  int16_t get_rpm(int id) const;
  uint8_t get_temp(int id) const;
  RobomasData get_data(int id) const;

 private:
  can::ICanBus& bus_;
  RobomasCore core_;
};
}  // namespace dji
}  // namespace omuraisu
#endif  // OMURAISU_CPP_DJI_ROBOMAS_HPP_