#ifndef OMURAISU_CPP_DJI_ROBOMAS_CORE_HPP_
#define OMURAISU_CPP_DJI_ROBOMAS_CORE_HPP_

#include "dji/robomas_core.h"
namespace omuraisu {
namespace dji {
struct RobomasData : public ::RobomasData {
  RobomasData() noexcept;
  explicit RobomasData(const ::RobomasData& other) noexcept;
  RobomasData(const RobomasData& other) noexcept;

  RobomasData& operator=(const RobomasData& other) noexcept;

  void parse(const uint8_t raw[8]) noexcept;
};

class RobomasCore {
 public:
  RobomasCore() noexcept;
  explicit RobomasCore(const ::RobomasCore& other) noexcept;
  RobomasCore(const RobomasCore& other) noexcept;

  RobomasCore& operator=(const RobomasCore& other) noexcept;

  void set_max_output(int16_t max);
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
  ::RobomasCore core_;
};
}  // namespace dji
}  // namespace omuraisu
#endif  // OMURAISU_CPP_DJI_ROBOMAS_CORE_HPP_