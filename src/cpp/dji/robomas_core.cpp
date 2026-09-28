#include "dji/robomas_core.hpp"

namespace omuraisu {
namespace dji {
RobomasData::RobomasData() noexcept : ::RobomasData{0} {}
RobomasData::RobomasData(const ::RobomasData& other) noexcept
    : ::RobomasData{other} {}
RobomasData::RobomasData(const RobomasData& other) noexcept
    : ::RobomasData{other} {}
RobomasData& RobomasData::operator=(const RobomasData& other) noexcept {
  ::RobomasData::operator=(other);
  return *this;
}

void RobomasData::parse(const uint8_t raw[8]) noexcept {
  om_rm_data_parse(this, raw);
}  // namespace dji

RobomasCore::RobomasCore() noexcept : core_(om_rm_core_init()) {}
RobomasCore::RobomasCore(const ::RobomasCore& other) noexcept : core_(other) {}
RobomasCore::RobomasCore(const RobomasCore& other) noexcept
    : core_(other.core_) {}

RobomasCore& RobomasCore::operator=(const RobomasCore& other) noexcept {
  if (this == &other) {
    return *this;
  }
  core_ = other.core_;
  return *this;
}

void RobomasCore::set_max_output(int16_t max) {
  om_rm_core_set_max_output(&core_, max);
}
int RobomasCore::parse(uint32_t id, const uint8_t data[8]) {
  return om_rm_core_parse(&core_, id, data);
}
void RobomasCore::set_output(int16_t current, int id) {
  om_rm_core_set_output(&core_, current, id);
}
void RobomasCore::set_output_percent(float percent, int id) {
  om_rm_core_set_output_percent(&core_, percent, id);
}
void RobomasCore::get_output(uint8_t out[2][8]) const {
  om_rm_core_get_output(&core_, out);
}
void RobomasCore::get_output_group(uint8_t out[8], unsigned int group) const {
  om_rm_core_get_output_group(&core_, out, group);
}
int16_t RobomasCore::get_current(int id) const {
  return om_rm_core_get_current(&core_, id);
}
uint16_t RobomasCore::get_angle(int id) const {
  return om_rm_core_get_angle(&core_, id);
}
int32_t RobomasCore::get_rotation_count(int id) const {
  return om_rm_core_get_rotation_count(&core_, id);
}
float RobomasCore::get_total_angle(int id) const {
  return om_rm_core_get_total_angle(&core_, id);
}
int16_t RobomasCore::get_rpm(int id) const {
  return om_rm_core_get_rpm(&core_, id);
}
uint8_t RobomasCore::get_temp(int id) const {
  return om_rm_core_get_temp(&core_, id);
}
RobomasData RobomasCore::get_data(int id) const {
  return static_cast<RobomasData>(om_rm_core_get_data(&core_, id));
}
}  // namespace dji
}  // namespace omuraisu