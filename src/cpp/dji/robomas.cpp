#include "dji/robomas.hpp"

namespace omuraisu {
namespace dji {
Robomas::Robomas(can::ICanBus& bus) noexcept : bus_(bus) {}

void Robomas::set_max_output(int16_t max) { core_.set_max_output(max); }

int Robomas::read() {
  can::CanMessage msg;
  if (!bus_.read(msg)) {
    return -1;
  }
  return core_.parse(msg.id, msg.data);
}

bool Robomas::write() {
  can::CanMessage msg1;
  can::CanMessage msg2;
  core_.get_output_group(msg1.data, 0);
  msg1.id = TX_ID_GROUP1;
  msg1.len = 8;

  core_.get_output_group(msg2.data, 1);
  msg2.id = TX_ID_GROUP2;
  msg2.len = 8;

  bool success1 = bus_.write(msg1);
  bool success2 = bus_.write(msg2);
  return success1 && success2;
}

int Robomas::parse(uint32_t id, const uint8_t data[8]) {
  return core_.parse(id, data);
}
void Robomas::set_output(int16_t current, int id) {
  core_.set_output(current, id);
}
void Robomas::set_output_percent(float percent, int id) {
  core_.set_output_percent(percent, id);
}
void Robomas::get_output(uint8_t out[2][8]) const { core_.get_output(out); }
void Robomas::get_output_group(uint8_t out[8], unsigned int group) const {
  core_.get_output_group(out, group);
}
int16_t Robomas::get_current(int id) const { return core_.get_current(id); }
uint16_t Robomas::get_angle(int id) const { return core_.get_angle(id); }
int32_t Robomas::get_rotation_count(int id) const {
  return core_.get_rotation_count(id);
}
float Robomas::get_total_angle(int id) const {
  return core_.get_total_angle(id);
}
int16_t Robomas::get_rpm(int id) const { return core_.get_rpm(id); }
uint8_t Robomas::get_temp(int id) const { return core_.get_temp(id); }
RobomasData Robomas::get_data(int id) const { return core_.get_data(id); }
}  // namespace dji
}  // namespace omuraisu
