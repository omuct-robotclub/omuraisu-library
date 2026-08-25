#include "controller/controller_transport.hpp"

namespace omuraisu {
namespace controller {
SerialPacket::SerialPacket() : ::SerialPacket(om_ctrl_serial_packet_init()) {}

SerialPacket::SerialPacket(const ControllerData& data)
    : ::SerialPacket(om_ctrl_serial_packet_from_data(
          static_cast<const ::ControllerData*>(&data))) {}

SerialPacket SerialPacket::from_data(const ControllerData& data) {
  return SerialPacket(data);
}

uint8_t SerialPacket::calc_checksum() const {
  return om_ctrl_serial_packet_calc_checksum(this);
}

bool SerialPacket::verify_checksum() const {
  return om_ctrl_serial_packet_verify_checksum(this);
}

ControllerData SerialPacket::to_data() const {
  ::ControllerData data = om_ctrl_serial_packet_to_data(this);
  ControllerData out = {};
  static_cast<::ControllerData&>(out) = data;
  return out;
}

bool data_from_can(ControllerData* destination, uint32_t id,
                   const uint8_t data[8]) {
  if (om_ctrl_data_from_can(destination, id, data)) return true;
  return false;
}

ControllerData data_from_ros_joy(const float* axes, size_t axes_size,
                                 const int32_t* buttons, size_t buttons_size) {
  ::ControllerData c_data =
      om_ctrl_data_from_ros_joy(axes, axes_size, buttons, buttons_size);
  ControllerData out = {};
  static_cast<::ControllerData&>(out) = c_data;
  return out;
}

bool data_to_can_analog(const ControllerData& data, uint8_t out[8]) {
  return om_ctrl_data_to_can_analog(static_cast<const ::ControllerData*>(&data),
                                    out);
}

bool data_to_can_buttons(const ControllerData& data, uint8_t out[8]) {
  return om_ctrl_data_to_can_buttons(
      static_cast<const ::ControllerData*>(&data), out);
}

bool data_to_ros_joy(const ControllerData& data, float* axes, size_t axes_size,
                     int32_t* buttons, size_t buttons_size) {
  return om_ctrl_data_to_ros_joy(static_cast<const ::ControllerData*>(&data),
                                 axes, axes_size, buttons, buttons_size);
}

}  // namespace controller
}  // namespace omuraisu
