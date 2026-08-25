#ifndef OMURAISU_CPP_CONTROLLER_CONTROLLER_TRANSPORT_HPP_
#define OMURAISU_CPP_CONTROLLER_CONTROLLER_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>

#include "controller/controller_core.hpp"
#include "controller/controller_transport.h"

namespace omuraisu {
namespace controller {
struct SerialPacket : public ::SerialPacket {
  SerialPacket();
  explicit SerialPacket(const ControllerData& data);
  static SerialPacket from_data(const ControllerData& data);

  uint8_t calc_checksum() const;
  bool verify_checksum() const;
  ControllerData to_data() const;
} __attribute__((packed));

bool data_from_can(ControllerData* destination, uint32_t id,
                   const uint8_t data[8]);
ControllerData data_from_ros_joy(const float* axes, size_t axes_size,
                                 const int32_t* buttons, size_t buttons_size);
bool data_to_can_analog(const ControllerData& data, uint8_t out[8]);
bool data_to_can_buttons(const ControllerData& data, uint8_t out[8]);
bool data_to_ros_joy(const ControllerData& data, float* axes, size_t axes_size,
                     int32_t* buttons, size_t buttons_size);

}  // namespace controller
}  // namespace omuraisu

#endif  // OMURAISU_CPP_CONTROLLER_CONTROLLER_TRANSPORT_HPP_
