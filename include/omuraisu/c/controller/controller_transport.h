#ifndef CONTROLLER_TRANSPORT_H
#define CONTROLLER_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

#include "controller_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief シリアル通信用のパケット構造体（COBS用）
typedef struct __attribute__((packed)) {
  uint8_t header;
  int8_t left_x;
  int8_t left_y;
  int8_t right_x;
  int8_t right_y;
  uint8_t l2_trigger;
  uint8_t r2_trigger;
  uint16_t buttons;
  uint8_t dpad;
  uint8_t checksum;
} SerialPacket;

/// @brief シリアルパケットのヘッダ値
#define OM_CONTROLLER_SERIAL_PACKET_HEADER 0xAA

/// @brief CAN通信用のID定義
#define OM_CONTROLLER_CAN_ID_ANALOG 50
#define OM_CONTROLLER_CAN_ID_BUTTONS 51

/// @brief ROS Joy互換の軸インデックス
#define OM_CONTROLLER_AXIS_LEFT_X 0
#define OM_CONTROLLER_AXIS_LEFT_Y 1
#define OM_CONTROLLER_AXIS_RIGHT_X 2
#define OM_CONTROLLER_AXIS_RIGHT_Y 3
#define OM_CONTROLLER_AXIS_L2 4
#define OM_CONTROLLER_AXIS_R2 5

/// @brief ROS Joy互換のボタンインデックス
#define OM_CONTROLLER_BUTTON_INDEX_CIRCLE 0
#define OM_CONTROLLER_BUTTON_INDEX_CROSS 1
#define OM_CONTROLLER_BUTTON_INDEX_SQUARE 2
#define OM_CONTROLLER_BUTTON_INDEX_TRIANGLE 3
#define OM_CONTROLLER_BUTTON_INDEX_DPAD_UP 4
#define OM_CONTROLLER_BUTTON_INDEX_DPAD_DOWN 5
#define OM_CONTROLLER_BUTTON_INDEX_DPAD_LEFT 6
#define OM_CONTROLLER_BUTTON_INDEX_DPAD_RIGHT 7
#define OM_CONTROLLER_BUTTON_INDEX_L1 8
#define OM_CONTROLLER_BUTTON_INDEX_R1 9
#define OM_CONTROLLER_BUTTON_INDEX_L3 10
#define OM_CONTROLLER_BUTTON_INDEX_R3 11
#define OM_CONTROLLER_BUTTON_INDEX_SHARE 12
#define OM_CONTROLLER_BUTTON_INDEX_OPTIONS 13

uint8_t om_ctrl_serial_packet_calc_checksum(const SerialPacket* packet);
bool om_ctrl_serial_packet_verify_checksum(const SerialPacket* packet);
SerialPacket om_ctrl_serial_packet_init(void);
SerialPacket om_ctrl_serial_packet_from_data(const ControllerData* data);
ControllerData om_ctrl_serial_packet_to_data(const SerialPacket* packet);
ControllerData om_ctrl_data_from_serial(const SerialPacket* packet);
bool om_ctrl_data_from_can(ControllerData* destination, uint32_t id,
                           const uint8_t data[8]);
bool om_ctrl_data_to_can_analog(const ControllerData* data, uint8_t out[8]);
bool om_ctrl_data_to_can_buttons(const ControllerData* data, uint8_t out[8]);
ControllerData om_ctrl_data_from_ros_joy(const float* axes, size_t axes_size,
                                         const int32_t* buttons,
                                         size_t buttons_size);
bool om_ctrl_data_to_ros_joy(const ControllerData* data, float* axes,
                             size_t axes_size, int32_t* buttons,
                             size_t buttons_size);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROLLER_TRANSPORT_H
