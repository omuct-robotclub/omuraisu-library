#include "controller/controller_transport.h"

uint8_t om_ctrl_serial_packet_calc_checksum(const SerialPacket* packet) {
  if (packet == NULL) {
    return 0;
  }

  uint8_t checksum = packet->header;
  checksum ^= (uint8_t)packet->left_x;
  checksum ^= (uint8_t)packet->left_y;
  checksum ^= (uint8_t)packet->right_x;
  checksum ^= (uint8_t)packet->right_y;
  checksum ^= packet->l2_trigger;
  checksum ^= packet->r2_trigger;
  checksum ^= (uint8_t)(packet->buttons & 0xFF);
  checksum ^= (uint8_t)((packet->buttons >> 8) & 0xFF);
  checksum ^= packet->dpad;
  return checksum;
}

bool om_ctrl_serial_packet_verify_checksum(const SerialPacket* packet) {
  return packet != NULL &&
         om_ctrl_serial_packet_calc_checksum(packet) == packet->checksum;
}

SerialPacket om_ctrl_serial_packet_init(void) {
  SerialPacket packet = {0};
  packet.header = OM_CONTROLLER_SERIAL_PACKET_HEADER;
  packet.checksum = om_ctrl_serial_packet_calc_checksum(&packet);
  return packet;
}

SerialPacket om_ctrl_serial_packet_from_data(const ControllerData* data) {
  SerialPacket packet = {0};
  packet.header = OM_CONTROLLER_SERIAL_PACKET_HEADER;

  if (data != NULL) {
    packet.left_x = (int8_t)(data->left_x * 127.0f);
    packet.left_y = (int8_t)(data->left_y * 127.0f);
    packet.right_x = (int8_t)(data->right_x * 127.0f);
    packet.right_y = (int8_t)(data->right_y * 127.0f);
    packet.l2_trigger = (uint8_t)(data->l2_trigger * 255.0f);
    packet.r2_trigger = (uint8_t)(data->r2_trigger * 255.0f);
    packet.buttons = data->buttons;
    packet.dpad = data->dpad;
  }

  packet.checksum = om_ctrl_serial_packet_calc_checksum(&packet);
  return packet;
}

ControllerData om_ctrl_serial_packet_to_data(const SerialPacket* packet) {
  ControllerData data = {0};

  if (packet != NULL) {
    data.left_x = packet->left_x / 127.0f;
    data.left_y = packet->left_y / 127.0f;
    data.right_x = packet->right_x / 127.0f;
    data.right_y = packet->right_y / 127.0f;
    data.l2_trigger = packet->l2_trigger / 255.0f;
    data.r2_trigger = packet->r2_trigger / 255.0f;
    data.buttons = packet->buttons;
    data.dpad = packet->dpad;
  }

  return data;
}

ControllerData om_ctrl_data_from_serial(const SerialPacket* packet) {
  return om_ctrl_serial_packet_to_data(packet);
}

ControllerData om_ctrl_data_from_can(uint32_t id, const uint8_t data[8]) {
  ControllerData controller = {0};

  if (data == NULL) {
    return controller;
  }

  switch (id) {
    case OM_CONTROLLER_CAN_ID_ANALOG:
      controller.left_x = (int8_t)data[0] / 127.0f;
      controller.left_y = (int8_t)data[1] / 127.0f;
      controller.right_x = (int8_t)data[2] / 127.0f;
      controller.right_y = (int8_t)data[3] / 127.0f;
      controller.l2_trigger = data[4] / 255.0f;
      controller.r2_trigger = data[5] / 255.0f;
      return controller;

    case OM_CONTROLLER_CAN_ID_BUTTONS:
      if ((data[0] >> 2) & 1) {
        controller.dpad |= OM_CONTROLLER_DPAD_UP;
      }
      if (data[0] & 1) {
        controller.dpad |= OM_CONTROLLER_DPAD_DOWN;
      }
      if ((data[0] >> 1) & 1) {
        controller.dpad |= OM_CONTROLLER_DPAD_LEFT;
      }
      if ((data[0] >> 3) & 1) {
        controller.dpad |= OM_CONTROLLER_DPAD_RIGHT;
      }

      if (data[1] & 0x08) {
        controller.buttons |= OM_CONTROLLER_BUTTON_CIRCLE;
      }
      if (data[1] & 0x04) {
        controller.buttons |= OM_CONTROLLER_BUTTON_TRIANGLE;
      }
      if (data[1] & 0x02) {
        controller.buttons |= OM_CONTROLLER_BUTTON_SQUARE;
      }
      if (data[1] & 0x01) {
        controller.buttons |= OM_CONTROLLER_BUTTON_CROSS;
      }
      if (data[2]) {
        controller.buttons |= OM_CONTROLLER_BUTTON_L1;
      }
      if (data[3]) {
        controller.buttons |= OM_CONTROLLER_BUTTON_R1;
      }
      if (data[4]) {
        controller.buttons |= OM_CONTROLLER_BUTTON_L3;
      }
      if (data[5]) {
        controller.buttons |= OM_CONTROLLER_BUTTON_R3;
      }
      if (data[6]) {
        controller.buttons |= OM_CONTROLLER_BUTTON_OPTIONS;
      }
      if (data[7]) {
        controller.buttons |= OM_CONTROLLER_BUTTON_SHARE;
      }
      return controller;

    default:
      return controller;
  }
}

bool om_ctrl_data_to_can_analog(const ControllerData* data, uint8_t out[8]) {
  if (data == NULL || out == NULL) {
    return false;
  }

  out[0] = (uint8_t)(int8_t)(data->left_x * 127.0f);
  out[1] = (uint8_t)(int8_t)(data->left_y * 127.0f);
  out[2] = (uint8_t)(int8_t)(data->right_x * 127.0f);
  out[3] = (uint8_t)(int8_t)(data->right_y * 127.0f);
  out[4] = (uint8_t)(data->l2_trigger * 255.0f);
  out[5] = (uint8_t)(data->r2_trigger * 255.0f);
  out[6] = 0;
  out[7] = 0;
  return true;
}

bool om_ctrl_data_to_can_buttons(const ControllerData* data, uint8_t out[8]) {
  if (data == NULL || out == NULL) {
    return false;
  }

  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;

  if ((data->dpad & OM_CONTROLLER_DPAD_DOWN) != 0) {
    out[0] |= 0x01;
  }
  if ((data->dpad & OM_CONTROLLER_DPAD_LEFT) != 0) {
    out[0] |= 0x02;
  }
  if ((data->dpad & OM_CONTROLLER_DPAD_UP) != 0) {
    out[0] |= 0x04;
  }
  if ((data->dpad & OM_CONTROLLER_DPAD_RIGHT) != 0) {
    out[0] |= 0x08;
  }

  if ((data->buttons & OM_CONTROLLER_BUTTON_CROSS) != 0) {
    out[1] |= 0x01;
  }
  if ((data->buttons & OM_CONTROLLER_BUTTON_SQUARE) != 0) {
    out[1] |= 0x02;
  }
  if ((data->buttons & OM_CONTROLLER_BUTTON_TRIANGLE) != 0) {
    out[1] |= 0x04;
  }
  if ((data->buttons & OM_CONTROLLER_BUTTON_CIRCLE) != 0) {
    out[1] |= 0x08;
  }

  out[2] = (data->buttons & OM_CONTROLLER_BUTTON_L1) != 0 ? 1 : 0;
  out[3] = (data->buttons & OM_CONTROLLER_BUTTON_R1) != 0 ? 1 : 0;
  out[4] = (data->buttons & OM_CONTROLLER_BUTTON_L3) != 0 ? 1 : 0;
  out[5] = (data->buttons & OM_CONTROLLER_BUTTON_R3) != 0 ? 1 : 0;
  out[6] = (data->buttons & OM_CONTROLLER_BUTTON_OPTIONS) != 0 ? 1 : 0;
  out[7] = (data->buttons & OM_CONTROLLER_BUTTON_SHARE) != 0 ? 1 : 0;
  return true;
}

ControllerData om_ctrl_data_from_ros_joy(const float* axes, size_t axes_size,
                                         const int32_t* buttons,
                                         size_t buttons_size) {
  ControllerData controller = {0};

  if (axes == NULL || buttons == NULL || axes_size <= OM_CONTROLLER_AXIS_R2 ||
      buttons_size <= OM_CONTROLLER_BUTTON_INDEX_OPTIONS) {
    return controller;
  }

  controller.left_x = axes[OM_CONTROLLER_AXIS_LEFT_X];
  controller.left_y = axes[OM_CONTROLLER_AXIS_LEFT_Y];
  controller.right_x = axes[OM_CONTROLLER_AXIS_RIGHT_X];
  controller.right_y = axes[OM_CONTROLLER_AXIS_RIGHT_Y];
  controller.l2_trigger = axes[OM_CONTROLLER_AXIS_L2];
  controller.r2_trigger = axes[OM_CONTROLLER_AXIS_R2];

  if (buttons[OM_CONTROLLER_BUTTON_INDEX_CIRCLE]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_CIRCLE;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_CROSS]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_CROSS;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_SQUARE]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_SQUARE;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_TRIANGLE]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_TRIANGLE;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_L1]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_L1;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_R1]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_R1;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_L3]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_L3;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_R3]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_R3;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_SHARE]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_SHARE;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_OPTIONS]) {
    controller.buttons |= OM_CONTROLLER_BUTTON_OPTIONS;
  }

  if (buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_UP]) {
    controller.dpad |= OM_CONTROLLER_DPAD_UP;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_DOWN]) {
    controller.dpad |= OM_CONTROLLER_DPAD_DOWN;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_LEFT]) {
    controller.dpad |= OM_CONTROLLER_DPAD_LEFT;
  }
  if (buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_RIGHT]) {
    controller.dpad |= OM_CONTROLLER_DPAD_RIGHT;
  }

  return controller;
}

bool om_ctrl_data_to_ros_joy(const ControllerData* data, float* axes,
                             size_t axes_size, int32_t* buttons,
                             size_t buttons_size) {
  if (data == NULL || axes == NULL || buttons == NULL ||
      axes_size <= OM_CONTROLLER_AXIS_R2 ||
      buttons_size <= OM_CONTROLLER_BUTTON_INDEX_OPTIONS) {
    return false;
  }

  axes[OM_CONTROLLER_AXIS_LEFT_X] = data->left_x;
  axes[OM_CONTROLLER_AXIS_LEFT_Y] = data->left_y;
  axes[OM_CONTROLLER_AXIS_RIGHT_X] = data->right_x;
  axes[OM_CONTROLLER_AXIS_RIGHT_Y] = data->right_y;
  axes[OM_CONTROLLER_AXIS_L2] = data->l2_trigger;
  axes[OM_CONTROLLER_AXIS_R2] = data->r2_trigger;

  buttons[OM_CONTROLLER_BUTTON_INDEX_CIRCLE] =
      (data->buttons & OM_CONTROLLER_BUTTON_CIRCLE) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_CROSS] =
      (data->buttons & OM_CONTROLLER_BUTTON_CROSS) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_SQUARE] =
      (data->buttons & OM_CONTROLLER_BUTTON_SQUARE) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_TRIANGLE] =
      (data->buttons & OM_CONTROLLER_BUTTON_TRIANGLE) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_L1] =
      (data->buttons & OM_CONTROLLER_BUTTON_L1) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_R1] =
      (data->buttons & OM_CONTROLLER_BUTTON_R1) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_L3] =
      (data->buttons & OM_CONTROLLER_BUTTON_L3) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_R3] =
      (data->buttons & OM_CONTROLLER_BUTTON_R3) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_SHARE] =
      (data->buttons & OM_CONTROLLER_BUTTON_SHARE) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_OPTIONS] =
      (data->buttons & OM_CONTROLLER_BUTTON_OPTIONS) != 0 ? 1 : 0;

  buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_UP] =
      (data->dpad & OM_CONTROLLER_DPAD_UP) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_DOWN] =
      (data->dpad & OM_CONTROLLER_DPAD_DOWN) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_LEFT] =
      (data->dpad & OM_CONTROLLER_DPAD_LEFT) != 0 ? 1 : 0;
  buttons[OM_CONTROLLER_BUTTON_INDEX_DPAD_RIGHT] =
      (data->dpad & OM_CONTROLLER_DPAD_RIGHT) != 0 ? 1 : 0;

  return true;
}
