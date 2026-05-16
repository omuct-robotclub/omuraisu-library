#include "vesc/vesc_core.h"

#include <stdint.h>
#include <string.h>

const uint8_t VESC_ID_MIN = 1;
const uint8_t VESC_ID_MAX = 8;

const uint32_t VESC_CAN_PACKET_SET_DUTY = 0;
const uint32_t VESC_CAN_PACKET_SET_CURRENT = 1;
const uint32_t VESC_CAN_PACKET_SET_CURRENT_BRAKE = 2;
const uint32_t VESC_CAN_PACKET_SET_RPM = 3;
const uint32_t VESC_CAN_PACKET_STATUS = 9;

static int16_t om_vesc_get_i16(const uint8_t raw[8], int index) {
  return (int16_t)(((uint16_t)raw[index] << 8) | (uint16_t)raw[index + 1]);
}

static int32_t om_vesc_get_i32(const uint8_t raw[8], int index) {
  return (int32_t)(((uint32_t)raw[index] << 24) |
                   ((uint32_t)raw[index + 1] << 16) |
                   ((uint32_t)raw[index + 2] << 8) | (uint32_t)raw[index + 3]);
}

static void om_vesc_set_i32(uint8_t out[8], int32_t value) {
  out[0] = (uint8_t)((uint32_t)value >> 24);
  out[1] = (uint8_t)((uint32_t)value >> 16);
  out[2] = (uint8_t)((uint32_t)value >> 8);
  out[3] = (uint8_t)value;
}

static float om_vesc_abs_float(float value) {
  return value > 0.0f ? value : -value;
}

static float om_vesc_clamp_float(float value, float max) {
  if (value > max) {
    return max;
  } else if (value < -max) {
    return -max;
  }
  return value;
}

static int om_vesc_id_to_index(int id) {
  if (id < VESC_ID_MIN || id > VESC_ID_MAX) {
    return -1;
  }
  return id - 1;
}

static void om_vesc_core_set_output_i32(VescCore* core, uint32_t packet_id,
                                        int32_t value, int id) {
  if (om_vesc_id_to_index(id) < 0) {
    return;
  }
  core->output_.id = (packet_id << 8) | (uint32_t)id;
  core->output_.len = 4;
  memset(core->output_.data, 0, sizeof(core->output_.data));
  om_vesc_set_i32(core->output_.data, value);
}

VescData om_vesc_data_init() {
  VescData data;
  data.rpm = 0;
  data.current_x10 = 0;
  data.duty_x1000 = 0;
  return data;
}

void om_vesc_data_parse(VescData* data, const uint8_t raw[8]) {
  data->rpm = om_vesc_get_i32(raw, 0);
  data->current_x10 = om_vesc_get_i16(raw, 4);
  data->duty_x1000 = om_vesc_get_i16(raw, 6);
}

VescTxFrame om_vesc_tx_frame_init() {
  VescTxFrame frame;
  frame.id = 0;
  frame.len = 0;
  memset(frame.data, 0, sizeof(frame.data));
  return frame;
}

VescCore om_vesc_core_init() {
  VescCore core;
  for (int i = 0; i < 8; ++i) {
    core.data_[i] = om_vesc_data_init();
  }
  core.output_ = om_vesc_tx_frame_init();
  core.max_id_ = VESC_ID_MAX;
  core.max_current_ = 30.0f;
  core.max_duty_ = 1.0f;
  return core;
}

void om_vesc_core_set_max_current(VescCore* core, float max) {
  core->max_current_ = om_vesc_abs_float(max);
}

void om_vesc_core_set_max_duty(VescCore* core, float max) {
  core->max_duty_ = om_vesc_abs_float(max);
  if (core->max_duty_ > 1.0f) {
    core->max_duty_ = 1.0f;
  }
}

int om_vesc_core_parse(VescCore* core, uint32_t id, const uint8_t data[8]) {
  const uint32_t packet_id = (id >> 8) & 0xFFU;
  const int controller_id = (int)(id & 0xFFU);
  const int index = om_vesc_id_to_index(controller_id);

  if (packet_id != VESC_CAN_PACKET_STATUS || index < 0) {
    return -1;
  }

  om_vesc_data_parse(&core->data_[index], data);
  return index;
}

void om_vesc_core_set_current(VescCore* core, float current, int id) {
  const float clamped_current =
      om_vesc_clamp_float(current, core->max_current_);
  om_vesc_core_set_output_i32(core, VESC_CAN_PACKET_SET_CURRENT,
                              (int32_t)(clamped_current * 1000.0f), id);
}

void om_vesc_core_set_current_percent(VescCore* core, float percent, int id) {
  om_vesc_core_set_current(core, percent * core->max_current_, id);
}

void om_vesc_core_set_brake_current(VescCore* core, float current, int id) {
  const float clamped_current =
      om_vesc_clamp_float(current, core->max_current_);
  om_vesc_core_set_output_i32(core, VESC_CAN_PACKET_SET_CURRENT_BRAKE,
                              (int32_t)(clamped_current * 1000.0f), id);
}

void om_vesc_core_set_duty(VescCore* core, float duty, int id) {
  const float clamped_duty = om_vesc_clamp_float(duty, core->max_duty_);
  om_vesc_core_set_output_i32(core, VESC_CAN_PACKET_SET_DUTY,
                              (int32_t)(clamped_duty * 100000.0f), id);
}

void om_vesc_core_set_duty_percent(VescCore* core, float percent, int id) {
  om_vesc_core_set_duty(core, percent * core->max_duty_, id);
}

void om_vesc_core_set_rpm(VescCore* core, int32_t rpm, int id) {
  om_vesc_core_set_output_i32(core, VESC_CAN_PACKET_SET_RPM, rpm, id);
}

void om_vesc_core_get_output(const VescCore* core, uint32_t* id, uint8_t out[8],
                             uint8_t* len) {
  if (id != 0) {
    *id = core->output_.id;
  }
  if (len != 0) {
    *len = core->output_.len;
  }
  memcpy(out, core->output_.data, 8);
}

int32_t om_vesc_core_get_rpm(const VescCore* core, int id) {
  const int index = om_vesc_id_to_index(id);
  if (index < 0) {
    return 0;
  }
  return core->data_[index].rpm;
}

int16_t om_vesc_core_get_current_x10(const VescCore* core, int id) {
  const int index = om_vesc_id_to_index(id);
  if (index < 0) {
    return 0;
  }
  return core->data_[index].current_x10;
}

int16_t om_vesc_core_get_duty_x1000(const VescCore* core, int id) {
  const int index = om_vesc_id_to_index(id);
  if (index < 0) {
    return 0;
  }
  return core->data_[index].duty_x1000;
}

VescData om_vesc_core_get_data(const VescCore* core, int id) {
  const int index = om_vesc_id_to_index(id);
  if (index < 0) {
    return om_vesc_data_init();
  }
  return core->data_[index];
}
