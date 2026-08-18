#include "dji/robomas_core.h"

#include <stdint.h>
#include <string.h>

const uint16_t M3508_GEAR_RATIO = 19;
const uint16_t M2006_GEAR_RATIO = 36;
const uint16_t ANGLE_MAX_VALUE = 8192;

static const uint32_t RX_ID_MIN = 0x201;
static const uint32_t RX_ID_MAX = 0x208;

const uint32_t TX_ID_GROUP1 = 0x200;
const uint32_t TX_ID_GROUP2 = 0x1FF;

RobomasData om_rm_data_init() {
  RobomasData data;
  data.angle = 0;
  data.rpm = 0;
  data.current = 0;
  data.temp = 0;
  return data;
}

void om_rm_data_parse(RobomasData* data, const uint8_t raw[8]) {
  data->angle = ((uint16_t)raw[0] << 8) | (uint16_t)raw[1];
  data->rpm = ((int16_t)raw[2] << 8) | (int16_t)raw[3];
  data->current = ((int16_t)raw[4] << 8) | (int16_t)raw[5];
  data->temp = raw[6];
}

RobomasCore om_rm_core_init() {
  RobomasCore core;
  for (int i = 0; i < 8; ++i) {
    core.data_[i] = om_rm_data_init();
  }
  core.max_output_ = 16384;
  memset(core.output_, 0, sizeof(core.output_));
  return core;
}

void om_rm_core_set_max_output(RobomasCore* core, int16_t max) {
  if (max == INT16_MIN) {
    core->max_output_ =
        INT16_MAX;  // INT16_MINは絶対値がINT16_MAXより大きいため、特別に処理
    return;
  }
  core->max_output_ = max > 0 ? max : -max;
}

int om_rm_core_parse(RobomasCore* core, uint32_t id, const uint8_t data[8]) {
  if (id < RX_ID_MIN || id > RX_ID_MAX) {
    return -1;
  }
  int index = id - RX_ID_MIN;
  om_rm_data_parse(&core->data_[index], data);
  return index;
}

void om_rm_core_set_output(RobomasCore* core, int16_t current, int id) {
  if (id < 1 || id > 8) {
    return;
  }
  int index = id - 1;
  int16_t clamped_current = current;
  if (clamped_current > core->max_output_) {
    clamped_current = core->max_output_;
  } else if (clamped_current < -core->max_output_) {
    clamped_current = -core->max_output_;
  }
  const unsigned int group = index / 4;
  core->output_[group][(index % 4) * 2] = (clamped_current >> 8) & 0xFF;
  core->output_[group][(index % 4) * 2 + 1] = clamped_current & 0xFF;
}

void om_rm_core_set_output_percent(RobomasCore* core, float percent, int id) {
  if (id < 1 || id > 8) {
    return;
  }
  int16_t current = (int16_t)(percent * core->max_output_);
  om_rm_core_set_output(core, current, id);
}

void om_rm_core_get_output(const RobomasCore* core, uint8_t out[2][8]) {
  memcpy(out, core->output_, sizeof(core->output_));
}

void om_rm_core_get_output_group(const RobomasCore* core, uint8_t out[8],
                                 const unsigned int group) {
  if (group > 1) {
    memset(out, 0, 8);
    return;
  }
  memcpy(out, core->output_[group], 8);
}

int16_t om_rm_core_get_current(const RobomasCore* core, int id) {
  if (id < 1 || id > 8) {
    return 0;
  }
  int index = id - 1;
  return core->data_[index].current;
}

uint16_t om_rm_core_get_angle(const RobomasCore* core, int id) {
  if (id < 1 || id > 8) {
    return 0;
  }
  int index = id - 1;
  return core->data_[index].angle;
}

int16_t om_rm_core_get_rpm(const RobomasCore* core, int id) {
  if (id < 1 || id > 8) {
    return 0;
  }
  int index = id - 1;
  return core->data_[index].rpm;
}

uint8_t om_rm_core_get_temp(const RobomasCore* core, int id) {
  if (id < 1 || id > 8) {
    return 0;
  }
  int index = id - 1;
  return core->data_[index].temp;
}

RobomasData om_rm_core_get_data(const RobomasCore* core, int id) {
  if (id < 1 || id > 8) {
    return om_rm_data_init();
  }
  int index = id - 1;
  return core->data_[index];
}