#include "fp/fp_core.h"

#include <string.h>

FpData om_fp_data_init() {
  FpData data = {0};
  return data;
}

void om_fp_data_parse(FpData* data, const uint8_t raw[8]) {
  memcpy(&data->encoder, &raw[0], sizeof(data->encoder));
  memcpy(&data->adc, &raw[4], sizeof(data->adc));
}

FpCore om_fp_core_init(uint32_t tx_id) {
  FpCore core;
  for (int i = 0; i < FP_MOTOR_NUM; ++i) {
    core.data_[i] = om_fp_data_init();
  }
  core.tx_id_ = tx_id;
  memset(core.output_, 0, sizeof(core.output_));
  return core;
}

int om_fp_core_parse(FpCore* core, uint32_t id, const uint8_t data[8],
                     uint8_t len) {
  if (len != sizeof(core->data_[0])) {
    return -1;
  }

  if (id <= core->tx_id_ || id > core->tx_id_ + FP_MOTOR_NUM) {
    return -1;
  }

  const uint32_t index = id - core->tx_id_ - 1U;
  om_fp_data_parse(&core->data_[index], data);
  return (int)index;
}

void om_fp_core_set_pwm(FpCore* core, int16_t pwm, int id) {
  if (id < 1 || id > FP_MOTOR_NUM) {
    return;
  }

  core->output_[id - 1] = pwm;
}

void om_fp_core_get_output(const FpCore* core, uint8_t out[8]) {
  memcpy(out, core->output_, sizeof(core->output_));
}

int32_t om_fp_core_get_encoder(const FpCore* core, int id) {
  if (id < 1 || id > FP_MOTOR_NUM) {
    return 0;
  }
  return core->data_[id - 1].encoder;
}

uint32_t om_fp_core_get_adc(const FpCore* core, int id) {
  if (id < 1 || id > FP_MOTOR_NUM) {
    return 0;
  }
  return core->data_[id - 1].adc;
}

FpData om_fp_core_get_data(const FpCore* core, int id) {
  if (id < 1 || id > FP_MOTOR_NUM) {
    return om_fp_data_init();
  }
  return core->data_[id - 1];
}
