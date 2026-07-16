#include "fp/fp_core.h"

#include <string.h>

FpData om_fp_data_init() {
  FpData data = {0};
  return data;
}

void om_fp_data_parse(FpData* data, const uint8_t raw[8]) {
  memcpy(&data->enc, &raw[0], sizeof(data->enc));
  memcpy(&data->adc, &raw[4], sizeof(data->adc));
}

Fp om_fp_core_init(uint32_t send_id) {
  Fp fp;
  for (int i = 0; i < FP_MOTOR_MAX; ++i) {
    fp.data_[i] = om_fp_data_init();
  }
  fp.send_id_ = send_id;
  memset(fp.output_, 0, sizeof(fp.output_));
  return fp;
}

int om_fp_core_parse(Fp* fp, uint32_t id, const uint8_t data[8], uint8_t len) {
  if (len != sizeof(fp->data_[0])) {
    return -1;
  }

  if (id <= fp->send_id_ || id > fp->send_id_ + FP_MOTOR_MAX) {
    return -1;
  }

  const uint32_t index = id - fp->send_id_ - 1U;
  om_fp_data_parse(&fp->data_[index], data);
  return (int)index;
}

void om_fp_core_set_pwm(Fp* fp, int16_t pwm, int id) {
  if (id < 1 || id > FP_MOTOR_MAX) {
    return;
  }

  fp->output_[id - 1] = pwm;
}

void om_fp_core_get_output(const Fp* fp, uint8_t out[8]) {
  memcpy(out, fp->output_, sizeof(fp->output_));
}

int32_t om_fp_core_get_enc(const Fp* fp, int id) {
  if (id < 1 || id > FP_MOTOR_MAX) {
    return 0;
  }
  return fp->data_[id - 1].enc;
}

uint32_t om_fp_core_get_adc(const Fp* fp, int id) {
  if (id < 1 || id > FP_MOTOR_MAX) {
    return 0;
  }
  return fp->data_[id - 1].adc;
}

FpData om_fp_core_get_data(const Fp* fp, int id) {
  if (id < 1 || id > FP_MOTOR_MAX) {
    return om_fp_data_init();
  }
  return fp->data_[id - 1];
}
