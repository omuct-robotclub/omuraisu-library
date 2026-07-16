#ifndef FP_CORE_H
#define FP_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FP_MOTOR_NUM 4U

typedef struct {
  int32_t encoder;
  uint32_t adc;
} FpData;

typedef struct {
  uint32_t tx_id_;
  int16_t output_[FP_MOTOR_NUM];
  FpData data_[FP_MOTOR_NUM];
} FpCore;

FpData om_fp_data_init();
void om_fp_data_parse(FpData* data, const uint8_t raw[8]);

FpCore om_fp_core_init(uint32_t tx_id);

int om_fp_core_parse(FpCore* core, uint32_t id, const uint8_t data[8],
                     uint8_t len);

void om_fp_core_set_pwm(FpCore* core, int16_t pwm, int id);
void om_fp_core_get_output(const FpCore* core, uint8_t out[8]);

int32_t om_fp_core_get_encoder(const FpCore* core, int id);
uint32_t om_fp_core_get_adc(const FpCore* core, int id);
FpData om_fp_core_get_data(const FpCore* core, int id);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FP_H
