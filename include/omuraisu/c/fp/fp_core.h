#ifndef FP_CORE_H
#define FP_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FP_MOTOR_MAX 4U

typedef struct {
  int32_t enc;
  uint32_t adc;
} FpData;

typedef struct {
  uint32_t send_id_;
  int16_t output_[FP_MOTOR_MAX];
  FpData data_[FP_MOTOR_MAX];
} Fp;

FpData om_fp_data_init();
void om_fp_data_parse(FpData* data, const uint8_t raw[8]);

Fp om_fp_core_init(uint32_t send_id);

int om_fp_core_parse(Fp* fp, uint32_t id, const uint8_t data[8], uint8_t len);

void om_fp_core_set_pwm(Fp* fp, int16_t pwm, int id);
void om_fp_core_get_output(const Fp* fp, uint8_t out[8]);

int32_t om_fp_core_get_enc(const Fp* fp, int id);
uint32_t om_fp_core_get_adc(const Fp* fp, int id);
FpData om_fp_core_get_data(const Fp* fp, int id);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FP_CORE_H
