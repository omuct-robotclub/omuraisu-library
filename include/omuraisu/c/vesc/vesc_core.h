#ifndef VESC_CORE_H
#define VESC_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VESC_ID_COUNT 256U

extern const uint32_t VESC_CAN_PACKET_SET_DUTY;
extern const uint32_t VESC_CAN_PACKET_SET_CURRENT;
extern const uint32_t VESC_CAN_PACKET_SET_CURRENT_BRAKE;
extern const uint32_t VESC_CAN_PACKET_SET_RPM;
extern const uint32_t VESC_CAN_PACKET_STATUS;

/// @brief VESCから受信したステータスデータ
typedef struct {
  int32_t rpm;
  int16_t current_x10;
  int16_t duty_x1000;
} VescData;

VescData om_vesc_data_init();

void om_vesc_data_parse(VescData* data, const uint8_t raw[8]);

typedef struct {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
} VescTxFrame;

typedef struct {
  VescTxFrame output_;            // 送信用CAN拡張IDとデータ
  VescData data_[VESC_ID_COUNT];  // 受信データ（VESC ID 0-255）
  float max_current_;             // 最大電流指令値（絶対値）
  float max_duty_;                // 最大Duty指令値（絶対値）
} VescCore;

VescTxFrame om_vesc_tx_frame_init();

VescCore om_vesc_core_init();

void om_vesc_core_init_in_place(VescCore* core);

void om_vesc_core_set_max_current(VescCore* core, float max);

void om_vesc_core_set_max_duty(VescCore* core, float max);

int om_vesc_core_parse(VescCore* core, uint32_t id, const uint8_t data[8]);

void om_vesc_core_set_current(VescCore* core, float current, uint8_t id);

void om_vesc_core_set_current_percent(VescCore* core, float percent,
                                      uint8_t id);

void om_vesc_core_set_brake_current(VescCore* core, float current, uint8_t id);

void om_vesc_core_set_duty(VescCore* core, float duty, uint8_t id);

void om_vesc_core_set_duty_percent(VescCore* core, float percent, uint8_t id);

void om_vesc_core_set_rpm(VescCore* core, int32_t rpm, uint8_t id);

void om_vesc_core_get_output(const VescCore* core, uint32_t* id, uint8_t out[8],
                             uint8_t* len);

int32_t om_vesc_core_get_rpm(const VescCore* core, uint8_t id);

int16_t om_vesc_core_get_current_x10(const VescCore* core, uint8_t id);

int16_t om_vesc_core_get_duty_x1000(const VescCore* core, uint8_t id);

VescData om_vesc_core_get_data(const VescCore* core, uint8_t id);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // VESC_CORE_H
