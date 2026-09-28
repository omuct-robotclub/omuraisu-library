#ifndef ROBOMAS_CORE_H
#define ROBOMAS_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint16_t M3508_GEAR_RATIO;
extern const uint16_t M2006_GEAR_RATIO;
extern const uint16_t ANGLE_MAX_VALUE;

extern const uint32_t TX_ID_GROUP1;
extern const uint32_t TX_ID_GROUP2;

/// @brief Robomasから受信したモーターデータ
typedef struct {
  uint16_t angle;  // ローター角度[カウント]
  int16_t rpm;
  int16_t current;
  uint8_t temp;
  bool angle_initialized;
  int32_t rotation_count;  // 初回受信からの回転数
  float total_angle;       // 初回受信位置を0とした累積ローター角度[rad]
} RobomasData;

RobomasData om_rm_data_init();

void om_rm_data_parse(RobomasData* data, const uint8_t raw[8]);

typedef struct {
  uint8_t output_[2][8];  // グループ1とグループ2の出力データ
  RobomasData data_[8];   // r受信データ（モーター1-8）
  int16_t max_output_;    // 最大出力値（絶対値）
} RobomasCore;

RobomasCore om_rm_core_init();

void om_rm_core_set_max_output(RobomasCore* core, int16_t max);

int om_rm_core_parse(RobomasCore* core, uint32_t id, const uint8_t data[8]);

void om_rm_core_set_output(RobomasCore* core, int16_t current, int id);

void om_rm_core_set_output_percent(RobomasCore* core, float percent, int id);

void om_rm_core_get_output(const RobomasCore* core, uint8_t out[2][8]);

void om_rm_core_get_output_group(const RobomasCore* core, uint8_t out[8], unsigned int group);

int16_t om_rm_core_get_current(const RobomasCore* core, int id);

uint16_t om_rm_core_get_angle(const RobomasCore* core, int id);

int32_t om_rm_core_get_rotation_count(const RobomasCore* core, int id);

/// @return 初回受信位置を0とした累積ローター角度[rad]。無効なIDでは0。
float om_rm_core_get_total_angle(const RobomasCore* core, int id);

int16_t om_rm_core_get_rpm(const RobomasCore* core, int id);

uint8_t om_rm_core_get_temp(const RobomasCore* core, int id);

RobomasData om_rm_core_get_data(const RobomasCore* core, int id);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // ROBOMAS_CORE_H