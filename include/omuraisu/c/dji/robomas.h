#ifndef ROBOMAS_H
#define ROBOMAS_H

#include <stddef.h>

#include "can/can_interface.h"
#include "robomas_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  CanBus* can;
  RobomasCore core;
} Robomas;

Robomas om_rm_init(CanBus* can);

void om_rm_set_max_output(Robomas* rm, int16_t max);

int om_rm_read(Robomas* rm);

int om_rm_parse(Robomas* rm, uint32_t id, const uint8_t data[8]);

void om_rm_set_output(Robomas* rm, int16_t current, int id);

void om_rm_set_output_percent(Robomas* rm, float percent, int id);

bool om_rm_write(Robomas* rm);

int16_t om_rm_get_current(const Robomas* rm, int id);

uint16_t om_rm_get_angle(const Robomas* rm, int id);

int16_t om_rm_get_rpm(const Robomas* rm, int id);

uint8_t om_rm_get_temp(const Robomas* rm, int id);

RobomasData om_rm_get_data(const Robomas* rm, int id);

const RobomasData* om_rm_get_data_const(const Robomas* rm, int id);

RobomasCore om_rm_get_core(const Robomas* rm);

const RobomasCore* om_rm_get_core_const(const Robomas* rm);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif  // ROBOMAS_H