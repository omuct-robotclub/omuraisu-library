#include "dji/robomas.h"

Robomas om_rm_init(CanBus* can) {
  Robomas rm;
  rm.can = can;
  rm.core = om_rm_core_init();
  return rm;
}

void om_rm_set_max_output(Robomas* rm, int16_t max) {
  om_rm_core_set_max_output(&rm->core, max);
}

int om_rm_read(Robomas* rm) {
  CanMessage msg;
  if (rm->can->read(rm->can->impl, &msg)) {
    return om_rm_core_parse(&rm->core, msg.id, msg.data);
  }
  return -1;
}

int om_rm_parse(Robomas* rm, uint32_t id, const uint8_t data[8]) {
  return om_rm_core_parse(&rm->core, id, data);
}

void om_rm_set_output(Robomas* rm, int16_t current, int id) {
  om_rm_core_set_output(&rm->core, current, id);
}

void om_rm_set_output_percent(Robomas* rm, float percent, int id) {
  om_rm_core_set_output_percent(&rm->core, percent, id);
}

bool om_rm_write(Robomas* rm) {
  CanMessage msg1, msg2;
  msg1.id = TX_ID_GROUP1;
  msg2.id = TX_ID_GROUP2;
  om_rm_core_get_output_group(&rm->core, msg1.data, 0);
  om_rm_core_get_output_group(&rm->core, msg2.data, 1);
  msg1.len = 8;
  msg2.len = 8;
  bool success1 = rm->can->write(rm->can->impl, &msg1);
  bool success2 = rm->can->write(rm->can->impl, &msg2);
  return success1 && success2;
}

int16_t om_rm_get_current(const Robomas* rm, int id) {
  return om_rm_core_get_current(&rm->core, id);
}

uint16_t om_rm_get_angle(const Robomas* rm, int id) {
  return om_rm_core_get_angle(&rm->core, id);
}

int32_t om_rm_get_rotation_count(const Robomas* rm, int id) {
  return om_rm_core_get_rotation_count(&rm->core, id);
}

float om_rm_get_total_angle(const Robomas* rm, int id) {
  return om_rm_core_get_total_angle(&rm->core, id);
}

int16_t om_rm_get_rpm(const Robomas* rm, int id) {
  return om_rm_core_get_rpm(&rm->core, id);
}

uint8_t om_rm_get_temp(const Robomas* rm, int id) {
  return om_rm_core_get_temp(&rm->core, id);
}

RobomasData om_rm_get_data(const Robomas* rm, int id) {
  return om_rm_core_get_data(&rm->core, id);
}

const RobomasData* om_rm_get_data_const(const Robomas* rm, int id) {
  if (id < 1 || id > 8) {
    return NULL;  // IDが範囲外の場合はNULLを返す
  }
  return &rm->core.data_[id - 1];
}

RobomasCore om_rm_get_core(const Robomas* rm) {
  return rm->core;
}

const RobomasCore* om_rm_get_core_const(const Robomas* rm) {
  return &rm->core;
}