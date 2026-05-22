#ifndef OMURAISU_C_SENSOR_AMT21_AMT21_CORE_H
#define OMURAISU_C_SENSOR_AMT21_AMT21_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AMT21_12BIT_RESOLUTION 0
#define AMT21_14BIT_RESOLUTION 1

static const uint8_t CMD_READ_POS = 0x00;
static const uint8_t CMD_READ_TURN = 0x01;
static const uint8_t CMD_EXTEND = 0x02;
static const uint8_t CMD_RESET = 0x75;
static const uint8_t CMD_SET_ZERO_POS = 0x5E;

typedef struct {
  uint16_t raw_pos;
  int16_t turn;
} Amt21Data;

typedef enum {
  OM_AMT21_READ_POS,
  OM_AMT21_READ_TURN,
  OM_AMT21_RESET,
  OM_AMT21_SET_ZERO_POS,
} Amt21Cmd;

bool om_amt21_calc_checksum(const uint16_t data);
bool om_amt21_set_pos(Amt21Data* data, const uint8_t raw[2], int resolution);
bool om_amt21_set_turn(Amt21Data* data, const uint8_t raw[2]);
uint16_t om_amt21_get_pos(const Amt21Data* data);
int16_t om_amt21_get_turn(const Amt21Data* data);
void om_amt21_build_cmd(uint8_t cmd[2], size_t* len, Amt21Cmd command,
                        uint8_t address);
void om_amt21_build_read_pos_cmd(uint8_t cmd[2], size_t* len, uint8_t address);
void om_amt21_build_read_turn_cmd(uint8_t cmd[2], size_t* len, uint8_t address);
void om_amt21_build_reset_cmd(uint8_t cmd[2], size_t* len, uint8_t address);
void om_amt21_build_set_zero_pos_cmd(uint8_t cmd[2], size_t* len,
                                     uint8_t address);
#endif  // OMURAISU_C_SENSOR_AMT21_AMT21_CORE_H