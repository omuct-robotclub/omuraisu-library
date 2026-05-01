#include "sensor/amt21/amt21_core.h"
bool om_amt21_calc_checksum(uint16_t data) {
  bool k0 = data >> 15;
  bool k1 = data >> 14 & 1;
  do {
    k1 ^= data & 0x8000;          // even
    k0 ^= (data <<= 1) & 0x8000;  // odd
  } while (data <<= 1);
  return k0 && k1;
}
bool om_amt21_set_pos(Amt21Data* data, const uint8_t raw[2], int resolution) {
  if (data == NULL || raw == NULL) {
    return false;
  }
  if (resolution != AMT21_12BIT_RESOLUTION &&
      resolution != AMT21_14BIT_RESOLUTION) {
    return false;
  }
  uint16_t raw_data = ((uint16_t)raw[1] << 8) | raw[0];
  if (!om_amt21_calc_checksum(raw_data)) {
    return false;
  }
  data->raw_pos = AMT21_12BIT_RESOLUTION == resolution
                      ? ((raw_data >> 2) & 0x0FFF)
                      : (raw_data & 0x3FFF);
  return true;
}
bool om_amt21_set_turn(Amt21Data* data, const uint8_t raw[2]) {
  if (data == NULL || raw == NULL) {
    return false;
  }
  uint16_t raw_data = ((uint16_t)raw[1] << 8) | raw[0];
  if (!om_amt21_calc_checksum(raw_data)) {
    return false;
  }
  if (raw_data & 0x2000) {
    data->turn =
        (int16_t)(~(raw_data & 0x1FFF) + 1);  // 負の値の場合、2の補数表現に変換
  } else {
    data->turn = (int16_t)(raw_data & 0x1FFF);  // 正の値の場合、そのまま使用
  }
  return true;
}
uint16_t om_amt21_get_pos(const Amt21Data* data) {
  return data == NULL ? 0 : data->raw_pos;
}
int16_t om_amt21_get_turn(const Amt21Data* data) {
  return data == NULL ? 0 : data->turn;
}
void om_amt21_build_cmd(uint8_t cmd[2], size_t* len, Amt21Cmd command,
                        uint8_t address) {
  if (cmd == NULL || len == NULL) {
    return;
  }
  switch (command) {
    case OM_AMT21_READ_POS:
      cmd[0] = address | CMD_READ_POS;
      cmd[1] = 0;
      *len = 1;
      break;
    case OM_AMT21_READ_TURN:
      cmd[0] = address | CMD_READ_TURN;
      cmd[1] = 0;
      *len = 1;
      break;
    case OM_AMT21_RESET:
      cmd[0] = address | CMD_EXTEND;
      cmd[1] = CMD_RESET;
      *len = 2;
      break;
    case OM_AMT21_SET_ZERO_POS:
      cmd[0] = address | CMD_EXTEND;
      cmd[1] = CMD_SET_ZERO_POS;
      *len = 2;
      break;
    default:
      return;  // 無効なコマンド
  }
}
void om_amt21_build_read_pos_cmd(uint8_t cmd[2], size_t* len, uint8_t address) {
  om_amt21_build_cmd(cmd, len, OM_AMT21_READ_POS, address);
}
void om_amt21_build_read_turn_cmd(uint8_t cmd[2], size_t* len,
                                  uint8_t address) {
  om_amt21_build_cmd(cmd, len, OM_AMT21_READ_TURN, address);
}
void om_amt21_build_reset_cmd(uint8_t cmd[2], size_t* len, uint8_t address) {
  om_amt21_build_cmd(cmd, len, OM_AMT21_RESET, address);
}
void om_amt21_build_set_zero_pos_cmd(uint8_t cmd[2], size_t* len,
                                     uint8_t address) {
  om_amt21_build_cmd(cmd, len, OM_AMT21_SET_ZERO_POS, address);
}