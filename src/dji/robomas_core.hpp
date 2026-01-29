#ifndef ROBOMAS_CORE_HPP
#define ROBOMAS_CORE_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace dji {

/// @brief Robomasから受信したモーターデータ
struct RobomasData {
  uint16_t angle = 0;
  int16_t rpm = 0;
  int16_t ampere = 0;
  uint8_t temp = 0;

  /// @brief CANメッセージからデータをパース
  /// @param data 8バイトのCANデータ
  void parse(const uint8_t data[8]) {
    angle = static_cast<uint16_t>(data[0] << 8 | data[1]);
    rpm = static_cast<int16_t>(data[2] << 8 | data[3]);
    ampere = static_cast<int16_t>(data[4] << 8 | data[5]);
    temp = data[6];
  }
};

/// @brief Robomas/M3508モーター制御のコアロジック（プラットフォーム非依存）
/// @details CAN通信を含まない純粋なデータ処理クラス。
///          CAN送受信は使用者側で行う。
class RobomasCore {
 public:
  /// @brief CAN ID範囲
  static constexpr uint32_t RX_ID_MIN = 0x201;
  static constexpr uint32_t RX_ID_MAX = 0x208;
  static constexpr uint32_t TX_ID_GROUP1 = 0x200;  // モーター1-4用
  static constexpr uint32_t TX_ID_GROUP2 = 0x1FF;  // モーター5-8用

  RobomasCore() : max_output_(16384) {
    std::memset(output_, 0, sizeof(output_));
  }

  /// @brief 最大出力を設定
  /// @param max 最大出力値（絶対値）
  void set_max_output(int16_t max) { max_output_ = max < 0 ? -max : max; }

  /// @brief 受信したCANメッセージを処理
  /// @param id CAN ID
  /// @param data 8バイトのCANデータ
  /// @return 処理したモーターのインデックス（0-7）、無効なIDの場合は-1
  int parse_received(uint32_t id, const uint8_t data[8]) {
    if (id < RX_ID_MIN || id > RX_ID_MAX) {
      return -1;
    }
    int index = static_cast<int>(id - RX_ID_MIN);
    data_[index].parse(data);
    return index;
  }

  /// @brief モーターへの出力を設定
  /// @param current 出力電流値
  /// @param id モーターID（1-8）
  void set_output(int16_t current, int id) {
    if (id < 1 || id > 8) {
      return;
    }
    int16_t fixed_current =
        std::clamp(static_cast<int>(current), -max_output_, max_output_);

    if (id <= 4) {
      output_[0][(id - 1) * 2] = (fixed_current >> 8) & 0xFF;
      output_[0][(id - 1) * 2 + 1] = fixed_current & 0xFF;
    } else {
      output_[1][(id - 5) * 2] = (fixed_current >> 8) & 0xFF;
      output_[1][(id - 5) * 2 + 1] = fixed_current & 0xFF;
    }
  }

  /// @brief 全モーターへの出力を設定
  /// @param current 8個の出力電流値の配列
  void set_output(const int16_t current[8]) {
    for (int i = 0; i < 8; ++i) {
      set_output(current[i], i + 1);
    }
  }

  /// @brief パーセント指定で出力を設定
  /// @param percent 出力パーセント（-1.0〜1.0）
  /// @param id モーターID（1-8）
  void set_output_percent(float percent, int id) {
    if (id < 1 || id > 8) {
      return;
    }
    int16_t current = static_cast<int16_t>(percent * max_output_);
    set_output(current, id);
  }

  /// @brief 送信用データを取得（グループ1: モーター1-4）
  /// @param[out] id 送信先CAN ID
  /// @param[out] data 8バイトの出力バッファ
  void get_output_group1(uint32_t& id, uint8_t data[8]) const {
    id = TX_ID_GROUP1;
    std::memcpy(data, output_[0], 8);
  }

  /// @brief 送信用データを取得（グループ2: モーター5-8）
  /// @param[out] id 送信先CAN ID
  /// @param[out] data 8バイトの出力バッファ
  void get_output_group2(uint32_t& id, uint8_t data[8]) const {
    id = TX_ID_GROUP2;
    std::memcpy(data, output_[1], 8);
  }

  /// @brief 設定された電流値を取得
  /// @param id モーターID（1-8）
  /// @return 電流値、無効なIDの場合は0
  int16_t get_current(int id) const {
    if (id < 1 || id > 8) {
      return 0;
    }
    if (id <= 4) {
      return static_cast<int16_t>(output_[0][(id - 1) * 2] << 8 |
                                  output_[0][(id - 1) * 2 + 1]);
    } else {
      return static_cast<int16_t>(output_[1][(id - 5) * 2] << 8 |
                                  output_[1][(id - 5) * 2 + 1]);
    }
  }

  /// @brief モーターの角度を取得
  /// @param id モーターID（1-8）
  /// @return 角度値（0-8191）、無効なIDの場合は0
  uint16_t get_angle(int id) const {
    if (id < 1 || id > 8) {
      return 0;
    }
    return data_[id - 1].angle;
  }

  /// @brief モーターの回転数を取得
  /// @param id モーターID（1-8）
  /// @return RPM、無効なIDの場合は0
  int16_t get_rpm(int id) const {
    if (id < 1 || id > 8) {
      return 0;
    }
    return data_[id - 1].rpm;
  }

  /// @brief モーターの電流値を取得
  /// @param id モーターID（1-8）
  /// @return 電流値、無効なIDの場合は0
  int16_t get_ampere(int id) const {
    if (id < 1 || id > 8) {
      return 0;
    }
    return data_[id - 1].ampere;
  }

  /// @brief モーターの温度を取得
  /// @param id モーターID（1-8）
  /// @return 温度（℃）、無効なIDの場合は0
  uint8_t get_temp(int id) const {
    if (id < 1 || id > 8) {
      return 0;
    }
    return data_[id - 1].temp;
  }

  /// @brief モーターデータへの直接アクセス
  /// @param index モーターインデックス（0-7）
  /// @return モーターデータへの参照
  const RobomasData& get_data(int index) const { return data_[index]; }

 private:
  std::array<RobomasData, 8> data_;
  uint8_t output_[2][8];
  int max_output_;
};

}  // namespace dji

#endif  // ROBOMAS_CORE_HPP
