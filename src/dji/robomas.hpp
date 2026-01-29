#ifndef ROBOMAS_HPP
#define ROBOMAS_HPP

#include "can_interface.hpp"
#include "robomas_core.hpp"

namespace dji {

/// @brief Robomas/M3508モーター制御クラス（CAN通信込み）
/// @details ICanBusインターフェースを通じてCAN通信を行う。
///          各プラットフォーム用のICanBus実装を渡して使用する。
class Robomas {
 public:
  /// @brief コンストラクタ
  /// @param can CANバスインターフェースへの参照
  explicit Robomas(ICanBus& can) : can_(can) {}

  /// @brief 最大出力を設定
  /// @param max 最大出力値（絶対値）
  void set_max_output(int16_t max) { core_.set_max_output(max); }

  /// @brief Robomasからデータを読み取り
  /// @return 読み取ったモーターのインデックス（0-7）、データなしの場合-1
  int read_data() {
    CanMessage msg;
    if (can_.read(msg)) {
      return core_.parse_received(msg.id, msg.data);
    }
    return -1;
  }

  /// @brief モーターへの出力を設定
  /// @param current 出力電流値
  /// @param id モーターID（1-8）
  void set_output(int16_t current, int id) { core_.set_output(current, id); }

  /// @brief 全モーターへの出力を設定
  /// @param current 8個の出力電流値の配列
  void set_output(const int16_t current[8]) { core_.set_output(current); }

  /// @brief パーセント指定で出力を設定
  /// @param percent 出力パーセント（-1.0〜1.0）
  /// @param id モーターID（1-8）
  void set_output_percent(float percent, int id) {
    core_.set_output_percent(percent, id);
  }

  /// @brief Robomasへ出力を書き込み
  /// @return 両グループの送信が成功した場合true
  bool write() {
    CanMessage msg1, msg2;
    core_.get_output_group1(msg1.id, msg1.data);
    core_.get_output_group2(msg2.id, msg2.data);

    bool success1 = can_.write(msg1);
    bool success2 = can_.write(msg2);

    return success1 && success2;
  }

  /// @brief 設定された電流値を取得
  int16_t get_current(int id) const { return core_.get_current(id); }

  /// @brief モーターの角度を取得
  uint16_t get_angle(int id) const { return core_.get_angle(id); }

  /// @brief モーターの回転数を取得
  int16_t get_rpm(int id) const { return core_.get_rpm(id); }

  /// @brief モーターの電流値を取得
  int16_t get_ampere(int id) const { return core_.get_ampere(id); }

  /// @brief モーターの温度を取得
  uint8_t get_temp(int id) const { return core_.get_temp(id); }

  /// @brief コアロジックへの直接アクセス
  RobomasCore& core() { return core_; }
  const RobomasCore& core() const { return core_; }

 private:
  ICanBus& can_;
  RobomasCore core_;
};

}  // namespace dji

#endif  // ROBOMAS_HPP