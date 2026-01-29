#ifndef CAN_MBED_HPP
#define CAN_MBED_HPP

// このファイルはmbed環境でのみ使用可能
#ifdef MBED_H

#include "can_interface.hpp"
#include "mbed.h"

namespace dji {

/// @brief mbed用のICanBus実装
class MbedCanBus : public ICanBus {
 public:
  /// @brief コンストラクタ（ピン指定）
  /// @param rx CAN RXピン
  /// @param tx CAN TXピン
  /// @param frequency CANバス周波数（デフォルト1MHz）
  MbedCanBus(PinName rx, PinName tx, int frequency = 1000000)
      : can_(new CAN(rx, tx, frequency)), owned_(true) {}

  /// @brief コンストラクタ（既存CANオブジェクト使用）
  /// @param can 既存のCANオブジェクトへの参照
  explicit MbedCanBus(CAN& can) : can_(&can), owned_(false) {}

  ~MbedCanBus() override {
    if (owned_) {
      delete can_;
    }
  }

  // コピー禁止
  MbedCanBus(const MbedCanBus&) = delete;
  MbedCanBus& operator=(const MbedCanBus&) = delete;

  bool write(const CanMessage& msg) override {
    return can_->write(CANMessage(msg.id, msg.data, msg.len));
  }

  bool read(CanMessage& msg) override {
    CANMessage mbed_msg;
    if (can_->read(mbed_msg)) {
      msg.id = mbed_msg.id;
      msg.len = mbed_msg.len;
      for (int i = 0; i < 8; ++i) {
        msg.data[i] = mbed_msg.data[i];
      }
      return true;
    }
    return false;
  }

  /// @brief 内部のCANオブジェクトへのアクセス
  CAN& get_can() { return *can_; }

 private:
  CAN* can_;
  bool owned_;
};

}  // namespace dji

#endif  // MBED_H
#endif  // CAN_MBED_HPP
