#ifndef CONTROLLER_CAN_HPP
#define CONTROLLER_CAN_HPP

#include "controller_core.hpp"
#include "dji/can_interface.hpp"

namespace controller {

/// @brief CAN受信によるコントローラークラス
/// @details ICanBusインターフェースを使用してプラットフォーム非依存
class CanController {
 public:
  /// @brief コンストラクタ
  /// @param can CANバスインターフェースへの参照
  explicit CanController(dji::ICanBus& can) : can_(can) {}

  /// @brief CANからデータを読み取り
  /// @return 有効なコントローラーデータを受信したらtrue
  bool read() {
    dji::CanMessage msg;
    if (can_.read(msg)) {
      return parser_.parse(msg.id, msg.data);
    }
    return false;
  }

  /// @brief コントローラーデータを取得
  const ControllerData& data() const { return parser_.data(); }
  ControllerData& data() { return parser_.data(); }

  /// @brief パーサーへの直接アクセス
  CanParser& parser() { return parser_; }
  const CanParser& parser() const { return parser_; }

 private:
  dji::ICanBus& can_;
  CanParser parser_;
};

}  // namespace controller

#endif  // CONTROLLER_CAN_HPP
