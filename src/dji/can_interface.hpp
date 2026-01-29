#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <cstdint>

namespace dji {

/// @brief プラットフォーム非依存のCANメッセージ構造体
struct CanMessage {
  uint32_t id = 0;
  uint8_t data[8] = {};
  uint8_t len = 8;
};

/// @brief CANバスの抽象インターフェース
/// @details 各プラットフォーム（mbed, Arduino, Linux SocketCANなど）で
///          このインターフェースを実装する。
class ICanBus {
 public:
  virtual ~ICanBus() = default;

  /// @brief CANメッセージを送信
  /// @param msg 送信するメッセージ
  /// @return 送信成功時true
  virtual bool write(const CanMessage& msg) = 0;

  /// @brief CANメッセージを受信（ノンブロッキング）
  /// @param[out] msg 受信したメッセージの格納先
  /// @return メッセージを受信した場合true、受信データがない場合false
  virtual bool read(CanMessage& msg) = 0;
};

}  // namespace dji

#endif  // CAN_INTERFACE_HPP
