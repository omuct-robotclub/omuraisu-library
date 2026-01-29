#ifndef CONTROLLER_SERIAL_HPP
#define CONTROLLER_SERIAL_HPP

#include <cobs.hpp>
#include <cstring>
#include <vector>

#include "controller_core.hpp"

namespace controller {

/// @brief シリアル（COBS）受信によるコントローラークラス
/// @details COBSエンコードされたシリアルデータを受信してデコードする
class SerialController {
 public:
  SerialController() : buffer_index_(0) {
    std::memset(receive_buffer_, 0, sizeof(receive_buffer_));
  }

  /// @brief 1バイトずつデータを処理
  /// @param byte 受信バイト
  /// @return 完全なフレームをデコード完了したらtrue
  bool process_byte(uint8_t byte) {
    if (byte == 0x00) {
      // デリミタ検出 - フレーム終了
      if (buffer_index_ > 0) {
        // デリミタを追加（COBSデコード用）
        receive_buffer_[buffer_index_++] = 0x00;

        // COBSデコード実行
        decoded_data_ = cobs::decode(receive_buffer_, buffer_index_);

        // 次のフレーム用にリセット
        buffer_index_ = 0;

        if (decoded_data_.size() == sizeof(SerialPacket)) {
          // パケットとして解釈
          const SerialPacket* packet =
              reinterpret_cast<const SerialPacket*>(decoded_data_.data());

          // ヘッダーとチェックサムを検証
          if (packet->header == SerialPacket::HEADER &&
              packet->verify_checksum()) {
            data_ = packet->to_controller_data();
            return true;
          }
        }
      }
      buffer_index_ = 0;
      return false;
    }

    // データ蓄積
    if (buffer_index_ < kMaxBufferSize - 1) {
      receive_buffer_[buffer_index_++] = byte;
    } else {
      // バッファオーバーフロー - リセット
      buffer_index_ = 0;
    }

    return false;
  }

  /// @brief 複数バイトを一度に処理
  /// @param bytes バイト配列
  /// @param len 配列の長さ
  /// @return 完全なフレームをデコード完了したらtrue
  bool process_bytes(const uint8_t* bytes, size_t len) {
    bool result = false;
    for (size_t i = 0; i < len; ++i) {
      if (process_byte(bytes[i])) {
        result = true;  // 最後に成功したフレームを返す
      }
    }
    return result;
  }

  /// @brief コントローラーデータを取得
  const ControllerData& data() const { return data_; }

  /// @brief バッファをリセット
  void reset() {
    buffer_index_ = 0;
    std::memset(receive_buffer_, 0, sizeof(receive_buffer_));
  }

 private:
  static constexpr size_t kMaxBufferSize = 256;

  uint8_t receive_buffer_[kMaxBufferSize];
  std::vector<uint8_t> decoded_data_;
  size_t buffer_index_;
  ControllerData data_;
};

}  // namespace controller

#endif  // CONTROLLER_SERIAL_HPP
