#ifndef CONTROLLER_CORE_HPP
#define CONTROLLER_CORE_HPP

#include <cstdint>
#include <cstring>

namespace controller {

/// @brief PS5コントローラーのボタンビットフラグ
enum class Button : uint16_t {
  kCross = 0x0001,     ///< × (bit 0)
  kCircle = 0x0002,    ///< ○ (bit 1)
  kSquare = 0x0004,    ///< □ (bit 2)
  kTriangle = 0x0008,  ///< △ (bit 3)
  kL1 = 0x0010,        ///< L1 (bit 4)
  kR1 = 0x0020,        ///< R1 (bit 5)
  kL3 = 0x0040,        ///< L3 (bit 6)
  kR3 = 0x0080,        ///< R3 (bit 7)
  kShare = 0x0100,     ///< Share (bit 8)
  kOptions = 0x0200,   ///< Options (bit 9)
};

/// @brief D-Padの方向
enum class DPad : uint8_t {
  kNone = 0,       ///< 中央
  kUp = 1,         ///< 上
  kUpRight = 2,    ///< 右上
  kRight = 3,      ///< 右
  kDownRight = 4,  ///< 右下
  kDown = 5,       ///< 下
  kDownLeft = 6,   ///< 左下
  kLeft = 7,       ///< 左
  kUpLeft = 8,     ///< 左上
};

/// @brief PS5コントローラーデータ（プラットフォーム非依存）
struct ControllerData {
  // アナログスティック (-128〜127, 中央=0)
  int8_t left_x = 0;
  int8_t left_y = 0;
  int8_t right_x = 0;
  int8_t right_y = 0;

  // トリガー (0-255)
  uint8_t l2_trigger = 0;
  uint8_t r2_trigger = 0;

  // ボタン (ビットフラグ)
  uint16_t buttons = 0;

  // 方向キー
  uint8_t dpad = 0;

  /// @brief ボタンが押されているか確認
  /// @param button 確認するボタン
  /// @return 押されていればtrue
  bool is_pressed(Button button) const {
    return (buttons & static_cast<uint16_t>(button)) != 0;
  }

  /// @brief D-Padの方向を取得
  DPad get_dpad() const { return static_cast<DPad>(dpad); }

  /// @brief D-Padの上が押されているか
  bool dpad_up() const { return dpad == 1 || dpad == 2 || dpad == 8; }

  /// @brief D-Padの下が押されているか
  bool dpad_down() const { return dpad == 4 || dpad == 5 || dpad == 6; }

  /// @brief D-Padの左が押されているか
  bool dpad_left() const { return dpad == 6 || dpad == 7 || dpad == 8; }

  /// @brief D-Padの右が押されているか
  bool dpad_right() const { return dpad == 2 || dpad == 3 || dpad == 4; }

  // ボタンヘルパー
  bool cross() const { return is_pressed(Button::kCross); }
  bool circle() const { return is_pressed(Button::kCircle); }
  bool square() const { return is_pressed(Button::kSquare); }
  bool triangle() const { return is_pressed(Button::kTriangle); }
  bool l1() const { return is_pressed(Button::kL1); }
  bool r1() const { return is_pressed(Button::kR1); }
  bool l3() const { return is_pressed(Button::kL3); }
  bool r3() const { return is_pressed(Button::kR3); }
  bool share() const { return is_pressed(Button::kShare); }
  bool options() const { return is_pressed(Button::kOptions); }
};

/// @brief シリアル通信用のパケット構造体（COBS用）
struct SerialPacket {
  static constexpr uint8_t HEADER = 0xAA;

  uint8_t header;  ///< 0xAA（固定）
  int8_t left_x;
  int8_t left_y;
  int8_t right_x;
  int8_t right_y;
  uint8_t l2_trigger;
  uint8_t r2_trigger;
  uint16_t buttons;
  uint8_t dpad;
  uint8_t checksum;

  /// @brief チェックサムを計算
  uint8_t calc_checksum() const {
    uint8_t cs = header;
    cs ^= static_cast<uint8_t>(left_x);
    cs ^= static_cast<uint8_t>(left_y);
    cs ^= static_cast<uint8_t>(right_x);
    cs ^= static_cast<uint8_t>(right_y);
    cs ^= l2_trigger;
    cs ^= r2_trigger;
    cs ^= static_cast<uint8_t>(buttons & 0xFF);
    cs ^= static_cast<uint8_t>((buttons >> 8) & 0xFF);
    cs ^= dpad;
    return cs;
  }

  /// @brief チェックサムを検証
  bool verify_checksum() const { return calc_checksum() == checksum; }

  /// @brief ControllerDataに変換
  ControllerData to_controller_data() const {
    ControllerData data;
    data.left_x = left_x;
    data.left_y = left_y;
    data.right_x = right_x;
    data.right_y = right_y;
    data.l2_trigger = l2_trigger;
    data.r2_trigger = r2_trigger;
    data.buttons = buttons;
    data.dpad = dpad;
    return data;
  }
} __attribute__((packed));

/// @brief CAN通信用のパーサー（コアロジック）
class CanParser {
 public:
  /// @brief CAN ID定義
  static constexpr uint32_t CAN_ID_ANALOG = 50;   ///< スティック・トリガー用
  static constexpr uint32_t CAN_ID_BUTTONS = 51;  ///< ボタン用

  /// @brief CANメッセージをパース
  /// @param id CAN ID
  /// @param data 8バイトのCANデータ
  /// @return パースしたIDが有効ならtrue
  bool parse(uint32_t id, const uint8_t data[8]) {
    switch (id) {
      case CAN_ID_ANALOG:
        data_.left_x = static_cast<int8_t>(data[0]);
        data_.left_y = static_cast<int8_t>(data[1]);
        data_.right_x = static_cast<int8_t>(data[2]);
        data_.right_y = static_cast<int8_t>(data[3]);
        data_.l2_trigger = data[4];
        data_.r2_trigger = data[5];
        return true;

      case CAN_ID_BUTTONS:
        // D-Pad (data[0]のビット)
        {
          bool right = (data[0] >> 3) & 1;
          bool up = (data[0] >> 2) & 1;
          bool left = (data[0] >> 1) & 1;
          bool down = data[0] & 1;
          data_.dpad = decode_dpad(up, down, left, right);
        }
        // ボタン (data[1]のビット)
        {
          uint16_t btns = 0;
          if (data[1] & 0x08) btns |= static_cast<uint16_t>(Button::kCircle);
          if (data[1] & 0x04) btns |= static_cast<uint16_t>(Button::kTriangle);
          if (data[1] & 0x02) btns |= static_cast<uint16_t>(Button::kSquare);
          if (data[1] & 0x01) btns |= static_cast<uint16_t>(Button::kCross);
          if (data[2]) btns |= static_cast<uint16_t>(Button::kL1);
          if (data[3]) btns |= static_cast<uint16_t>(Button::kR1);
          if (data[4]) btns |= static_cast<uint16_t>(Button::kL3);
          if (data[5]) btns |= static_cast<uint16_t>(Button::kR3);
          if (data[6]) btns |= static_cast<uint16_t>(Button::kOptions);
          if (data[7]) btns |= static_cast<uint16_t>(Button::kShare);
          data_.buttons = btns;
        }
        return true;

      default:
        return false;
    }
  }

  /// @brief コントローラーデータを取得
  const ControllerData& data() const { return data_; }
  ControllerData& data() { return data_; }

 private:
  ControllerData data_;

  /// @brief 上下左右のフラグからD-Pad値に変換
  static uint8_t decode_dpad(bool up, bool down, bool left, bool right) {
    if (up && right) return 2;
    if (down && right) return 4;
    if (down && left) return 6;
    if (up && left) return 8;
    if (up) return 1;
    if (right) return 3;
    if (down) return 5;
    if (left) return 7;
    return 0;
  }
};

}  // namespace controller

#endif  // CONTROLLER_CORE_HPP
