#ifndef CONTROLLER_CORE_HPP
#define CONTROLLER_CORE_HPP

#include <cstdint>
#include <cstring>

namespace controller {

/// @brief PS5コントローラーのボタンビットフラグ
enum class Button : uint16_t {
  Circle = 0x0001,    ///< × (bit 0)
  Cross = 0x0002,     ///< ○ (bit 1)
  Square = 0x0004,    ///< □ (bit 2)
  Triangle = 0x0008,  ///< △ (bit 3)
  L1 = 0x0010,        ///< L1 (bit 4)
  R1 = 0x0020,        ///< R1 (bit 5)
  L3 = 0x0040,        ///< L3 (bit 6)
  R3 = 0x0080,        ///< R3 (bit 7)
  Share = 0x0100,     ///< Share (bit 8)
  Options = 0x0200,   ///< Options (bit 9)
};

/// @brief D-Padのビットフラグ
enum class DPad : uint8_t {
  Up = 0x01,     ///< 上
  Right = 0x02,  ///< 右
  Down = 0x04,   ///< 下
  Left = 0x08,   ///< 左
};

/// @brief PS5コントローラーデータ（プラットフォーム非依存）
struct ControllerData {
  // アナログスティック (-1.0〜1.0, 中央=0.0)
  float left_x = 0;
  float left_y = 0;
  float right_x = 0;
  float right_y = 0;

  // トリガー (0-255)
  float l2_trigger = 0;
  float r2_trigger = 0;

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
  bool is_pressed(DPad dpad_dir) const {
    return (dpad & static_cast<uint8_t>(dpad_dir)) != 0;
  }

  /// @brief D-Padの方向を取得
  DPad get_dpad() const { return static_cast<DPad>(dpad); }

  /// @brief D-Padの上が押されているか
  bool up() const { return is_pressed(DPad::Up); }

  /// @brief D-Padの下が押されているか
  bool down() const { return is_pressed(DPad::Down); }

  /// @brief D-Padの左が押されているか
  bool left() const { return is_pressed(DPad::Left); }

  /// @brief D-Padの右が押されているか
  bool right() const { return is_pressed(DPad::Right); }

  // ボタンヘルパー
  bool cross() const { return is_pressed(Button::Cross); }
  bool circle() const { return is_pressed(Button::Circle); }
  bool square() const { return is_pressed(Button::Square); }
  bool triangle() const { return is_pressed(Button::Triangle); }
  bool l1() const { return is_pressed(Button::L1); }
  bool r1() const { return is_pressed(Button::R1); }
  bool l3() const { return is_pressed(Button::L3); }
  bool r3() const { return is_pressed(Button::R3); }
  bool share() const { return is_pressed(Button::Share); }
  bool options() const { return is_pressed(Button::Options); }
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
          if (data[1] & 0x08) btns |= static_cast<uint16_t>(Button::Circle);
          if (data[1] & 0x04) btns |= static_cast<uint16_t>(Button::Triangle);
          if (data[1] & 0x02) btns |= static_cast<uint16_t>(Button::Square);
          if (data[1] & 0x01) btns |= static_cast<uint16_t>(Button::Cross);
          if (data[2]) btns |= static_cast<uint16_t>(Button::L1);
          if (data[3]) btns |= static_cast<uint16_t>(Button::R1);
          if (data[4]) btns |= static_cast<uint16_t>(Button::L3);
          if (data[5]) btns |= static_cast<uint16_t>(Button::R3);
          if (data[6]) btns |= static_cast<uint16_t>(Button::Options);
          if (data[7]) btns |= static_cast<uint16_t>(Button::Share);
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
/// @brief ROS2 sensor_msgs/msg/Joy互換のパーサー（ROS2非依存）
/// @details axes[]とbuttons[]を受け取りControllerDataに変換する
class RosJoyParser {
 public:
  /// @brief 軸のインデックスマッピング（PS5コントローラー用）
  enum class AxisIndex : uint8_t {
    LeftX = 0,
    LeftY = 1,
    RightX = 2,
    RightY = 3,
    L2 = 4,
    R2 = 5,
  };

  /// @brief ボタンのインデックスマッピング（PS5コントローラー用）
  enum class ButtonIndex : uint8_t {
    Circle = 0,
    Cross = 1,
    Square = 2,
    Triangle = 3,
    DPadUp = 4,
    DPadDown = 5,
    DPadLeft = 6,
    DPadRight = 7,
    L1 = 8,
    R1 = 9,
    L3 = 10,
    R3 = 11,
    Share = 12,
    Options = 13,
  };

  static constexpr size_t kMinAxesSize = 6;
  static constexpr size_t kMinButtonsSize = 14;

  /// @brief Joyメッセージからパースする（配列ポインタ版）
  /// @param axes 軸の配列（少なくとも6要素）
  /// @param axes_size 軸配列のサイズ
  /// @param buttons ボタンの配列（少なくとも14要素）
  /// @param buttons_size ボタン配列のサイズ
  /// @return パース成功ならtrue
  bool parse(const float* axes, size_t axes_size, const int32_t* buttons,
             size_t buttons_size) {
    if (axes_size < kMinAxesSize || buttons_size < kMinButtonsSize) {
      return false;
    }

    // スティック入力（-1.0〜1.0）
    data_.left_x = axes[static_cast<uint8_t>(AxisIndex::LeftX)];
    data_.left_y = axes[static_cast<uint8_t>(AxisIndex::LeftY)];
    data_.right_x = axes[static_cast<uint8_t>(AxisIndex::RightX)];
    data_.right_y = axes[static_cast<uint8_t>(AxisIndex::RightY)];

    // トリガー
    data_.l2_trigger = (axes[static_cast<uint8_t>(AxisIndex::L2)]);
    data_.r2_trigger = (axes[static_cast<uint8_t>(AxisIndex::R2)]);

    // ボタン（ビットフラグに変換）
    uint16_t btns = 0;
    if (buttons[static_cast<uint8_t>(ButtonIndex::Circle)])
      btns |= static_cast<uint16_t>(Button::Circle);
    if (buttons[static_cast<uint8_t>(ButtonIndex::Cross)])
      btns |= static_cast<uint16_t>(Button::Cross);
    if (buttons[static_cast<uint8_t>(ButtonIndex::Square)])
      btns |= static_cast<uint16_t>(Button::Square);
    if (buttons[static_cast<uint8_t>(ButtonIndex::Triangle)])
      btns |= static_cast<uint16_t>(Button::Triangle);
    if (buttons[static_cast<uint8_t>(ButtonIndex::L1)])
      btns |= static_cast<uint16_t>(Button::L1);
    if (buttons[static_cast<uint8_t>(ButtonIndex::R1)])
      btns |= static_cast<uint16_t>(Button::R1);
    if (buttons[static_cast<uint8_t>(ButtonIndex::L3)])
      btns |= static_cast<uint16_t>(Button::L3);
    if (buttons[static_cast<uint8_t>(ButtonIndex::R3)])
      btns |= static_cast<uint16_t>(Button::R3);
    if (buttons[static_cast<uint8_t>(ButtonIndex::Share)])
      btns |= static_cast<uint16_t>(Button::Share);
    if (buttons[static_cast<uint8_t>(ButtonIndex::Options)])
      btns |= static_cast<uint16_t>(Button::Options);
    data_.buttons = btns;

    // D-Pad（ビットフラグに変換）
    uint8_t dpad = 0;
    if (buttons[static_cast<uint8_t>(ButtonIndex::DPadUp)])
      dpad |= static_cast<uint8_t>(DPad::Up);
    if (buttons[static_cast<uint8_t>(ButtonIndex::DPadDown)])
      dpad |= static_cast<uint8_t>(DPad::Down);
    if (buttons[static_cast<uint8_t>(ButtonIndex::DPadLeft)])
      dpad |= static_cast<uint8_t>(DPad::Left);
    if (buttons[static_cast<uint8_t>(ButtonIndex::DPadRight)])
      dpad |= static_cast<uint8_t>(DPad::Right);
    data_.dpad = dpad;

    return true;
  }

  /// @brief Joyメッセージからパースする（テンプレート版）
  /// @tparam JoyMsg
  /// axes()とbuttons()メソッドを持つ型（sensor_msgs::msg::Joy互換）
  /// @param joy_msg Joyメッセージ
  /// @return パース成功ならtrue
  template <typename JoyMsg>
  bool parse(const JoyMsg& joy_msg) {
    return parse(joy_msg.axes.data(), joy_msg.axes.size(),
                 joy_msg.buttons.data(), joy_msg.buttons.size());
  }

  /// @brief コントローラーデータを取得
  const ControllerData& data() const { return data_; }
  ControllerData& data() { return data_; }

 private:
  ControllerData data_;
};
}  // namespace controller

#endif  // CONTROLLER_CORE_HPP
