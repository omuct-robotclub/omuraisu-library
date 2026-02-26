# omuraisu-library

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/)

ロボット開発において頻繁に使用される基本的な機能を提供する C++ ライブラリ群です。

PlatformIO / CMake の両方に対応しており、組み込み（mbed）からデスクトップまで幅広い環境で利用できます。

## 目次

- [インストール](#インストール)
- [モジュール一覧](#モジュール一覧)
  - [coordinate — 2D 座標演算](#coordinate--2d-座標演算)
  - [chassis — メカナムホイール制御](#chassis--メカナムホイール制御)
  - [pid — PID 制御](#pid--pid-制御)
  - [dji — DJI ロボマスモーター制御](#dji--dji-ロボマスモーター制御)
  - [can — CAN バス抽象化](#can--can-バス抽象化)
  - [controller — コントローラ入力](#controller--コントローラ入力)
  - [servo — サーボ制御](#servo--サーボ制御)
- [サンプルコード](#サンプルコード)
- [ビルド方法](#ビルド方法)
- [テスト](#テスト)
- [ライセンス](#ライセンス)

---

## インストール

### PlatformIO

`platformio.ini` に以下を追加してください。

```ini
lib_deps = https://github.com/omuct-robotclub/omuraisu-library.git
```

### CMake（FetchContent）

```cmake
include(FetchContent)
FetchContent_Declare(
  omuraisu
  GIT_REPOSITORY https://github.com/omuct-robotclub/omuraisu-library.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(omuraisu)

target_link_libraries(your_target PRIVATE omuraisu)
```

### CMake（システムインストール）

```bash
mkdir build && cd build
cmake ..
cmake --build .
sudo cmake --install .
```

```cmake
find_package(omuraisu REQUIRED)
target_link_libraries(your_target PRIVATE omuraisu::omuraisu)
```

> 個別モジュールのみをリンクすることも可能です（`omuraisu::coordinate`, `omuraisu::omuraisu_pid` など）。

---

## モジュール一覧

### coordinate — 2D 座標演算

**ヘッダ:** `coordinate/coordinate.hpp`
**名前空間:** `coordinate`

2D の直交座標・極座標および速度を扱う構造体と演算を提供します。

| 型 | 説明 |
|---|---|
| `Coordinate` | 直交座標（`x`, `y`, `ang`, `axis_ang`） |
| `CoordinatePolar` | 極座標（`r`, `theta`, `ang`, `axis_ang`） |
| `Velocity` | `Coordinate` のエイリアス |
| `VelocityPolar` | `CoordinatePolar` のエイリアス |

```cpp
using namespace coordinate;

// 直交座標 ↔ 極座標の相互変換
Coordinate cart(3.0f, 4.0f);
CoordinatePolar polar(cart);  // r=5.0, theta=atan2(4,3)

// 算術演算
Coordinate a(1.0f, 2.0f), b(3.0f, 4.0f);
a += b;  // a = {4.0, 6.0, 0.0}
a *= 2.0f;

// 座標系の回転
convert_ang(cart, M_PI / 2);

// 2点間の距離
float d = distance(a, b);
```

### chassis — メカナムホイール制御

**ヘッダ:** `chassis/mecanum.hpp`
**名前空間:** `chassis`

メカナムホイール（4 輪）の逆運動学計算を行い、指定した速度ベクトルから各ホイールの目標速度を算出します。

| クラス | 説明 |
|---|---|
| `Mecanum` | 4 輪メカナムの速度計算 |

```cpp
using namespace chassis;

// 各ホイールの位置を極座標で指定
std::array<CoordinatePolar, 4> wheel_pos = {
  CoordinatePolar(0.2f,     M_PI / 4),
  CoordinatePolar(0.2f, 3 * M_PI / 4),
  CoordinatePolar(0.2f, 5 * M_PI / 4),
  CoordinatePolar(0.2f, 7 * M_PI / 4),
};
Mecanum mecanum(wheel_pos);
// ホイールの中心からの半径でホイール位置を指定(各ホイールが正方形に配置されている場合)
float radius = 300;
Mecanum mecanum(radius);

// 目標速度 → 各ホイール速度
Velocity vel = {1.0f, 0.0f, 0.0f};  // 前進
float result[4];
mecanum.calc(vel, result);
```

### pid — PID 制御

**ヘッダ:** `pid/pid.hpp`
**名前空間:** `pid`

出力リミット付きの PID コントローラを提供します。

$$
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de}{dt}
$$

| 型 | 説明 |
|---|---|
| `PidGain` | ゲイン（`kp`, `ki`, `kd`） |
| `PidParameter` | ゲイン + 出力上下限（`gain`, `min`, `max`） |
| `Pid` | PID 制御クラス |

```cpp
pid::PidParameter param = {
  .gain = {.kp = 1.0f, .ki = 0.1f, .kd = 0.01f},
  .min = -100.0f,
  .max = 100.0f,
};
pid::Pid pid(param);

float output = pid.calc(/*goal=*/100.0f, /*actual=*/0.0f, /*dt=*/0.01f);

pid.reset();  // 積分値・前回偏差をクリア
```

### dji — DJI ロボマスモーター制御

**ヘッダ:** `dji/robomas.hpp`, `dji/robomas_core.hpp`
**名前空間:** `dji`

DJI M3508 / C620 モーターの CAN 通信プロトコルを実装します。最大 8 軸を同時制御できます。

| クラス | 説明 |
|---|---|
| `RobomasData` | モーターフィードバック（角度・回転数・電流・温度） |
| `RobomasCore` | プラットフォーム非依存のコアロジック |
| `Robomas` | `RobomasCore` + `ICanBus` を組み合わせた高レベル API |

```cpp
// mbed 環境の例
can::MbedCanBus can_bus(PA_11, PA_12);
dji::Robomas robomas(can_bus);

// モーター出力を設定（ID: 1〜8）
robomas.set_output(5000, 1);

// パーセント指定も可能（-1.0 〜 1.0）
robomas.set_output_percent(0.5f, 2);

// CAN 送信
robomas.write();

// フィードバック受信
int motor_idx = robomas.read_data();
if (motor_idx >= 0) {
  uint16_t angle = robomas.get_angle(motor_idx + 1);
  int16_t rpm    = robomas.get_rpm(motor_idx + 1);
}
```

### can — CAN バス抽象化

**ヘッダ:** `can/can_interface.hpp`, `can/can_mbed.hpp`
**名前空間:** `can`

プラットフォーム非依存の CAN バスインターフェースを定義します。

| 型 | 説明 |
|---|---|
| `CanMessage` | CAN メッセージ（ID, データ, 長さ） |
| `ICanBus` | 抽象 CAN バスインターフェース（`write` / `read`） |
| `MbedCanBus` | mbed 環境向け実装 |

```cpp
// mbed の例
can::MbedCanBus bus(PA_11, PA_12, 1000000);

can::CanMessage msg;
msg.id = 0x200;
msg.data[0] = 0xFF;
msg.len = 1;
bus.write(msg);
```

### controller — コントローラ入力

**ヘッダ:** `controller/controller_core.hpp`, `controller/controller_can.hpp`, `controller/controller_serial.hpp`
**名前空間:** `controller`

PS5 コントローラからの入力を受信・パースします。CAN 通信・シリアル（COBS）通信・ROS2 Joy メッセージの 3 つの入力方式に対応しています。

> **依存:** シリアル通信を使用する場合 [cobs-serial-manager](https://github.com/NekoChan9382/cobs-serial-manager) が必要です。

| 型 | 説明 |
|---|---|
| `ControllerData` | コントローラの状態（スティック・トリガー・ボタン） |
| `Button` / `DPad` | ボタン / 十字キーの列挙型 |
| `CanParser` | CAN フレームからのパーサ |
| `RosJoyParser` | ROS2 `sensor_msgs/msg/Joy` からのパーサ |
| `CanController` | CAN 経由でのコントローラ受信クラス |
| `SerialController` | シリアル（COBS）経由でのコントローラ受信クラス |

```cpp
// CAN 経由
can::MbedCanBus can_bus(PA_11, PA_12);
controller::CanController ctrl(can_bus);

if (ctrl.read()) {
  const auto& data = ctrl.data();
  float lx = data.left_x;   // -1.0 〜 1.0
  bool  a  = data.circle();  // ○ボタン
  bool  up = data.is_pressed(controller::DPad::Up);
}

// ROS2 経由（テンプレート対応）
controller::RosJoyParser parser;
parser.parse(joy_msg);  // sensor_msgs::msg::Joy
```

### servo — サーボ制御

**ヘッダ:** `servo/servo_core.hpp`
**名前空間:** `servo`

CAN 経由でサーボモーターを制御するための機能を提供します。角度（0〜180°）を内部で 0〜255 の値に変換し、CAN メッセージとして送信できます。

| クラス | 説明 |
|---|---|
| `ServoCore` | サーボ制御のコアロジック（プラットフォーム非依存） |

**主なメソッド:**

| メソッド | 説明 |
|---|---|
| `ServoCore(uint8_t id)` | CAN ID を指定して初期化 |
| `set_degree(float degree)` | 角度（0〜180°）を設定 |
| `set_degree(float degrees[8])` | 8 チャンネル分の角度を一括設定 |
| `to_can_message()` | 現在の状態を `CANMessage` に変換 |

```cpp
servo::ServoCore servo(0x10);  // CAN ID を指定

// 単一角度を設定
servo.set_degree(90.0f);

// CAN メッセージとして取得・送信
auto msg = servo.to_can_message();
```

---

## サンプルコード

`examples/` ディレクトリにサンプルプログラムがあります。

| ファイル | 内容 |
|---|---|
| `coordinate_example.cpp` | 座標の生成・変換・演算 |
| `mecanum_example.cpp` | メカナムホイールの速度計算 |
| `pid_example.cpp` | PID 制御シミュレーション（100 ステップ） |

サンプルをビルドするには：

```bash
cmake -DBUILD_EXAMPLES=ON ..
cmake --build .
```

---

## ビルド方法

### 要件

- CMake 3.14 以上
- C++17 対応コンパイラ

### ビルド手順

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### ビルドオプション

| オプション | 説明 | デフォルト |
|---|---|---|
| `BUILD_EXAMPLES` | サンプルプログラムをビルド | `OFF` |
| `BUILD_TESTS` | テストをビルド | `OFF` |

---

## テスト

Google Test を使用しています。

```bash
mkdir build && cd build
cmake -DBUILD_TESTS=ON ..
cmake --build .
ctest --output-on-failure
```

| テストファイル | 内容 |
|---|---|
| `tests/coordinate_test.cpp` | 座標の構築・演算・変換 |
| `tests/pid_test.cpp` | 比例制御・出力制限・リセット |

---

## ライセンス

[MIT License](LICENSE)

## 作者
[bit](https://github.com/NekoChan9382)