# omuraisu-library

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C11](https://img.shields.io/badge/C-11-blue.svg)](https://en.cppreference.com/w/c/language)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

ロボット開発において頻繁に使用される基本的な機能を提供する C / C++ ライブラリ群です。

PlatformIO / CMake の両方に対応しており、組み込み（CubeMX, Mbedなど）からデスクトップまで幅広い環境で利用できます。

## 目次

- [インストール](#インストール)
- [C API / CPP API の使い分け](#c-api--cpp-api-の使い分け)
- [モジュール一覧](#モジュール一覧)
  - [coordinate — 2D 座標演算](#coordinate--2d-座標演算)
  - [chassis — メカナムホイール制御](#chassis--メカナムホイール制御)
  - [cobs — COBS エンコード/デコード](#cobs--cobs-エンコードデコード)
  - [pid — PID 制御](#pid--pid-制御)
  - [dji — DJI ロボマスモーター制御](#dji--dji-ロボマスモーター制御)
  - [vesc — VESC モーター制御](#vesc--vesc-モーター制御)
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

### CubeMX

**Project Manager** で **Toolchain / IDE** を **CMake** に設定し、CMake のインストール方法に従ってください。C++ ラッパを使う場合は C++17 を有効化してください。

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

STM32 Cube HAL 向け CAN アダプタ（`can_stm32`）を使う場合は、追加で以下の設定が必要です。

```cmake
target_compile_definitions(omuraisu_can PRIVATE
    USE_HAL_DRIVER
    <使用MCU名>
)
target_include_directories(omuraisu_can PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/<使用MCU名>_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/<使用MCU名>_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/<使用MCU名>/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)
```

`<使用MCU名>` には `STM32F446xx` などが入ります。環境に合うように変更してください。

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

```cmake
# C API を使う場合
target_link_libraries(your_c_target PRIVATE omuraisu::omuraisu_c)

# C++ ラッパ API を使う場合
target_link_libraries(your_cpp_target PRIVATE omuraisu::omuraisu_cpp)
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

> 個別モジュールのみをリンクすることも可能です（例: `omuraisu::omuraisu_pid`, `omuraisu::omuraisu_cpp_pid`）。

---

## C API / CPP API の使い分け

本ライブラリは C 実装を基盤に、同等機能を C++ から扱いやすくするラッパを提供します。

| 種別    | インクルード例           | CMake ターゲット例                                      |
| ------- | ------------------------ | ------------------------------------------------------- |
| C API   | `#include "pid/pid.h"`   | `omuraisu::omuraisu_c` / `omuraisu::omuraisu_pid`       |
| C++ API | `#include "pid/pid.hpp"` | `omuraisu::omuraisu_cpp` / `omuraisu::omuraisu_cpp_pid` |

```cpp
#include "chassis/mecanum.hpp"
#include "coordinate/coordinate.hpp"

omuraisu::coordinate::Velocity vel(1.0f, 0.0f, 0.0f, 0.0f);
omuraisu::chassis::Mecanum mecanum(0.2f);

float wheel_speed[4] = {0};
mecanum.calc(vel, wheel_speed);
```

---

## モジュール一覧

### coordinate — 2D 座標演算

**ヘッダ:** `coordinate/coordinate.h`

2D の直交座標・極座標および速度を扱う C 構造体と演算関数を提供します。

| 型                | 説明                                      |
| ----------------- | ----------------------------------------- |
| `Coordinate`      | 直交座標（`x`, `y`, `ang`, `axis_ang`）   |
| `CoordinatePolar` | 極座標（`r`, `theta`, `ang`, `axis_ang`） |
| `Velocity`        | `Coordinate` のエイリアス                 |
| `VelocityPolar`   | `CoordinatePolar` のエイリアス            |

```c
#include "coordinate/coordinate.h"

Coordinate a = om_coordinate_init_value(1.0f, 2.0f, 0.0f, 0.0f);
Coordinate b = om_coordinate_init_value(3.0f, 4.0f, 0.0f, 0.0f);
om_coordinate_add(&a, &b);

CoordinatePolar p = om_coordinate_polar_from_rectangular(&a);
float d = om_coordinate_distance(&a, &b);
```

### chassis — メカナムホイール制御

**ヘッダ:** `c/chassis/mecanum.h`, `cpp/chassis/mecanum.hpp`

メカナムホイール（4 輪）の逆運動学計算を C API で提供します。指定した速度ベクトルから各ホイールの目標速度を算出できます。

| 型        | 説明                     |
| --------- | ------------------------ |
| `Mecanum` | 4 輪メカナムの設定と状態 |

```c
#include "chassis/mecanum.h"

Mecanum mecanum = om_mecanum_init_radius(0.2f);
Velocity vel = {.x = 1.0f, .y = 0.0f, .ang = 0.0f, .axis_ang = 0.0f};
float wheel_speed[4] = {0};

om_mecanum_calc(&mecanum, &vel, wheel_speed);
```

### cobs — COBS エンコード/デコード

**ヘッダ:** `c/cobs/cobs.h`, `cpp/cobs/cobs.hpp`

シリアル通信で使いやすい COBS（Consistent Overhead Byte Stuffing）形式のエンコード/デコードを提供します。

| 型 / 関数群      | 説明                         |
| ---------------- | ---------------------------- |
| `om_cobs_encode` | 生バイト列を COBS 形式へ変換 |
| `om_cobs_decode` | COBS 形式のデータを復元      |

```c
#include "cobs/cobs.h"

const uint8_t raw[] = {0x11, 0x00, 0x22};
uint8_t encoded[16] = {0};
uint8_t decoded[16] = {0};

size_t enc_len = sizeof(encoded);
size_t dec_len = sizeof(decoded);
bool enc_ok = om_cobs_encode(raw, sizeof(raw), encoded, &enc_len);
bool dec_ok = om_cobs_decode(encoded, enc_len, decoded, &dec_len);
```

### pid — PID 制御

**ヘッダ:** `c/pid/pid.h`, `cpp/pid/pid.hpp`

出力リミット付きの PID コントローラを C API で提供します。

$$
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de}{dt}
$$

| 型              | 説明                                             |
| --------------- | ------------------------------------------------ |
| `PidGain`       | ゲイン（`kp`, `ki`, `kd`）                       |
| `PidParameter`  | ゲイン + 出力上下限（`gain`, `min`, `max`）      |
| `PidController` | PID 制御の内部状態（前回偏差・積分値などを保持） |

```c
#include "pid/pid.h"

PidParameter param = {
  .gain = {.kp = 1.0f, .ki = 0.1f, .kd = 0.01f},
  .min = -100.0f,
  .max = 100.0f,
};
PidController pid = om_pid_init(param);

float output = om_pid_calc(&pid, 100.0f, 0.0f, 0.01f);
om_pid_reset(&pid);
```

### dji — DJI ロボマスモーター制御

**ヘッダ:** `c/dji/robomas.h`, `c/dji/robomas_core.h`, `cpp/dji/robomas.hpp`, `cpp/dji/robomas_core.hpp`

DJI M3508 / C620 モーター向けの CAN 制御ロジックを C API で提供します。最大 8 軸を同時制御できます。

| 型            | 説明                                               |
| ------------- | -------------------------------------------------- |
| `RobomasData` | モーターフィードバック（角度・回転数・電流・温度） |
| `RobomasCore` | プラットフォーム非依存のコアロジック               |
| `Robomas`     | `CanBus` と `RobomasCore` をまとめた制御ハンドラ   |

```c
#include "dji/robomas.h"

Robomas rm = om_rm_init(&bus);
om_rm_set_output(&rm, 5000, 1);
om_rm_set_output_percent(&rm, 0.5f, 2);
om_rm_write(&rm);

int motor_idx = om_rm_read(&rm);
if (motor_idx >= 0) {
  uint16_t angle = om_rm_get_angle(&rm, motor_idx + 1);
  int16_t rpm = om_rm_get_rpm(&rm, motor_idx + 1);
}
```

### vesc — VESC モーター制御

**ヘッダ:** `c/vesc/vesc_core.h`

VESC の CAN 拡張 ID を使った送信フレーム生成と、ステータスフレームのパースを行う C API を提供します。最大 8 軸の VESC ID（1〜8）を扱えます。

| 型            | 説明                                               |
| ------------- | -------------------------------------------------- |
| `VescData`    | VESC フィードバック（回転数・電流・Duty）          |
| `VescTxFrame` | 送信用 CAN 拡張 ID、データ、データ長               |
| `VescCore`    | 送信フレーム生成と受信データを保持するコアロジック |

```c
#include "vesc/vesc_core.h"

VescCore vesc = om_vesc_core_init();
om_vesc_core_set_max_current(&vesc, 30.0f);
om_vesc_core_set_current(&vesc, 5.0f, 1);

uint32_t tx_id = 0;
uint8_t tx_data[8] = {0};
uint8_t tx_len = 0;
om_vesc_core_get_output(&vesc, &tx_id, tx_data, &tx_len);

// CanMessage msg = {.id = tx_id, .data = {0}, .len = tx_len};
// memcpy(msg.data, tx_data, tx_len);
// can_bus_write(&bus, &msg);

const uint8_t status[8] = {0};
if (om_vesc_core_parse(&vesc, (VESC_CAN_PACKET_STATUS << 8) | 1, status) >= 0) {
  int32_t rpm = om_vesc_core_get_rpm(&vesc, 1);
  int16_t current_x10 = om_vesc_core_get_current_x10(&vesc, 1);
}
```

### can — CAN バス抽象化

プラットフォーム非依存の CAN バスインターフェースを C API で定義します。

#### C 側実装

**ヘッダ:** `c/can/can_interface.h`, `c/can/can_cube.h`, `c/can/can_stm32.h`

| 型                | 説明                                       |
| ----------------- | ------------------------------------------ |
| `CanMessage`      | CAN メッセージ（ID, データ, 長さ）         |
| `CanBus`          | 抽象 CAN バスインターフェース              |
| `CanCube`         | Cube HAL 向けの受信キュー付き CAN ブリッジ |
| `CanStm32Context` | STM32 HAL 用の登録コンテキスト             |

```c
#include "can/can_interface.h"

CanMessage msg = {0};
msg.id = 0x200;
msg.data[0] = 0xFF;
msg.len = 1;

can_bus_write(&bus, &msg);
```

Cube HAL で受信コールバックと組み合わせる場合は、`can_cube` に `can_stm32` の ops を渡して使います。

#### C++ 側実装

**ヘッダ:** `cpp/can/can_interface.hpp`, `cpp/can/can_mbed.hpp` （mbed 環境のみ）

- `ICanBus`: C++ 側の抽象インターフェース（仮想関数ベース）
- `CCanBusAdapter`: C 実装（CanBus）を C++ ユーザーに ICanBus として提供
- `MbedCanBus`: mbed ハードウェア用 ICanBus 実装（C++ 専用）

```cpp
#include "can/can_interface.hpp"
#include "can/can_mbed.hpp"

// mbed 環境で CAN バスを初期化
omuraisu::can::MbedCanBus bus(p9, p10);  // RX, TX ピン

omuraisu::can::CanMessage msg{0x123, {0x01, 0x02}, 2};
bus.write(msg);

if (bus.read(msg)) {
  // メッセージ受信
}
```

```c
CanCube cube;
CanStm32Context stm32;
CanCubeOps ops;

can_stm32_context_init(&stm32, &cube, &hcan1, CAN_STM32_KIND_CAN, 0);
can_stm32_make_ops(&ops);
can_cube_init(&cube, &stm32, &ops);
can_stm32_register(&stm32);
can_cube_start_read(&cube);
CanBus* bus = can_cube_bus(&cube);


void Can_Callback(const CanMessage* msg, void* user_arg);
can_cube_set_rx_callback(&cube, Can_Callback, NULL); // CAN受信時のコールバックを登録(任意)

// HAL callback 内
// HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//   can_stm32_dispatch_rx(hcan);
// }
```

### controller — コントローラ入力

**ヘッダ:** `c/controller/controller_core.h`, `c/controller/controller_transport.h`, `cpp/controller/controller_core.hpp`, `cpp/controller/controller_transport.hpp`

PS5 コントローラ入力を扱うデータ型と、CAN/シリアル/ROS Joy からの変換関数を C API で提供します。

| 型 / 関数群                                                        | 説明                           |
| ------------------------------------------------------------------ | ------------------------------ |
| `ControllerData`                                                   | コントローラ状態とボタン定義   |
| `om_ctrl_*`（`controller_core.h`）                                 | ボタン判定・方向判定のヘルパー |
| `SerialPacket` / `om_ctrl_data_from_*`（`controller_transport.h`） | シリアル・CAN・ROS Joy の変換  |

```c
#include "controller/controller_core.h"
#include "controller/controller_transport.h"

ControllerData data = om_ctrl_data_from_can(can_id, can_payload);
if (om_ctrl_cross(&data)) {
  // CROSS ボタンが押されている
}

SerialPacket pkt = om_ctrl_serial_packet_from_data(&data);
bool valid = om_ctrl_serial_packet_verify_checksum(&pkt);
```

### servo — サーボ制御

**ヘッダ:** `c/servo/servo_core.h`, `cpp/servo/servo_core.hpp`

角度（0〜180°）を 0〜255 に変換して CAN メッセージ化するサーボ制御 API を提供します。

| 型          | 説明                               |
| ----------- | ---------------------------------- |
| `ServoData` | 8ch サーボ値と送信先 CAN ID を保持 |

```c
#include "servo/servo_core.h"

ServoData servo = {.data = {0}, .id = 0x10};
om_servo_set_degree(&servo, 90.0f, 0);

CanMessage msg = om_servo_to_can_message(&servo);
can_bus_write(&bus, &msg);
```

---

## サンプルコード

`examples/` ディレクトリにサンプルプログラムがあります。

| ファイル                       | 内容                                     |
| ------------------------------ | ---------------------------------------- |
| `coordinate_generic_example.c` | C API での座標演算の基本例               |
| `coordinate_example.cpp`       | 座標の生成・変換・演算                   |
| `mecanum_example.cpp`          | メカナムホイールの速度計算               |
| `pid_example.cpp`              | PID 制御シミュレーション（100 ステップ） |

サンプルをビルドするには：

```bash
cmake -DBUILD_EXAMPLES=ON ..
cmake --build .
```

---

## ビルド方法

### 要件

- CMake 3.14 以上
- C11 対応コンパイラ
- C++ ラッパを使う場合は C++17 対応コンパイラ

### ビルド手順

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

### ビルドオプション

| オプション                        | 説明                                     | デフォルト |
| --------------------------------- | ---------------------------------------- | ---------- |
| `BUILD_EXAMPLES`                  | サンプルプログラムをビルド               | `OFF`      |
| `BUILD_TESTS`                     | テストをビルド                           | `OFF`      |
| `OMURAISU_CAN_STM32_ENABLE`       | STM32 Cube HAL 向け CAN アダプタを有効化 | `OFF`      |
| `OMURAISU_CAN_STM32_FDCAN_ENABLE` | STM32 アダプタで FDCAN 対応を有効化      | `ON`       |

---

## テスト

Google Test を使用しています。

```bash
mkdir build && cd build
cmake -DBUILD_TESTS=ON ..
cmake --build .
ctest --output-on-failure
```

| テストファイル                  | 内容                                 |
| ------------------------------- | ------------------------------------ |
| `tests/cobs_test.c`             | C API の COBS エンコード/デコード    |
| `tests/cobs_cpp_test.cpp`       | C++ ラッパ COBS の動作確認           |
| `tests/can_cpp_test.cpp`        | C/C++ CAN インターフェースの接続確認 |
| `tests/coordinate_cpp_test.cpp` | C++ 座標ラッパの演算・変換           |
| `tests/pid_cpp_test.cpp`        | C++ PID ラッパの制御計算             |
| `tests/chassis_cpp_test.cpp`    | C++ メカナムラッパの速度計算         |
| `tests/dji_cpp_test.cpp`        | C++ DJI ラッパの基本挙動             |
| `tests/servo_cpp_test.cpp`      | C++ サーボラッパの CAN 変換          |
| `tests/controller_cpp_test.cpp` | C++ コントローラ入力ラッパ           |

---

## ライセンス

[MIT License](LICENSE)

## 作者

[bit](https://github.com/NekoChan9382)
