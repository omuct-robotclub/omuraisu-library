# omuraisu-library

ロボット開発において頻繁に使用される基本的な機能を提供するC++ライブラリ群

PlatformIOでの使用をメインの用途として想定しています。

## 機能

### chassis/mecanum.hpp

メカナムホイールの足回りのホイール速度の算出を提供

### coordinate/coordinate.hpp

2D座標の基本的な演算を提供

- `Coordinate`: 直交座標系
- `CoordinatePolar`: 極座標系
- `Velocity`: 速度構造体

### pid/pid.hpp

基本的なPID制御を提供

- `PidGain`: PIDゲイン構造体
- `PidParameter`: PIDパラメータ構造体
- `Pid`: PID制御クラス

### dji/robomas.hpp

ロボマスモーター（M3508/C620）の制御を提供

- **注意**: このモジュールはmbed環境が必要です

### controller/controller.hpp

- **注意**: https://github.com/NekoChan9382/cobs-serial-manager に依存しています

## ビルド方法

### 要件

- CMake 3.14以上
- C++17対応コンパイラ

### ビルド手順

```bash
# ビルドディレクトリを作成
mkdir build && cd build

# CMakeを実行
cmake ..

# ビルド
cmake --build .
```

### ビルドオプション

| オプション       | 説明                       | デフォルト |
| ---------------- | -------------------------- | ---------- |
| `BUILD_EXAMPLES` | サンプルプログラムをビルド | OFF        |
| `BUILD_TESTS`    | テストをビルド             | OFF        |

```bash
# サンプルを含めてビルド
cmake -DBUILD_EXAMPLES=ON ..
cmake --build .
```

## 使用方法

### CMakeプロジェクトでの使用

```cmake
find_package(omuraisu REQUIRED)
target_link_libraries(your_target omuraisu::omuraisu)
```

### ヘッダファイルの直接インクルード

```cpp
#include "coordinate/coordinate.hpp"
#include "chassis/mecanum.hpp"
#include "pid/pid.hpp"
```

## サンプルコード

### PID制御

```cpp
#include "pid/pid.hpp"

int main() {
    PidParameter param = {
        .gain = {.kp = 1.0f, .ki = 0.1f, .kd = 0.01f},
        .min = -100.0f,
        .max = 100.0f
    };

    Pid pid(param);

    float goal = 100.0f;
    float actual = 0.0f;
    float dt = 0.01f;

    float output = pid.calc(goal, actual, dt);
    return 0;
}
```

### メカナムホイール

```cpp
#include "chassis/mecanum.hpp"
#include "coordinate/coordinate.hpp"

int main() {
    using namespace bit;

    // ホイール位置を設定
    std::array<CoordinatePolar, 4> wheel_pos = {
        CoordinatePolar(0.2f, M_PI/4, 0, 0),
        CoordinatePolar(0.2f, 3*M_PI/4, 0, 0),
        CoordinatePolar(0.2f, 5*M_PI/4, 0, 0),
        CoordinatePolar(0.2f, 7*M_PI/4, 0, 0)
    };

    Mecanum mecanum(wheel_pos);

    // 速度を設定して各ホイール速度を計算
    Velocity vel = {1.0f, 0.0f, 0.0f};
    float result[4];
    mecanum.calc(vel, result);

    return 0;
}
```

## ライセンス

MIT License
