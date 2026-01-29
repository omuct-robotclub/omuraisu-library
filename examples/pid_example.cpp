#include "pid/pid.hpp"
#include <iostream>

int main()
{
    // PIDパラメータの設定
    PidParameter param = {
        .gain = {.kp = 2.0f, .ki = 0.5f, .kd = 0.1f},
        .min = -100.0f,
        .max = 100.0f
    };

    Pid pid(param);

    // シミュレーション
    float goal = 100.0f;
    float actual = 0.0f;
    float dt = 0.01f;  // 10ms

    std::cout << "PID Control Simulation" << std::endl;
    std::cout << "Goal: " << goal << std::endl;
    std::cout << "---" << std::endl;

    for (int i = 0; i < 100; ++i)
    {
        float output = pid.calc(goal, actual, dt);
        
        // 簡易的なプラントモデル（1次遅れ系）
        actual += output * 0.01f;

        if (i % 10 == 0)
        {
            std::cout << "Step " << i << ": actual=" << actual 
                      << ", output=" << output << std::endl;
        }
    }

    std::cout << "---" << std::endl;
    std::cout << "Final value: " << actual << std::endl;

    return 0;
}
