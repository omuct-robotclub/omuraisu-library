#include "chassis/mecanum.hpp"
#include "coordinate/coordinate.hpp"
#include <iostream>
#include <cmath>

int main()
{
    using namespace bit;

    // 正方形配置のメカナムホイール（中心から0.2mの位置）
    float radius = 0.2f;
    std::array<CoordinatePolar, 4> wheel_pos = {
        CoordinatePolar(radius, M_PI / 4, 0, 0),       // 前右
        CoordinatePolar(radius, 3 * M_PI / 4, 0, 0),   // 前左
        CoordinatePolar(radius, 5 * M_PI / 4, 0, 0),   // 後左
        CoordinatePolar(radius, 7 * M_PI / 4, 0, 0)    // 後右
    };

    Mecanum mecanum(wheel_pos);

    // 前進
    std::cout << "Forward motion (vx=0, vy=1, ang=0):" << std::endl;
    {
        Velocity vel = {0.0f, 1.0f, 0.0f};
        float result[4];
        mecanum.calc(vel, result);
        std::cout << "  Wheel speeds: [" 
                  << result[0] << ", " << result[1] << ", " 
                  << result[2] << ", " << result[3] << "]" << std::endl;
    }

    // 横移動
    std::cout << "Strafe motion (vx=1, vy=0, ang=0):" << std::endl;
    {
        Velocity vel = {1.0f, 0.0f, 0.0f};
        float result[4];
        mecanum.calc(vel, result);
        std::cout << "  Wheel speeds: [" 
                  << result[0] << ", " << result[1] << ", " 
                  << result[2] << ", " << result[3] << "]" << std::endl;
    }

    // 回転
    std::cout << "Rotation (vx=0, vy=0, ang=1):" << std::endl;
    {
        Velocity vel = {0.0f, 0.0f, 1.0f};
        float result[4];
        mecanum.calc(vel, result);
        std::cout << "  Wheel speeds: [" 
                  << result[0] << ", " << result[1] << ", " 
                  << result[2] << ", " << result[3] << "]" << std::endl;
    }

    return 0;
}
