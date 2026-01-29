#include "coordinate/coordinate.hpp"
#include <iostream>
#include <cmath>

int main()
{
    using namespace bit;

    // 直交座標の作成
    Coordinate coord1(3.0f, 4.0f, 0.0f);
    std::cout << "Coordinate 1: (" << coord1.x << ", " << coord1.y << ")" << std::endl;

    // 極座標の作成
    CoordinatePolar polar1(5.0f, M_PI / 4, 0.0f);
    std::cout << "Polar 1: (r=" << polar1.r << ", theta=" << polar1.theta << " rad)" << std::endl;

    // 極座標から直交座標への変換
    Coordinate coord2(polar1);
    std::cout << "Polar to Cartesian: (" << coord2.x << ", " << coord2.y << ")" << std::endl;

    // 直交座標から極座標への変換
    CoordinatePolar polar2(coord1);
    std::cout << "Cartesian to Polar: (r=" << polar2.r << ", theta=" << polar2.theta << " rad)" << std::endl;

    // 座標の演算
    Coordinate coord3(1.0f, 2.0f, 0.0f);
    coord1 += coord3;
    std::cout << "After addition: (" << coord1.x << ", " << coord1.y << ")" << std::endl;

    coord1 *= 2.0f;
    std::cout << "After multiplication: (" << coord1.x << ", " << coord1.y << ")" << std::endl;

    // Velocityの使用
    Velocity vel = {1.0f, 2.0f, 0.5f};
    std::cout << "Velocity: (vx=" << vel.x << ", vy=" << vel.y << ", ang=" << vel.ang << ")" << std::endl;

    return 0;
}
