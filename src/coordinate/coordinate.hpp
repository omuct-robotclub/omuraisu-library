#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <cmath>

namespace bit {

struct Coordinate;
struct CoordinatePolar;

struct Coordinate
{
    float x;
    float y;
    float ang;
    float axis_ang;

    float get_axis_ang() const noexcept;

    Coordinate& operator+=(const Coordinate& other) noexcept;

    Coordinate& operator-=(const Coordinate& other) noexcept;

    Coordinate& operator*=(const float& other) noexcept;

    Coordinate& operator/=(const float& other) noexcept;

    Coordinate& operator=(const Coordinate& other) noexcept;

    void set_axis_ang(const float& offset) noexcept;

    Coordinate() noexcept;

    Coordinate(const float x, const float y, const float ang = 0.0f, const float axis_ang = 0.0f) noexcept;

    Coordinate(const Coordinate& other) noexcept;

    explicit Coordinate(const CoordinatePolar& other) noexcept;
};

struct CoordinatePolar
{
    float r;
    float theta;
    float ang;
    float axis_ang;

    float get_axis_ang() const noexcept;

    CoordinatePolar& operator=(const CoordinatePolar& other) noexcept;
    
    void set_axis_ang(const float& offset) noexcept;

    CoordinatePolar() noexcept;

    CoordinatePolar(float r, float theta, float ang = 0.0f, float axis_ang = 0.0f) noexcept;

    explicit CoordinatePolar(const Coordinate& other) noexcept;
};

using Velocity = Coordinate;
using VelocityPolar = CoordinatePolar;

void convert_ang(CoordinatePolar& p, const float& ang) noexcept;

void convert_ang(Coordinate& p, const float& ang) noexcept;

float distance(const Coordinate& p, const Coordinate& q) noexcept;

} // namespace bit
#endif // COORDINATE_HPP
