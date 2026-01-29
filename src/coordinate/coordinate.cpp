#include "coordinate.hpp"

namespace bit {

float Coordinate::get_axis_ang() const noexcept {
    return axis_ang;
}

Coordinate& Coordinate::operator+=(const Coordinate& other) noexcept
{
    this->x += other.x;
    this->y += other.y;
    this->ang += other.ang;
    return *this;
}
Coordinate& Coordinate::operator-=(const Coordinate& other) noexcept
{
    this->x -= other.x;
    this->y -= other.y;
    this->ang -= other.ang;
    return *this;
}
Coordinate& Coordinate::operator*=(const float& other) noexcept
{
    this->x *= other;
    this->y *= other;
    this->ang *= other;
    return *this;
}
Coordinate& Coordinate::operator/=(const float& other) noexcept
{
    this->x /= other;
    this->y /= other;
    this->ang /= other;
    return *this;
}

Coordinate& Coordinate::operator=(const Coordinate& other) noexcept
{
    this->x = other.x;
    this->y = other.y;
    this->ang = other.ang;
    this->axis_ang = other.axis_ang;
    return *this;
}


void Coordinate::set_axis_ang(const float& offset) noexcept
{
    axis_ang = offset;
}

Coordinate::Coordinate() noexcept
    : x(0), y(0), ang(0), axis_ang(0) {}

Coordinate::Coordinate(const float x, const float y, const float ang, const float axis_ang) noexcept
    : x(x), y(y), ang(ang), axis_ang(axis_ang) {}

Coordinate::Coordinate(const Coordinate& other) noexcept
    : x(other.x), y(other.y), ang(other.ang), axis_ang(other.axis_ang) {}

Coordinate::Coordinate(const CoordinatePolar& other) noexcept
{
    x = other.r * cos(other.theta);
    y = other.r * sin(other.theta);
    ang = other.ang;
    axis_ang = other.axis_ang;
}



float CoordinatePolar::get_axis_ang() const noexcept {
    return axis_ang;
}

CoordinatePolar& CoordinatePolar::operator=(const CoordinatePolar& other) noexcept
{
    this->r = other.r;
    this->theta = other.theta;
    this->ang = other.ang;
    this->axis_ang = other.axis_ang;
    return *this;
}


void CoordinatePolar::set_axis_ang(const float& offset) noexcept
{
    axis_ang = offset;
}

CoordinatePolar::CoordinatePolar() noexcept
    : r(0), theta(0), ang(0), axis_ang(0) {}

CoordinatePolar::CoordinatePolar(float r, float theta, float ang, float axis_ang) noexcept
    : r(r), theta(theta), ang(ang), axis_ang(axis_ang) {}

CoordinatePolar::CoordinatePolar(const Coordinate& other) noexcept
{
    r = hypot(other.x, other.y);
    theta = atan2(other.y, other.x);
    ang = other.ang;
    axis_ang = other.axis_ang;
}


// using Velocity = Coordinate;
// using VelocityPolar = CoordinatePolar;

void convert_ang(CoordinatePolar& p, const float& ang) noexcept
{
    const float diff = ang - p.axis_ang;
    p.theta -= diff;
    p.ang -= diff;
    p.axis_ang = ang;
}

void convert_ang(Coordinate& p, const float& ang) noexcept
{
    CoordinatePolar tmp = static_cast<CoordinatePolar>(p);
    convert_ang(tmp, ang);
    p = static_cast<Coordinate>(tmp);
}

float distance(const Coordinate& p, const Coordinate& q) noexcept
{
    if (p.axis_ang == q.axis_ang)
    {
        return std::hypot(p.x - q.x, p.y - q.y);
    }
    else
    {
        Coordinate tmp = p;
        convert_ang(tmp, q.axis_ang);
        return std::hypot(tmp.x - q.x, tmp.y - q.y);
    }
}

} //namespace bit