#include "st_point.h"

namespace planning {
STPoint::STPoint(const float s, const float t) : Vec2d(t, s) 
{

}

STPoint::STPoint(const Vec2d& vec2d_point) : Vec2d(vec2d_point)
{

}

float STPoint::s() const
{
    return y_;
}

float STPoint::t() const
{
    return x_;
}

void STPoint::set_s(const float s)
{
    return set_y(s);
}

void STPoint::set_t(const float t)
{
    return set_x(t);
}

} // namespace planning