/**
 * @file
 * @brief Defines the Vec2d class.
 */
#pragma once

#include <cmath>
#include <string>


namespace planning {

constexpr float kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec2d {
public:
    //! Constructor which takes x- and y-coordinates.
    constexpr Vec2d(const float x, const float y) noexcept : x_(x), y_(y) {}

    //! Constructor returning the zero vector.
    constexpr Vec2d() noexcept : Vec2d(0, 0) {}

    //! Creates a unit-vector with a given angle to the positive x semi-axis
    static Vec2d CreateUnitVec2d(const float angle);

    //! Getter for x component
    float x() const { return x_; }

    //! Getter for y component
    float y() const { return y_; }

    //! Setter for x component
    void set_x(const float x) { x_ = x; }

    //! Setter for y component
    void set_y(const float y) { y_ = y; }

    //! Gets the length of the vector
    float Length() const;

    //! Gets the squared length of the vector
    float LengthSquare() const;

    //! Gets the angle between the vector and the positive x semi-axis
    float Angle() const;

    //! Returns the unit vector that is co-linear with this vector
    void Normalize();

    //! Returns the distance to the given vector
    float DistanceTo(const Vec2d &other) const;

    //! Returns the squared distance to the given vector
    float DistanceSquareTo(const Vec2d &other) const;

    //! Returns the "cross" product between these two Vec2d (non-standard).
    float CrossProd(const Vec2d &other) const;

    //! Returns the inner product between these two Vec2d.
    float InnerProd(const Vec2d &other) const;

    //! rotate the vector by angle.
    Vec2d rotate(const float angle) const;

    //! Sums two Vec2d
    Vec2d operator+(const Vec2d &other) const;

    //! Subtracts two Vec2d
    Vec2d operator-(const Vec2d &other) const;

    //! Multiplies Vec2d by a scalar
    Vec2d operator*(const float ratio) const;

    //! Divides Vec2d by a scalar
    Vec2d operator/(const float ratio) const;

    //! Sums another Vec2d to the current one
    Vec2d &operator+=(const Vec2d &other);

    //! Subtracts another Vec2d to the current one
    Vec2d &operator-=(const Vec2d &other);

    //! Multiplies this Vec2d by a scalar
    Vec2d &operator*=(const float ratio);

    //! Divides this Vec2d by a scalar
    Vec2d &operator/=(const float ratio);

    //! Compares two Vec2d
    bool operator==(const Vec2d &other) const;

protected:
    float x_ = 0.0;
    float y_ = 0.0;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const float ratio, const Vec2d &vec);

}  // namespace planning
