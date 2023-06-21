/**
 * @file
 * @brief Defines the AABox2d class.
 */

#pragma once

#include <string>
#include <vector>

#include "vec2d.h"

namespace planning {

/**
 * @class AABox2d
 * @brief Implements a class of (undirected) axes-aligned bounding boxes in 2-D.
 * This class is referential-agnostic.
 */
class AABox2d {
 public:
  /**
   * @brief Default constructor.
   * Creates an axes-aligned box with zero length and width at the origin.
   */
  AABox2d() = default;
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box with given center, length, and width.
   * @param center The center of the box
   * @param length The size of the box along the x-axis
   * @param width The size of the box along the y-axis
   */
  AABox2d(const Vec2d &center, const float length, const float width);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box from two opposite corners.
   * @param one_corner One corner of the box
   * @param opposite_corner The opposite corner to the first one
   */
  AABox2d(const Vec2d &one_corner, const Vec2d &opposite_corner);
  /**
   * @brief Parameterized constructor.
   * Creates an axes-aligned box containing all points in a given vector.
   * @param points Vector of points to be included inside the box.
   */
  explicit AABox2d(const std::vector<Vec2d> &points);

  /**
   * @brief Getter of center_
   * @return Center of the box
   */
  const Vec2d &center() const { return center_; }

  /**
  * @brief Getter of x-component of center_
  * @return x-component of the center of the box
  */
  float center_x() const { return center_.x(); }

  /**
  * @brief Getter of y-component of center_
  * @return y-component of the center of the box
  */
  float center_y() const { return center_.y(); }

  /**
  * @brief Getter of length_
  * @return The length of the box
  */
  float length() const { return length_; }

  /**
  * @brief Getter of width_
  * @return The width of the box
  */
  float width() const { return width_; }

  /**
  * @brief Getter of half_length_
  * @return Half of the length of the box
  */
  float half_length() const { return half_length_; }

  /**
  * @brief Getter of half_width_
  * @return Half of the width of the box
  */
  float half_width() const { return half_width_; }

  /**
  * @brief Getter of length_*width_
  * @return The area of the box
  */
  float area() const { return length_ * width_; }

  /**
  * @brief Returns the minimum x-coordinate of the box
  *
  * @return x-coordinate
  */
  float min_x() const { return center_.x() - half_length_; }

  /**
  * @brief Returns the maximum x-coordinate of the box
  *
  * @return x-coordinate
  */
  float max_x() const { return center_.x() + half_length_; }

  /**
  * @brief Returns the minimum y-coordinate of the box
  *
  * @return y-coordinate
  */
  float min_y() const { return center_.y() - half_width_; }

  /**
  * @brief Returns the maximum y-coordinate of the box
  *
  * @return y-coordinate
  */
  float max_y() const { return center_.y() + half_width_; }

  /**
  * @brief Gets all corners in counter clockwise order.
  *
  * @param corners Output where the corners are written
  */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  /**
  * @brief Determines whether a given point is in the box.
  *
  * @param point The point we wish to test for containment in the box
  */
  bool IsPointIn(const Vec2d &point) const;

  /**
  * @brief Determines whether a given point is on the boundary of the box.
  *
  * @param point The point we wish to test for boundary membership
  */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
  * @brief Determines the distance between a point and the box.
  *
  * @param point The point whose distance to the box we wish to determine.
  */
  float DistanceTo(const Vec2d &point) const;

  /**
  * @brief Determines the distance between two boxes.
  *
  * @param box Another box.
  */
  float DistanceTo(const AABox2d &box) const;

  /**
  * @brief Determines whether two boxes overlap.
  *
  * @param box Another box
  */
  bool HasOverlap(const AABox2d &box) const;

  /**
  * @brief Shift the center of AABox by the input vector.
  *
  * @param shift_vec The vector by which we wish to shift the box
  */
  void Shift(const Vec2d &shift_vec);

  /**
  * @brief Changes box to include another given box, as well as the current one.
  *
  * @param other_box Another box
  */
  void MergeFrom(const AABox2d &other_box);

  /**
  * @brief Changes box to include a given point, as well as the current box.
  *
  * @param other_point Another point
  */
  void MergeFrom(const Vec2d &other_point);


 private:
  Vec2d center_;
  float length_ = 0.0;
  float width_ = 0.0;
  float half_length_ = 0.0;
  float half_width_ = 0.0;
};

}  // namespace planning
