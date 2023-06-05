/**
 * @file
 * @brief The class of Box2d. Here, the x/y axes are respectively Forward/Left,
 *        as opposed to what happens in euler_angles_zxy.h (Right/Forward).
 */

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "aabox2d.h"
#include "line_segment2d.h"
#include "vec2d.h"


namespace planning {

/**
 * @class Box2d
 * @brief Rectangular (undirected) bounding box in 2-D.
 *
 * This class is referential-agnostic, although our convention on the use of
 * the word "heading" in this project (permanently set to be 0 at East)
 * forces us to assume that the X/Y frame here is East/North.
 * For disambiguation, we call the axis of the rectangle parallel to the
 * heading direction the "heading-axis". The size of the heading-axis is
 * called "length", and the size of the axis perpendicular to it "width".
 */
class Box2d {
 public:
  Box2d() = default;
  /**
   * @brief Constructor which takes the center, heading, length and width.
   * @param center The center of the rectangular bounding box.
   * @param heading The angle between the x-axis and the heading-axis,
   *        measured counter-clockwise.
   * @param length The size of the heading-axis.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
  Box2d(const Vec2d &center, const float heading, const float length,
        const float width);

  /**
   * @brief Constructor which takes the heading-axis and the width of the box
   * @param axis The heading-axis
   * @param width The width of the box, which is taken perpendicularly
   * to the heading direction.
   */
  Box2d(const LineSegment2d &axis, const float width);

  /**
   * @brief Constructor which takes an AABox2d (axes-aligned box).
   * @param aabox The input AABox2d.
   */
  explicit Box2d(const AABox2d &aabox);

  /**
   * @brief Creates an axes-aligned Box2d from two opposite corners
   * @param one_corner One of the corners
   * @param opposite_corner The opposite corner to the first one
   * @return An axes-aligned Box2d
   */
  static Box2d CreateAABox(const Vec2d &one_corner,
                           const Vec2d &opposite_corner);

  /**
   * @brief Getter of the center of the box
   * @return The center of the box
   */
  const Vec2d &center() const { return center_; }

  /**
   * @brief Getter of the x-coordinate of the center of the box
   * @return The x-coordinate of the center of the box
   */
  float center_x() const { return center_.x(); }

  /**
   * @brief Getter of the y-coordinate of the center of the box
   * @return The y-coordinate of the center of the box
   */
  float center_y() const { return center_.y(); }

  /**
   * @brief Getter of the length
   * @return The length of the heading-axis
   */
  float length() const { return length_; }

  /**
   * @brief Getter of the width
   * @return The width of the box taken perpendicularly to the heading
   */
  float width() const { return width_; }

  /**
   * @brief Getter of half the length
   * @return Half the length of the heading-axis
   */
  float half_length() const { return half_length_; }

  /**
   * @brief Getter of half the width
   * @return Half the width of the box taken perpendicularly to the heading
   */
  float half_width() const { return half_width_; }

  /**
   * @brief Getter of the heading
   * @return The counter-clockwise angle between the x-axis and the heading-axis
   */
  float heading() const { return heading_; }

  /**
   * @brief Getter of the cosine of the heading
   * @return The cosine of the heading
   */
  float cos_heading() const { return cos_heading_; }

  /**
   * @brief Getter of the sine of the heading
   * @return The sine of the heading
   */
  float sin_heading() const { return sin_heading_; }

  /**
   * @brief Getter of the area of the box
   * @return The product of its length and width
   */
  float area() const { return length_ * width_; }

  /**
   * @brief Getter of the size of the diagonal of the box
   * @return The diagonal size of the box
   */
  float diagonal() const { return std::hypot(length_, width_); }

  /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  /**
   * @brief Getter of the corners of the box
   * @param corners The vector where the corners are listed
   */
  std::vector<Vec2d> GetAllCorners() const;

  /**
   * @brief Tests points for membership in the box
   * @param point A point that we wish to test for membership in the box
   * @return True iff the point is contained in the box
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Tests points for membership in the boundary of the box
   * @param point A point that we wish to test for membership in the boundary
   * @return True iff the point is a boundary point of the box
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the box and a given point
   * @param point The point whose distance to the box we wish to compute
   * @return A distance
   */
  float DistanceTo(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the box and a given line segment
   * @param line_segment The line segment whose distance to the box we compute
   * @return A distance
   */
  float DistanceTo(const LineSegment2d &line_segment) const;

  /**
   * @brief Determines the distance between two boxes
   * @param box The box whose distance to this box we want to compute
   * @return A distance
   */
  float DistanceTo(const Box2d &box) const;

  /**
   * @brief Determines whether this box overlaps a given line segment
   * @param line_segment The line-segment
   * @return True if they overlap
   */
  bool HasOverlap(const LineSegment2d &line_segment) const;

  /**
   * @brief Determines whether these two boxes overlap
   * @param line_segment The other box
   * @return True if they overlap
   */
  bool HasOverlap(const Box2d &box) const;

  /**
   * @brief Gets the smallest axes-aligned box containing the current one
   * @return An axes-aligned box
   */
  AABox2d GetAABox() const;

  /**
   * @brief Rotate from center.
   * @param rotate_angle Angle to rotate.
   */
  void RotateFromCenter(const float rotate_angle);

  /**
   * @brief Shifts this box by a given vector
   * @param shift_vec The vector determining the shift
   */
  void Shift(const Vec2d &shift_vec);

  /**
   * @brief Extend the box longitudinally
   * @param extension_length the length to extend
   */
  void LongitudinalExtend(const float extension_length);

  void LateralExtend(const float extension_length);

  void InitCorners();

  float max_x() const { return max_x_; }
  float min_x() const { return min_x_; }
  float max_y() const { return max_y_; }
  float min_y() const { return min_y_; }

 private:
  Vec2d center_;
  float length_ = 0.0;
  float width_ = 0.0;
  float half_length_ = 0.0;
  float half_width_ = 0.0;
  float heading_ = 0.0;
  float cos_heading_ = 1.0;
  float sin_heading_ = 0.0;

  std::vector<Vec2d> corners_;

  float max_x_ = std::numeric_limits<float>::lowest();
  float min_x_ = std::numeric_limits<float>::max();
  float max_y_ = std::numeric_limits<float>::lowest();
  float min_y_ = std::numeric_limits<float>::max();
};

}  // namespace planning


