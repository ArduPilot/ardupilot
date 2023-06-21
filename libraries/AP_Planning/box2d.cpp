#include "box2d.h"

#include <algorithm>
#include <cmath>

#include "math_utils.h"
#include "polygon2d.h"

namespace planning {

namespace {

float PtSegDistance(float query_x, float query_y, float start_x,
                     float start_y, float end_x, float end_y,
                     float length) {
  const float x0 = query_x - start_x;
  const float y0 = query_y - start_y;
  const float dx = end_x - start_x;
  const float dy = end_y - start_y;
  const float proj = x0 * dx + y0 * dy;
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}

}  // namespace

Box2d::Box2d(const Vec2d &center, const float heading, const float length,
             const float width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
  InitCorners();
}

Box2d::Box2d(const LineSegment2d &axis, const float width)
    : center_(axis.center()),
      length_(axis.length()),
      width_(width),
      half_length_(axis.length() / 2.0),
      half_width_(width / 2.0),
      heading_(axis.heading()),
      cos_heading_(axis.cos_heading()),
      sin_heading_(axis.sin_heading()) {
  InitCorners();
}

void Box2d::InitCorners() {
  const float dx1 = cos_heading_ * half_length_;
  const float dy1 = sin_heading_ * half_length_;
  const float dx2 = sin_heading_ * half_width_;
  const float dy2 = -cos_heading_ * half_width_;
  corners_.clear();
  corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

  for (auto &corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}

Box2d::Box2d(const AABox2d &aabox)
    : center_(aabox.center()),
      length_(aabox.length()),
      width_(aabox.width()),
      half_length_(aabox.half_length()),
      half_width_(aabox.half_width()),
      heading_(0.0),
      cos_heading_(1.0),
      sin_heading_(0.0),
      min_x_(aabox.min_x()), min_y_(aabox.min_y()),
      max_x_(aabox.max_x()), max_y_(aabox.max_y()) {
      aabox.GetAllCorners(&corners_); 
}

Box2d Box2d::CreateAABox(const Vec2d &one_corner,
                         const Vec2d &opposite_corner) {
  const float x1 = std::min(one_corner.x(), opposite_corner.x());
  const float x2 = std::max(one_corner.x(), opposite_corner.x());
  const float y1 = std::min(one_corner.y(), opposite_corner.y());
  const float y2 = std::max(one_corner.y(), opposite_corner.y());
  return Box2d({(x1 + x2) / 2.0f, (y1 + y2) / 2.0f}, 0.0f, x2 - x1, y2 - y1);
}

void Box2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
  if (corners == nullptr) {
    return;
  }
  *corners = corners_;
}

std::vector<Vec2d> Box2d::GetAllCorners() const { return corners_; }

bool Box2d::IsPointIn(const Vec2d &point) const {
  const float x0 = point.x() - center_.x();
  const float y0 = point.y() - center_.y();
  const float dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const float dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
  return dx <= half_length_ + kMathEpsilon && dy <= half_width_ + kMathEpsilon;
}

bool Box2d::IsPointOnBoundary(const Vec2d &point) const {
  const float x0 = point.x() - center_.x();
  const float y0 = point.y() - center_.y();
  const float dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const float dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

float Box2d::DistanceTo(const Vec2d &point) const {
  const float x0 = point.x() - center_.x();
  const float y0 = point.y() - center_.y();
  const float dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const float dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0f, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

bool Box2d::HasOverlap(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return IsPointIn(line_segment.start());
  }
  if (std::fmax(line_segment.start().x(), line_segment.end().x()) < min_x() ||
      std::fmin(line_segment.start().x(), line_segment.end().x()) > max_x() ||
      std::fmax(line_segment.start().y(), line_segment.end().y()) < min_y() ||
      std::fmin(line_segment.start().y(), line_segment.end().y()) > max_y()) {
    return false;
  }
  return DistanceTo(line_segment) <= kMathEpsilon;
}

float Box2d::DistanceTo(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return DistanceTo(line_segment.start());
  }
  const float ref_x1 = line_segment.start().x() - center_.x();
  const float ref_y1 = line_segment.start().y() - center_.y();
  float x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
  float y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
  float box_x = half_length_;
  float box_y = half_width_;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }
  const float ref_x2 = line_segment.end().x() - center_.x();
  const float ref_y2 = line_segment.end().y() - center_.y();
  float x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
  float y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                             line_segment.length());
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 2:
        return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                         line_segment.length())
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case -1:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -4:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length())
                   : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                          line_segment.length()));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -3:
        return 0.0;
    }
  }
  return 0.0;
}

float Box2d::DistanceTo(const Box2d &box) const {
  return Polygon2d(box).DistanceTo(*this);
}

bool Box2d::HasOverlap(const Box2d &box) const {
  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
      box.min_y() > max_y()) {
    return false;
  }

  const float shift_x = box.center_x() - center_.x();
  const float shift_y = box.center_y() - center_.y();

  const float dx1 = cos_heading_ * half_length_;
  const float dy1 = sin_heading_ * half_length_;
  const float dx2 = sin_heading_ * half_width_;
  const float dy2 = -cos_heading_ * half_width_;
  const float dx3 = box.cos_heading() * box.half_length();
  const float dy3 = box.sin_heading() * box.half_length();
  const float dx4 = box.sin_heading() * box.half_width();
  const float dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}

AABox2d Box2d::GetAABox() const {
  const float dx1 = std::abs(cos_heading_ * half_length_);
  const float dy1 = std::abs(sin_heading_ * half_length_);
  const float dx2 = std::abs(sin_heading_ * half_width_);
  const float dy2 = std::abs(cos_heading_ * half_width_);
  return AABox2d(center_, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0);
}

void Box2d::RotateFromCenter(const float rotate_angle) {
  heading_ = NormalizeAngle(heading_ + rotate_angle);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
  InitCorners();
}

void Box2d::Shift(const Vec2d &shift_vec) {
  center_ += shift_vec;
  InitCorners();
}

void Box2d::LongitudinalExtend(const float extension_length) {
  length_ += extension_length;
  half_length_ += extension_length / 2.0;
  InitCorners();
}

void Box2d::LateralExtend(const float extension_length) {
  width_ += extension_length;
  half_width_ += extension_length / 2.0;
  InitCorners();
}

}  // namespace planning

