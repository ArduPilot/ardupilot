/**
 * @file
 * @brief Define the Polygon2d class.
 */

#pragma once

#include <string>
#include <vector>
#include "box2d.h"
#include "line_segment2d.h"
#include "vec2d.h"


namespace planning {

/**
 * @class Polygon2d
 * @brief The class of polygon in 2-D.
 */
class Polygon2d {
 public:
  /**
   * @brief Empty constructor.
   */
  Polygon2d() = default;

  /**
   * @brief Constructor which takes a box.
   * @param box The box to construct the polygon.
   */
  explicit Polygon2d(const Box2d &box);

  /**
   * @brief Constructor which takes a vector of points as its vertices.
   * @param points The points to construct the polygon.
   */
  explicit Polygon2d(std::vector<Vec2d> points);

  /**
   * @brief Get the vertices of the polygon.
   * @return The vertices of the polygon.
   */
  const std::vector<Vec2d> &points() const { return points_; }

  /**
   * @brief Get the edges of the polygon.
   * @return The edges of the polygon.
   */
  const std::vector<LineSegment2d> &line_segments() const {
    return line_segments_;
  }

  /**
   * @brief Get the number of vertices of the polygon.
   * @return The number of vertices of the polygon.
   */
  int num_points() const { return num_points_; }

  /**
   * @brief Check if the polygon is convex.
   * @return Whether the polygon is convex or not.
   */
  bool is_convex() const { return is_convex_; }

  /**
   * @brief Get the area of the polygon.
   * @return The area of the polygon.
   */
  float area() const { return area_; }

  /**
   * @brief Compute the distance from a point to the boundary of the polygon.
   *        This distance is equal to the minimal distance from the point
   *        to the edges of the polygon.
   * @param point The point to compute whose distance to the polygon.
   * @return The distance from the point to the polygon's boundary.
   */
  float DistanceToBoundary(const Vec2d &point) const;

  /**
   * @brief Compute the distance from a point to the polygon. If the point is
   *        within the polygon, return 0. Otherwise, this distance is
   *        the minimal distance from the point to the edges of the polygon.
   * @param point The point to compute whose distance to the polygon.
   * @return The distance from the point to the polygon.
   */
  float DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the distance from a line segment to the polygon.
   *        If the line segment is within the polygon, or it has intersect with
   *        the polygon, return 0. Otherwise, this distance is
   *        the minimal distance between the distances from the two ends
   *        of the line segment to the polygon.
   * @param line_segment The line segment to compute whose distance to
   *        the polygon.
   * @return The distance from the line segment to the polygon.
   */
  float DistanceTo(const LineSegment2d &line_segment) const;

  /**
   * @brief Compute the distance from a box to the polygon.
   *        If the box is within the polygon, or it has overlap with
   *        the polygon, return 0. Otherwise, this distance is
   *        the minimal distance among the distances from the edges
   *        of the box to the polygon.
   * @param box The box to compute whose distance to the polygon.
   * @return The distance from the box to the polygon.
   */
  float DistanceTo(const Box2d &box) const;

  /**
   * @brief Compute the distance from another polygon to the polygon.
   *        If the other polygon is within this polygon, or it has overlap with
   *        this polygon, return 0. Otherwise, this distance is
   *        the minimal distance among the distances from the edges
   *        of the other polygon to this polygon.
   * @param polygon The polygon to compute whose distance to this polygon.
   * @return The distance from the other polygon to this polygon.
   */
  float DistanceTo(const Polygon2d &polygon) const;

  /**
   * @brief Compute the square of distance from a point to the polygon.
   *        If the point is within the polygon, return 0. Otherwise,
   *        this square of distance is the minimal square of distance from
   *        the point to the edges of the polygon.
   * @param point The point to compute whose square of distance to the polygon.
   * @return The square of distance from the point to the polygon.
   */
  float DistanceSquareTo(const Vec2d &point) const;

  /**
   * @brief Check if a point is within the polygon.
   * @param point The target point. To check if it is within the polygon.
   * @return Whether a point is within the polygon or not.
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Check if a point is on the boundary of the polygon.
   * @param point The target point. To check if it is on the boundary
   *        of the polygon.
   * @return Whether a point is on the boundary of the polygon or not.
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Check if the polygon contains a line segment.
   * @param line_segment The target line segment. To check if the polygon
   *        contains it.
   * @return Whether the polygon contains the line segment or not.
   */
  bool Contains(const LineSegment2d &line_segment) const;

  /**
   * @brief Check if the polygon contains another polygon.
   * @param polygon The target polygon. To check if this polygon contains it.
   * @return Whether this polygon contains another polygon or not.
   */
  bool Contains(const Polygon2d &polygon) const;

  /**
   * @brief Compute the convex hull of a group of points.
   * @param points The target points. To compute the convex hull of them.
   * @param polygon The convex hull of the points.
   * @return If successfully compute the convex hull.
   */
  static bool ComputeConvexHull(const std::vector<Vec2d> &points,
                                Polygon2d *const polygon);

  /**
   * @brief Check if a line segment has overlap with this polygon.
   * @param line_segment The target line segment. To check if it has
   *        overlap with this polygon.
   * @return Whether the target line segment has overlap with this
   *         polygon or not.
   */
  bool HasOverlap(const LineSegment2d &line_segment) const;

  /**
   * @brief Get the overlap of a line segment and this polygon. If they have
   *        overlap, output the two ends of the overlapped line segment.
   * @param line_segment The target line segment. To get its overlap with
   *         this polygon.
   * @param first First end of the overlapped line segment.
   * @param second Second end of the overlapped line segment.
   * @return If the target line segment has overlap with this polygon.
   */
  bool GetOverlap(const LineSegment2d &line_segment, Vec2d *const first,
                  Vec2d *const last) const;

  /**
   * @brief Get all overlapped line segments of a line segment and this polygon.
   *        There are possibly multiple overlapped line segments if this
   *        polygon is not convex.
   * @param line_segment The target line segment. To get its all overlapped
   *        line segments with this polygon.
   * @return A group of overlapped line segments.
   */
  std::vector<LineSegment2d> GetAllOverlaps(
      const LineSegment2d &line_segment) const;

  /**
   * @brief Check if this polygon has overlap with another polygon.
   * @param polygon The target polygon. To check if it has overlap
   *        with this polygon.
   * @return If this polygon has overlap with another polygon.
   */
  bool HasOverlap(const Polygon2d &polygon) const;

  bool HasOverlap(const Box2d &box) const;

  // Only compute overlaps between two convex polygons.
  /**
   * @brief Compute the overlap of this polygon and the other polygon if any.
   *        Note: this function only works for computing overlap between
   *        two convex polygons.
   * @param other_polygon The target polygon. To compute its overlap with
   *        this polygon.
   * @param overlap_polygon The overlapped polygon.
   * @param If there is a overlapped polygon.
   */
  bool ComputeOverlap(const Polygon2d &other_polygon,
                      Polygon2d *const overlap_polygon) const;

  /**
   * @brief Get the axis-aligned bound box of the polygon.
   * @return The axis-aligned bound box of the polygon.
   */
  AABox2d AABoundingBox() const;

  /**
   * @brief Get the bound box according to a heading.
   * @param heading The specified heading of the bounding box.
   * @return The bound box according to the specified heading.
   */
  Box2d BoundingBoxWithHeading(const float heading) const;

  /**
   * @brief Get the bounding box with the minimal area.
   * @return The bounding box with the minimal area.
   */
  Box2d MinAreaBoundingBox() const;

  /**
   * @brief Get the extreme points along a heading direction.
   * @param heading The specified heading.
   * @param first The point on the boundary of this polygon with the minimal
   *        projection onto the heading direction.
   * @param last The point on the boundary of this polygon with the maximal
   *        projection onto the heading direction.
   */
  void ExtremePoints(const float heading, Vec2d *const first,
                     Vec2d *const last) const;

  /**
   * @brief Expand this polygon by a distance.
   * @param distance The specified distance. To expand this polygon by it.
   * @return The polygon after expansion.
   */
  Polygon2d ExpandByDistance(const float distance) const;

  float min_x() const { return min_x_; }
  float max_x() const { return max_x_; }
  float min_y() const { return min_y_; }
  float max_y() const { return max_y_; }

 protected:
  void BuildFromPoints();
  int Next(int at) const;
  int Prev(int at) const;

  static bool ClipConvexHull(const LineSegment2d &line_segment,
                             std::vector<Vec2d> *const points);

  std::vector<Vec2d> points_;
  int num_points_ = 0;
  std::vector<LineSegment2d> line_segments_;
  bool is_convex_ = false;
  float area_ = 0.0;
  float min_x_ = 0.0;
  float max_x_ = 0.0;
  float min_y_ = 0.0;
  float max_y_ = 0.0;
};

}  // namespace planning
