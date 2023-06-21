#include "discretized_trajectory.h"
#include "linear_interpolation.h"

#include <algorithm>
#include <utility>
#include <limits>
#include <iostream>


namespace planning {
// create a DiscretizedTrajectory
DiscretizedTrajectory::DiscretizedTrajectory(const std::vector<TrajectoryPoint>& trajectory_points) : std::vector<TrajectoryPoint>(trajectory_points)
{
    if (trajectory_points.empty()) {
        std::cout << "trajectory_points should NOT be empty()" << std::endl;
    }
}

// get start point
TrajectoryPoint DiscretizedTrajectory::StartPoint() const
{
    return front();
}

// get trajectory time length
float DiscretizedTrajectory::GetTemporalLength() const
{
    if (empty()) {
        return 0.0f;
    }
    return  back().relative_time - front().relative_time;
}

// get trajectory space legnth
float DiscretizedTrajectory::GetSpatialLength() const
{
    if (empty()) {
        return 0.0f;
    }
    return back().path_point.s - front().path_point.s;
}


// get trajectory point at time, at same time, interpolation is carried out
TrajectoryPoint DiscretizedTrajectory::Evaluate(const float relative_time) const
{
  auto comp = [](const TrajectoryPoint& p, const float time) {
    return p.relative_time < time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
      return front();
  } else if (it_lower == end()) {
      return back();
  }

   return InterpolateUsingLinearApproximation( *(it_lower - 1), *it_lower, relative_time);
}

// search for temporal projection
size_t DiscretizedTrajectory::QueryLowerBoundPoint(const float relative_time, const float epsilon) const
{
    if (empty()) {
        return 0;
    }

    if (relative_time >= back().relative_time) {
        return size() - 1;
    }

    auto func = [&epsilon](const TrajectoryPoint& tp,
                            const float time) {
        return tp.relative_time + epsilon < time;
    };

    auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
    return std::distance(begin(), it_lower);
}

// search for space projection
size_t DiscretizedTrajectory::QueryNearestPoint(const Vec2d& position) const
{
    float dist_sqr_min = std::numeric_limits<float>::max();
    size_t index_min = 0;
    for (size_t i = 0; i < size(); ++i) {
        const Vec2d curr_point(data()[i].path_point.x, data()[i].path_point.y);

        const float dist_sqr = curr_point.DistanceSquareTo(position);
        if (dist_sqr < dist_sqr_min) {
            dist_sqr_min = dist_sqr;
            index_min = i;
        }
    }
    return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(const Vec2d& position, const float buffer) const
{
  float dist_sqr_min = std::numeric_limits<float>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
     const Vec2d curr_point(data()[i].path_point.x, data()[i].path_point.y);

    const float dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

// add point to the end of trajectory
void DiscretizedTrajectory::AppendTrajectoryPoint(const TrajectoryPoint& trajectory_point)
{
    if (!empty() && trajectory_point.relative_time < back().relative_time) {
        return;
    }

    push_back(trajectory_point);
}

// add new trajectory into the top of old trajectory
void DiscretizedTrajectory::PrependTrajectoryPoints( const std::vector<TrajectoryPoint>& trajectory_points)
{
    if (!empty() && trajectory_points.size() > 1 && trajectory_points.back().relative_time >= front().relative_time) {
        return;
    }

    insert(begin(), trajectory_points.begin(), trajectory_points.end());
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(const size_t index) const
{
    return data()[index];
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const float s) {
  float s0 = p0.s;
  float s1 = p1.s;

  PathPoint path_point;
  float weight = (s - s0) / (s1 - s0);
  float x = (1 - weight) * p0.x + weight * p1.x;
  float y = (1 - weight) * p0.y + weight * p1.y;
  float theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  float kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  path_point.x = x;
  path_point.y = y;
  path_point.theta = theta;
  path_point.kappa = kappa;
  path_point.s = s;
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const float t) {

  const PathPoint pp0 = tp0.path_point;
  const PathPoint pp1 = tp1.path_point;
  float t0 = tp0.relative_time;
  float t1 = tp1.relative_time;

  TrajectoryPoint tp;
  tp.v = lerp(tp0.v, t0, tp1.v, t1, t);
  tp.a = lerp(tp0.a, t0, tp1.a, t1, t);
  tp.relative_time = t;

  PathPoint *path_point = &tp.path_point;
  path_point->x = (lerp(pp0.x, t0, pp1.x, t1, t));
  path_point->y = (lerp(pp0.y, t0, pp1.y, t1, t));
  path_point->theta = (slerp(pp0.theta, t0, pp1.theta, t1, t));
  path_point->kappa = (lerp(pp0.kappa, t0, pp1.kappa, t1, t));
  path_point->s = (lerp(pp0.s, t0, pp1.s, t1, t));

  return tp;
}
}