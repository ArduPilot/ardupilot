#pragma once 

#include <vector>

#include "vec2d.h"

namespace planning {

    struct PathPoint {
        // coordinates
        float x;
        float y;
        float z;

        // direction on the x-y plane
        float theta;
        // curvature on the x-y plane
        float kappa;
        // accumulated distance from beginning of the path
        float s;
    };

    struct TrajectoryPoint {
        // path point
        struct PathPoint path_point;
        // linear velocity
        float v;
        // linear acceleration
        float a;
        // relative time from begining of trajectory
        float relative_time;
        // longitudinal jerk
        float da;
    };

    class DiscretizedTrajectory : public std::vector<TrajectoryPoint> {
    public:
        DiscretizedTrajectory() = default;
        virtual ~DiscretizedTrajectory() = default;

        // create a DiscretizedTrajectory
        explicit DiscretizedTrajectory(const std::vector<TrajectoryPoint>& trajectory_points);

        virtual TrajectoryPoint StartPoint() const;

        virtual float GetTemporalLength() const;

        virtual float GetSpatialLength() const;

        virtual TrajectoryPoint Evaluate(const float relative_time) const;

        virtual size_t QueryLowerBoundPoint(const float relative_time, const float epsilon = 1.0e-5) const;

        virtual size_t QueryNearestPoint(const Vec2d& position) const;

        size_t QueryNearestPointWithBuffer(const Vec2d& position, const float buffer) const;

        virtual void AppendTrajectoryPoint(const TrajectoryPoint& trajectory_point);

        void PrependTrajectoryPoints( const std::vector<TrajectoryPoint>& trajectory_points);
        
        const TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

        size_t NumOfPoints() const;

        virtual void Clear();
    };

    inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

    inline void DiscretizedTrajectory::Clear() { clear(); }


    PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const float s);

    TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const float t);
}