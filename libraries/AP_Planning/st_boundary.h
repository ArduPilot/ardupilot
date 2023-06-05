#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "box2d.h"
#include "polygon2d.h"
#include "vec2d.h"
#include "st_point.h"

namespace planning {

class STBoundary : public Polygon2d {
public:
    // STBoundary must be initialized with a vector of ST-point pairs
    // each pari refers to a time t, with (lower_s, upper_s)
    STBoundary() = default;
    explicit STBoundary(
        const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
        bool is_accurate_boundary = false);
    
    explicit STBoundary(const Box2d& box) = delete;
    explicit STBoundary(std::vector<Vec2d> points) = delete;

    // wrapper of the constructor
    static STBoundary create_instance(const std::vector<STPoint>& lower_points,
                                      const std::vector<STPoint>& upper_points);

    // wrapper of constructor. it doesn't use remove redundant points and generates an accurate ST-boudary
    static STBoundary create_instance_accurate(const std::vector<STPoint>& lower_points,
                                               const std::vector<STPoint>& upper_points);
    
    // default destructor
    ~STBoundary() = default;

    bool is_empty() const { return _lower_points.empty(); }

    bool get_unblock_srange(const float curr_time, float* s_upper, float* s_lower);

    bool get_boundary_srange(const float curr_time, float* s_upper, float* s_lower);

    bool get_boundary_slopes(const float curr_time, float* ds_upper, float* ds_lower);

    float min_s() const;
    float min_t() const;
    float max_s() const;
    float max_t() const;

    std::vector<STPoint> upper_points() const { return _upper_points; }
    std::vector<STPoint> lower_points() const { return _lower_points; }

    // use bu st-optimizer
    bool is_point_in_boundary(const STPoint& st_point) const;
    STBoundary expand_by_s(const float s) const;
    STBoundary expand_by_t(const float t) const;
    
private:
    // sanity check function for a vector of ST-point pairs
    bool is_valid(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

    // return true if point is within max_dist distance to seg
    bool is_point_near(const LineSegment2d& seg, const Vec2d& point, const float max_dist);

    // sometimes a sequence of lower and upper points lie almost on two straightlines,
    // in this case, the intermediate points are removed,
    // with only the end-points retained
    void remove_redundant_points(std::vector<std::pair<STPoint, STPoint>>* point_pairs);

    // give time t, find a segment denoted by left and right index, that contains the time t
    // if t is less than all or larger than all, return false
    bool get_index_range(const std::vector<STPoint>& points, const float t, size_t* left, size_t* right) const;

private:
    std::vector<STPoint> _upper_points;
    std::vector<STPoint> _lower_points;

    float _min_s = std::numeric_limits<float>::max();
    float _max_s = std::numeric_limits<float>::lowest();
    float _min_t = std::numeric_limits<float>::max();
    float _max_t = std::numeric_limits<float>::lowest();

    STPoint _bottom_left_point;
    STPoint _bottom_right_point;
    STPoint _upper_left_point;
    STPoint _upper_right_point;

};


} // namespace planning