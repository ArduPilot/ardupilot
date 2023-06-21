#include "st_boundary.h"
#include "math_utils.h"

#include <algorithm>

#include <stdio.h>

namespace planning {

STBoundary::STBoundary(
const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
bool is_accurate_boundary)
{
    if (!is_valid(point_pairs)) {
        return;
    }
        std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
    if (!is_accurate_boundary) {
        remove_redundant_points(&reduced_pairs);
    }

    for (const auto& item : reduced_pairs) {
        // use same t for both points
        const float t = item.first.t();
        _lower_points.emplace_back(item.first.s(), t);
        _upper_points.emplace_back(item.second.s(), t);
    }

    for (const auto& point : _lower_points) {
        points_.emplace_back(point.t(), point.s());
    }
    for (auto rit = _upper_points.rbegin(); rit != _upper_points.rend(); ++rit) {
        points_.emplace_back(rit->t(), rit->s());
    }


    BuildFromPoints();

    for (const auto& point : _lower_points) {
        _min_s = std::fmin(_min_s, point.s());
    }
    for (const auto& point : _upper_points) {
        _max_s = std::fmax(_max_s, point.s());
    }
    _min_t = _lower_points.front().t();
    _max_t = _lower_points.back().t();
}



// wrapper of the constructor
STBoundary STBoundary::create_instance(const std::vector<STPoint>& lower_points,
                                    const std::vector<STPoint>& upper_points)
{
    if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
        return STBoundary();
    }

    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    for (size_t i = 0; i < lower_points.size(); ++i) {
        point_pairs.emplace_back(
            STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
            STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
    }
    return STBoundary(point_pairs);
    return STBoundary(std::move(point_pairs));
}

// wrapper of constructor. it doesn't use remove redundant points and generates an accurate ST-boudary
STBoundary STBoundary::create_instance_accurate(const std::vector<STPoint>& lower_points,
                                               const std::vector<STPoint>& upper_points)
{
    if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
        return STBoundary();
    }

    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    for (size_t i = 0; i < lower_points.size(); ++i) {
        point_pairs.emplace_back(
            STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
            STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
    }
    return STBoundary(point_pairs, true);
}


bool STBoundary::get_unblock_srange(const float curr_time, float* s_upper, float* s_lower)
{
    *s_upper = 1500.0f;
    *s_lower = 0.0;
    if (curr_time < _min_t || curr_time > _max_t) {
        return true;
    }

    size_t left = 0;
    size_t right = 0;
    if (!get_index_range(_lower_points, curr_time, &left, &right)) {
        printf("get_unblock_srange: Fail to get index range.\n");
        return false;
    }

    if (curr_time > _upper_points[right].t()) {
        return true;
    }

    const float r =
        (left == right
            ? 0.0
            : (curr_time - _upper_points[left].t()) /
                    (_upper_points[right].t() - _upper_points[left].t()));

    *s_upper = _upper_points[left].s() +
                r * (_upper_points[right].s() - _upper_points[left].s());

    *s_lower = _lower_points[left].s() +
                r * (_lower_points[right].s() - _lower_points[left].s());


    *s_upper = std::fmin(*s_upper, 1000.0f);
    *s_lower = std::fmax(*s_lower, 0.0);
 
    return true;
}

bool STBoundary::get_boundary_srange(const float curr_time, float* s_upper, float* s_lower)
{
    if (curr_time < _min_t || curr_time > _max_t) {
        return false;
    }

    size_t left = 0;
    size_t right = 0;
    if (!get_index_range(_lower_points, curr_time, &left, &right)) {
        printf("get_boundary_srange: Fail to get index range.\n");
        return false;
    }
    const float r =
        (left == right
            ? 0.0
            : (curr_time - _upper_points[left].t()) /
                    (_upper_points[right].t() - _upper_points[left].t()));

    *s_upper = _upper_points[left].s() +
                r * (_upper_points[right].s() - _upper_points[left].s());
    *s_lower = _lower_points[left].s() +
                r * (_lower_points[right].s() - _lower_points[left].s());

    *s_upper = std::fmin(*s_upper, 1000.0f);
    *s_lower = std::fmax(*s_lower, 0.0);
    return true;
}

bool STBoundary::get_boundary_slopes(const float curr_time, float* ds_upper, float* ds_lower)
{
    if (ds_upper == nullptr || ds_lower == nullptr) {
        return false;
    }
    if (curr_time < _min_t || curr_time > _max_t) {
        return false;
    }

    static constexpr float kTimeIncrement = 0.05;
    float t_prev = curr_time - kTimeIncrement;
    float prev_s_upper = 0.0;
    float prev_s_lower = 0.0;
    bool has_prev = get_boundary_srange(t_prev, &prev_s_upper, &prev_s_lower);
    float t_next = curr_time + kTimeIncrement;
    float next_s_upper = 0.0;
    float next_s_lower = 0.0;
    bool has_next = get_boundary_srange(t_next, &next_s_upper, &next_s_lower);
    float curr_s_upper = 0.0;
    float curr_s_lower = 0.0;
    get_boundary_srange(curr_time, &curr_s_upper, &curr_s_lower);
    if (!has_prev && !has_next) {
        return false;
    }
    if (has_prev && has_next) {
        *ds_upper = ((next_s_upper - curr_s_upper) / kTimeIncrement +
                    (curr_s_upper - prev_s_upper) / kTimeIncrement) *
                    0.5;
        *ds_lower = ((next_s_lower - curr_s_lower) / kTimeIncrement +
                    (curr_s_lower - prev_s_lower) / kTimeIncrement) *
                    0.5;
        return true;
    }
    if (has_prev) {
        *ds_upper = (curr_s_upper - prev_s_upper) / kTimeIncrement;
        *ds_lower = (curr_s_lower - prev_s_lower) / kTimeIncrement;
    } else {
        *ds_upper = (next_s_upper - curr_s_upper) / kTimeIncrement;
        *ds_lower = (next_s_lower - curr_s_lower) / kTimeIncrement;
    }
    return true;
}

float STBoundary::min_s() const
{
    return _min_s;
}

float STBoundary::min_t() const
{
    return _min_t;
}

float STBoundary::max_s() const
{
    return _max_s;
}

float STBoundary::max_t() const
{
    return _max_t;
}


// use bu st-optimizer
bool STBoundary::is_point_in_boundary(const STPoint& st_point) const
{
    if (st_point.t() < _min_t || st_point.t() > _max_t) {
        return false;
    }
    size_t left = 0;
    size_t right = 0;
    if (!get_index_range(_lower_points, st_point.t(), &left, &right)) {
        return false;
    }
    const float check_upper = planning::CrossProd(
        st_point, _upper_points[left], _upper_points[right]);
    const float check_lower = planning::CrossProd(
        st_point, _lower_points[left], _lower_points[right]);

    return (check_upper * check_lower < 0);
}

STBoundary STBoundary::expand_by_s(const float s) const
    {
    if (_lower_points.empty()) {
        return STBoundary();
    }
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    for (size_t i = 0; i < _lower_points.size(); ++i) {
        point_pairs.emplace_back(
            STPoint(_lower_points[i].s() - s, _lower_points[i].t()),
            STPoint(_upper_points[i].s() + s, _upper_points[i].t()));
    }
    return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::expand_by_t(const float t) const
{
    if (_lower_points.empty()) {
        return STBoundary();
    }

    std::vector<std::pair<STPoint, STPoint>> point_pairs;

    const float left_delta_t = _lower_points[1].t() - _lower_points[0].t();
    const float lower_left_delta_s = _lower_points[1].s() - _lower_points[0].s();
    const float upper_left_delta_s = _upper_points[1].s() - _upper_points[0].s();

    point_pairs.emplace_back(
        STPoint(_lower_points[0].s() - t * lower_left_delta_s / left_delta_t,
                _lower_points[0].t() - t),
        STPoint(_upper_points[0].s() - t * upper_left_delta_s / left_delta_t,
                _upper_points.front().t() - t));

    const float kMinSEpsilon = 1e-3;
    point_pairs.front().first.set_s(
        std::fmin(point_pairs.front().second.s() - kMinSEpsilon,
                    point_pairs.front().first.s()));

    for (size_t i = 0; i < _lower_points.size(); ++i) {
        point_pairs.emplace_back(_lower_points[i], _upper_points[i]);
    }

    size_t length = _lower_points.size();
    if (length < 2) {
        return STBoundary();
    }

    const float right_delta_t =
        _lower_points[length - 1].t() - _lower_points[length - 2].t();
    const float lower_right_delta_s =
        _lower_points[length - 1].s() - _lower_points[length - 2].s();
    const float upper_right_delta_s =
        _upper_points[length - 1].s() - _upper_points[length - 2].s();

    point_pairs.emplace_back(STPoint(_lower_points.back().s() +
                                        t * lower_right_delta_s / right_delta_t,
                                    _lower_points.back().t() + t),
                            STPoint(_upper_points.back().s() +
                                        t * upper_right_delta_s / right_delta_t,
                                    _upper_points.back().t() + t));
    point_pairs.back().second.set_s(
        std::fmax(point_pairs.back().second.s(),
                    point_pairs.back().first.s() + kMinSEpsilon));

    return STBoundary(std::move(point_pairs));
}


// sanity check function for a vector of ST-point pairs
bool STBoundary::is_valid(const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const
{
    if (point_pairs.size() < 2) {
        return false;
    }

    static constexpr float kStBoundaryEpsilon = 1e-9f;
    static constexpr float kMinDeltaT = 1e-6f;
    for (size_t i = 0; i < point_pairs.size(); ++i) {
        const auto& curr_lower = point_pairs[i].first;
        const auto& curr_upper = point_pairs[i].second;
        if (curr_upper.s() < curr_lower.s()) {
          return false;
        }
        if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
         return false;
        }
        if (i + 1 != point_pairs.size()) {
        const auto& next_lower = point_pairs[i + 1].first;
        const auto& next_upper = point_pairs[i + 1].second;
        if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
            std::fmin(next_lower.t(), next_upper.t())) {
            return false;
        }
        }
    }
    return true;
}

// return true if point is within max_dist distance to seg
bool STBoundary::is_point_near(const LineSegment2d& seg, const Vec2d& point, const float max_dist)
{
    return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

// sometimes a sequence of lower and upper points lie almost on two straightlines,
// in this case, the intermediate points are removed,
// with only the end-points retained
void STBoundary::remove_redundant_points(std::vector<std::pair<STPoint, STPoint>>* point_pairs)
{
    if (!point_pairs || point_pairs->size() <= 2) {
        return;
    }

    const float kMaxDist = 0.1;
    size_t i = 0;
    size_t j = 1;

    while (i < point_pairs->size() && j + 1 < point_pairs->size()) {
        LineSegment2d lower_seg(point_pairs->at(i).first,
                                point_pairs->at(j + 1).first);

        LineSegment2d upper_seg(point_pairs->at(i).second,
                                point_pairs->at(j + 1).second);

        if (!is_point_near(lower_seg, point_pairs->at(j).first, kMaxDist) ||
            !is_point_near(upper_seg, point_pairs->at(j).second, kMaxDist)) {
        ++i;
        if (i != j) {
            point_pairs->at(i) = point_pairs->at(j);
        }
        }
        ++j;
    }
    point_pairs->at(++i) = point_pairs->back();
    point_pairs->resize(i + 1);
}

// give time t, find a segment denoted by left and right index, that contains the time t
// if t is less than all or larger than all, return false
bool STBoundary::get_index_range(const std::vector<STPoint>& points, const float t, size_t* left, size_t* right) const
{
    if (t < points.front().t() || t > points.back().t()) {
        return false;
    }
    auto comp = [](const STPoint& p, const float t0) { return p.t() < t0; };
    auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);
    size_t index = std::distance(points.begin(), first_ge);
    if (index == 0) {
        *left = *right = 0;
    } else if (first_ge == points.end()) {
        *left = *right = points.size() - 1;
    } else {
        *left = index - 1;
        *right = index;
    }
    return true;
}

} // namespace planning