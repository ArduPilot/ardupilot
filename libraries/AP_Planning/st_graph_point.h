/// @file st_graph_point.h
#pragma once

#include <limits>
#include "st_point.h"

namespace planning {

class StGraphPoint {
public:
    std::uint32_t index_s() const;
    std::uint32_t index_t() const;

    const STPoint& point() const;
    const StGraphPoint* pre_point() const;

    float reference_cost() const;
    float obstacle_cost() const;
    float spatial_potential_cost() const;
    float total_cost() const;

    void init(const std::uint32_t index_t, const std::uint32_t index_s,
              const STPoint& st_point);
    
    // give reference speed profile, reach the cost, including position
    void set_reference_cost(const float reference_cost);

    // give obstacle info, get the cost
    void set_obstacle_cost(const float obs_cost);

    // give potential cost for minimal time traversal
    void set_spatial_potential_cost(const float spatial_potential_cost);

    // total cost
    void set_total_cost(const float total_cost);

    void set_pre_point(const StGraphPoint& pre_point);

    float get_optimal_speed() const;

    void set_optimal_speed(const float optimal_speed);

private:
    STPoint _point;
    const StGraphPoint* _pre_point{nullptr};
    std::uint32_t _index_s{0};
    std::uint32_t _index_t{0};

    float _optimal_speed{0.0f};
    float _reference_cost{0.0f};
    float _obstacle_cost{0.0f};
    float _spatial_potential_cost{0.0f};
    float _total_cost{std::numeric_limits<float>::infinity()};


};

} // namespace planning
