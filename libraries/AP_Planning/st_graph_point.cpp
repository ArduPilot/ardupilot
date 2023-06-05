#include "st_graph_point.h"

namespace planning {

std::uint32_t StGraphPoint::index_s() const
{ 
    return _index_s;
}

std::uint32_t StGraphPoint::index_t() const
{ 
    return _index_t; 
}

const STPoint& StGraphPoint::point() const 
{ 
    return _point; 
}

const StGraphPoint* StGraphPoint::pre_point() const 
{ 
    return _pre_point; 
}

float StGraphPoint::reference_cost() const 
{ 
    return _reference_cost; 
}

float StGraphPoint::obstacle_cost() const 
{ 
    return _obstacle_cost; 
}

float StGraphPoint::spatial_potential_cost() const
{
    return _spatial_potential_cost;
}

float StGraphPoint::total_cost() const
{
    return _total_cost;
}

void StGraphPoint::init(const std::uint32_t index_t, const std::uint32_t index_s,
            const STPoint& st_point)
{
    _index_s = index_s;
    _index_t = index_t;
    _point = st_point;
}

// give reference speed profile, reach the cost, including position
void StGraphPoint::set_reference_cost(const float reference_cost)
{
    _reference_cost = reference_cost;
}

// give obstacle info, get the cost
void StGraphPoint::set_obstacle_cost(const float obs_cost)
{
    _obstacle_cost = obs_cost;
}

// give potential cost for minimal time traversal
void StGraphPoint::set_spatial_potential_cost(const float spatial_potential_cost)
{
    _spatial_potential_cost = spatial_potential_cost;
}

// total cost
void StGraphPoint::set_total_cost(const float total_cost)
{
    _total_cost = total_cost;
}

void StGraphPoint::set_pre_point(const StGraphPoint& pre_point)
{
    _pre_point = &pre_point;
}

float StGraphPoint::get_optimal_speed() const
{
    return _optimal_speed;
}

void StGraphPoint::set_optimal_speed(const float optimal_speed)
{
    _optimal_speed = optimal_speed;
}


} // namespace planning