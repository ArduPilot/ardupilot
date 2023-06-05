#include "AP_SLTPlanner.h"
#include "AP_OADatabase.h"
#include <AC_Fence/AC_Fence.h>
#include <AP_Planning/linear_interpolation.h>
#include <AP_AHRS/AP_AHRS.h>
#include <algorithm>
#include <AP_Planning/math_utils.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_SLTPlanner::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable SLT planner
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_SLTPlanner, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NFE
    // @Description: number of finite elements used to descretize an OCP
    // @Range: 1 320
    // @User: Standard
    AP_GROUPINFO("NFE", 1, AP_SLTPlanner, _nfe, 20),

    // @Param: TF
    // @Description: time horizon length
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("TF", 2, AP_SLTPlanner, _tf, 16),

    // @Param: ROB_SIZE
    // @Description: vehicle width
    // @Units: m
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("ROB_SIZE", 3, AP_SLTPlanner, _vehicle_radius, 0.5f),

    // @Param: OBS_WGT
    // @Description: obstacle cpst
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("OBS_WGT",4 , AP_SLTPlanner, _obstacle_collision_weight, 10),

    // @Param: LAT_WGT
    // @Description: lateral cost
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("LAT_WGT", 5, AP_SLTPlanner, _lateral_weight, 1.0f),

    // @Param: LSAT_WGT
    // @Description: lateral change cost dl/ds
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("LSAT_WGT", 6, AP_SLTPlanner, _lateral_change_weight, 1.5f),

    // @Param: LTAT_WGT
    // @Description: lateral change cost, dl/dt
    // @Units: m/s^2
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("LTAT_WGT", 7, AP_SLTPlanner, _lateral_vel_change_weight, 1.5f),

    // @Param: LON_WGT
    // @Description: longitudinal velocity cost
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("LON_WGT", 8, AP_SLTPlanner, _longitudinal_vel_weight, 10.0f),

    // @Param: LTON_WGT
    // @Description: Cost of longitudinal velocity change, ds/dt
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("LTON_WGT", 9, AP_SLTPlanner, _longitudinal_vel_change_weight, 10.0f),

    // @Param: REF_LB
    // @Description: reference center line left range
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("REF_LB", 10, AP_SLTPlanner, _reference_left_bound, 20.0f),

    // @Param: REF_RB
    // @Description: reference center line right range
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("REF_RB", 11, AP_SLTPlanner, _reference_right_bound, 20.0f),

    // @Param: REF_RES
    // @Description: reference center line resolution
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("REF_RES", 12, AP_SLTPlanner, _reference_resolution, 0.2f),

  AP_GROUPEND
};

AP_SLTPlanner::AP_SLTPlanner() 
{
    AP_Param::setup_object_defaults(this, var_info);
}

// run background task to find best cruise speed and update avoidance results
// returns false if obstacle avoidance is not required
bool AP_SLTPlanner::update(const Location &current_loc, const Location& origin, const Location& destination, 
                          const float cruise_speed, 
                          planning::DiscretizedTrajectory &planned_trajectory_pb, 
                          const float planning_cycle_time)
{
    if (!_enable) {
        _shortest_path_ok = false;
         return false;
    }

    // initial 
    _desired_speed_input = cruise_speed;
    _nseg = _nfe / NT;
    _unit_time = _tf / NT;
    _time = planning::LinSpaced<NT>(_unit_time, _tf);
    _station = planning::LinSpaced<NS>(0.0f, _unit_time * (_desired_speed_input + 1));
    _lateral = planning::LinSpaced<NL-1>(0.0f, 1.0f);

    Vector2f current_ne, origin_ne, destination_ne;
    // convert location with lat-lng to offsets from ekf orgin
    if (!current_loc.get_vector_xy_from_origin_NE(current_ne) ||
        !origin.get_vector_xy_from_origin_NE(origin_ne) ||
        !destination.get_vector_xy_from_origin_NE(destination_ne)) {
        // OA is not required
        _shortest_path_ok = false;
        return false;
    }
    // translate units cm to m
    current_ne *= 0.01f;
    origin_ne *= 0.01f;
    destination_ne *= 0.01f;

    // get ground course
    float ground_course_deg;
    const Vector2f ground_speed_vec = AP::ahrs().groundspeed_vector();
    if (ground_speed_vec.length_squared() < sq(0.2f)) {
        // with zero ground speed use vehicle's heading
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
    } else {
        ground_course_deg = degrees(ground_speed_vec.angle());
    }

    // update reference line
    const bool reference_matches = (_start == origin_ne) && (_end == destination_ne);
    if (!reference_matches) {
        _start = origin_ne;
        _end = destination_ne;
        // calculate bearing and distance to final destination
        const float reference_bearing = origin.get_bearing_to(destination) * 0.01f;
        const float reference_length = origin.get_distance(destination);
        const Vector2f unit_ref = (_end - _start).normalized();
        planning::Trajectory traj;
        float s = 0;
        while ( s < reference_length + _reference_resolution) {
            const Vector2f pt = _start + unit_ref * s;
            planning::TrajPoint tp;
            tp.s = s;
            tp.x = pt.x;
            tp.y = pt.y;
            tp.theta = radians(reference_bearing);
            tp.kappa = 0;
            tp.velocity = _desired_speed_input;
            tp.left_bound = _reference_left_bound;
            tp.right_bound = _reference_right_bound;
            traj.emplace_back(tp);
            s += _reference_resolution;
        }
        set_reference(planning::DiscretizedTraj(traj));
    }
    
    // update obstacle list
    update_obstacles();

    if (_static_obstacles.empty() && _dynamic_obstacles.empty()) {
        _shortest_path_ok = false;
        return false;
    }

    auto sl = _reference.get_projection({current_ne.x, current_ne.y});
    _state.start_s = sl.x();
    _state.start_l = sl.y();
    _state.start_theta = radians(ground_course_deg);

    // reset state space
    for (int i = 0; i < NT; i++) {
        for (int j = 0; j < NS; j++) {
            for (int k = 0; k < NL; k++) {
                _state_space[i][j][k] = StateCell();
            }
        }
    }

    // evaluate first layer
    for (int i = 0; i < NS; i++) {
        for (int j = 0; j < NL; j++) {
            auto tup = get_total_cost(StateIndex(-1, -1, -1), StateIndex(0, i, j));
            _state_space[0][i][j].current_s = tup.first;
            _state_space[0][i][j].cost = tup.second;
        }
    }

    // dynamical programming
    for (int i = 0; i < NT - 1; i++) {
        for (int j = 0; j < NS; j++) {
            for (int k = 0; k < NL; k++) {
                StateIndex parent_ind(i, j, k);

                for (int m = 0; m < NS; m++) {
                    for (int n = 0; n < NL; n++) {
                        StateIndex current_ind(i + 1, m , n);
                        auto tup = get_total_cost(parent_ind, current_ind);
                        const float delta_cost = tup.second;
                        const float cur_s = tup.first;

                        const float cur_cost = _state_space[i][j][k].cost + delta_cost;
                        if (cur_cost < _state_space[i+1][m][n].cost) {
                            _state_space[i+1][m][n] = StateCell(cur_cost, cur_s, j, k);
                        }
                    }
                }
            }
        }
    }

    // find the least cost in final layer
    float min_cost = std::numeric_limits<float>::infinity();
    int16_t min_s_ind = 0, min_l_ind = 0;
    for (int16_t i = 0; i < NS; i++) {
        for (int16_t j = 0; j < NL; j++) {
            const float cost = _state_space[NT - 1][i][j].cost;
            if (cost < min_cost) {
                min_s_ind = i;
                min_l_ind = j;
                min_cost = cost;
            }
        }
    }

    // trace back layers to find optimum traj
    std::vector<std::pair<StateIndex, StateCell>> waypoints(NT);
    for (int16_t i = NT - 1; i >= 0; i--) {
        auto &cell  = _state_space[i][min_s_ind][min_l_ind];
        waypoints[i] = std::make_pair(StateIndex(i, min_s_ind, min_l_ind), cell);
        min_s_ind = cell.parent_s_ind;
        min_l_ind = cell.parent_l_ind;

        // if cost is inf , we should stop vehicle imemediatley
        if (std::isinf(cell.cost)) {
            // generate failsafe stop trajectory
            generate_stop_trajectory(planned_trajectory_pb);
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SLT planner failed, stop vehicle!");
            return true;
        }
    }

  
    // interpolation
    const float dt = _tf / (_nfe - 1);
    planning::Trajectory data(_nfe);
    float last_l = _state.start_l, last_s = _state.start_s;
    float lateral_err_max = 0, speed_err_max = 0;
    for (int16_t i = 0; i < NT; i++) {
        float parent_s = i > 0 ? waypoints[i-1].second.current_s : _state.start_s;
        auto segment = interpolate_linearly(parent_s, waypoints[i].second.parent_l_ind, waypoints[i].first.s, waypoints[i].first.l);

        for (int j = 0; j < _nseg; j++) {
            auto dl = segment[j].y() - last_l;
            auto ds = std::max(segment[j].x() - last_s, planning::kMathEpsilon);
            last_l = segment[j].y();
            last_s = segment[j].x();

            auto xy = _reference.get_cartesian(segment[j].x(), segment[j].y());
            auto tp = _reference.evaluate_station(segment[j].x());
            int n = i * _nseg + j;
            data[n].s = segment[j].x();
            data[n].x = xy.x();
            data[n].y = xy.y();
            data[n].velocity = (n == 0) ? _desired_speed_input : (data[n].s - data[n-1].s) / dt;
            data[n].theta = (n  == 0) ? _state.start_theta : tp.theta + atanf((dl / ds) / (1 - tp.kappa * segment[j].y()));
            // update planning error relative to reference line
            lateral_err_max = std::max(lateral_err_max, std::abs(last_l));
            speed_err_max = std::max(speed_err_max,  std::abs(data[n].velocity - _desired_speed_input));
        }
    }

    // construct planning trajectory
    _planning_traj = std::move(planning::DiscretizedTraj(data));
    _path_numpoints = MIN((uint16_t)_planning_traj.data().size(), 255);

    // exit planning process at the end of reference
    const bool oa_active = (lateral_err_max > 0.5f || speed_err_max > 0.25f) && (_state.start_s < _reference.data().back().s);
    _shortest_path_ok = oa_active;

    // return planned trajectory
    planned_trajectory_pb.clear();
    for (size_t i  = 0; i < _planning_traj.data().size(); i++) {
        planning::TrajectoryPoint point;
        point.path_point.x = _planning_traj.data().at(i).x;
        point.path_point.y = _planning_traj.data().at(i).y;
        point.path_point.theta = _planning_traj.data().at(i).theta;
        point.path_point.s = _planning_traj.data().at(i).s;
        point.path_point.kappa = _planning_traj.data().at(i).kappa;
        point.relative_time = i * dt;
        point.v = _planning_traj.data().at(i).velocity;
        planned_trajectory_pb.emplace_back(point);
    }

    return oa_active;
}


// generate stop trajectory
void AP_SLTPlanner::generate_stop_trajectory(planning::DiscretizedTrajectory& trajectory_data)
{
    Vector3f pos;
    if (!AP::ahrs().get_relative_position_NED_origin(pos)) {
        return;
    }
    float relative_time = 0.0;
    static constexpr int stop_trajectory_length = 10;
    static constexpr double relative_stop_time = 0.1f;
    trajectory_data.clear();
    for (size_t i = 0; i < stop_trajectory_length; i++) {
        planning::TrajectoryPoint point;
        point.path_point.x = pos.x;
        point.path_point.y = pos.y;
        point.path_point.theta = AP::ahrs().yaw;
        point.path_point.s = 0;
        point.path_point.kappa = 0;
        point.relative_time = relative_time;
        point.v = 0.0f;
        trajectory_data.emplace_back(point);
        relative_time += relative_stop_time;
    }
}

// return location point from final path
bool AP_SLTPlanner::get_shortest_path_location(uint8_t point_num, Location& Loc) const
{
    if (!_shortest_path_ok || _path_numpoints == 0 || point_num >= _path_numpoints) {
        // this is acting like a basic semaphore to save doing it properly
        return false;
    }

    // translate unit
    Vector2f pos {_planning_traj.data().at(point_num).x * 100, _planning_traj.data().at(point_num).y * 100};
   
    Location temp_loc(Vector3f{pos.x, pos.y, 0.0}, Location::AltFrame::ABOVE_ORIGIN);
    Loc.lat = temp_loc.lat;
    Loc.lng = temp_loc.lng;
    return true;
}

// set reference line
void AP_SLTPlanner::set_reference(const planning::DiscretizedTraj &reference)
{
    _reference = reference;
    _road_barrier.clear();
    const float start_s = _reference.data().front().s;
    const float back_s = _reference.data().back().s;
    int16_t sample_point = (back_s - start_s) / _reference_resolution;
    for (int16_t i = 0; i <= sample_point; i++) {
        const float s = start_s + i * _reference_resolution;
        auto ref = _reference.evaluate_station(s);
        _road_barrier.push_back(_reference.get_cartesian(s, ref.left_bound));
        _road_barrier.push_back(_reference.get_cartesian(s, -ref.right_bound));
    }

    std::sort(_road_barrier.begin(), _road_barrier.end(), [](const planning::Vec2d &a, const planning::Vec2d &b) {
    return a.x() < b.x();
    });
}


// get collision cost
float AP_SLTPlanner::get_collision_cost(StateIndex parent_ind, StateIndex current_ind)
{
    float parent_s = _state.start_s, grandparent_s = _state.start_s;
    float last_l = _state.start_l, last_s = _state.start_s;
    if (parent_ind.t >= 0 ) {
        auto &cell = _state_space[parent_ind.t][parent_ind.s][parent_ind.l];
        parent_s = cell.current_s;

        if (parent_ind.t > 0) {
            auto &parent_cell = _state_space[parent_ind.t - 1][cell.parent_s_ind][cell.parent_l_ind];
            grandparent_s = parent_cell.current_s;
        }
        
        // get previous path
        auto prev_path = interpolate_linearly(grandparent_s, cell.parent_l_ind, parent_ind.s, parent_ind.l);
        last_l = prev_path.back().y();
        last_s = prev_path.back().x();
    }

    auto path = interpolate_linearly(parent_s, parent_ind.l, current_ind.s, current_ind.l);

    float collision_cost= 0.0f;
    for (uint16_t i = 0; i < path.size(); i++) {
        auto &pt = path[i];
        float dl = pt.y() - last_l;
        float ds = MAX(pt.x() - last_s, planning::kMathEpsilon);
        last_l = pt.y();
        last_s = pt.x();

        auto cart = _reference.get_cartesian(pt.x(), pt.y());
        auto ref  = _reference.evaluate_station(pt.x());
        const float lb = std::min(0.0f, -ref.right_bound + _safe_margin + _vehicle_radius);
        const float ub = std::max(0.0f,  ref.left_bound - _safe_margin - _vehicle_radius);
        if (pt.y() < lb - planning::kMathEpsilon || pt.y() > ub + planning::kMathEpsilon) {
            return std::numeric_limits<float>::infinity();
        }
        const float heading = ref.theta + atanf((dl/ds) / (1 - ref.kappa * pt.y()));
        planning::Pose pose(cart.x(), cart.y(), heading);

        const float parent_time = parent_ind.t < 0 ? 0.0f : _time[parent_ind.t];
        const float time = parent_time + i * (_unit_time / _nseg);
        const float cost = calculate_obstacle_cost(time, pose);
        if (std::isinf(cost)) {
            return std::numeric_limits<float>::infinity();
        }
        collision_cost += cost;
    }

    return collision_cost;
}

// get total cost
// todo: it is necessary to consider the calculation of safety margin without collision
std::pair<float, float> AP_SLTPlanner::get_total_cost(StateIndex parent_ind, StateIndex cur_ind)
{
    float parent_s = _state.start_s, grandparent_s = _state.start_s;
    float parent_l = _state.start_l, grandparent_l = _state.start_l;

    if (parent_ind.t >= 0) {
        auto &cell = _state_space[parent_ind.t][parent_ind.s][parent_ind.l];
        int grandparent_s_ind = cell.parent_s_ind;
        int grandparent_l_ind = cell.parent_l_ind;
        parent_s = cell.current_s;
        parent_l = get_lateral_offset(parent_s, parent_ind.l);

        if (parent_ind.t >= 1) {
            grandparent_s = _state_space[parent_ind.t - 1][grandparent_s_ind][grandparent_l_ind].current_s;
            grandparent_l = get_lateral_offset(grandparent_s, grandparent_l_ind);
        }
    }

    float cur_s = parent_s + _station[cur_ind.s];
    float cur_l = get_lateral_offset(cur_s, cur_ind.l);

    float ds1 = cur_s - parent_s;
    float dl1 = cur_l - parent_l;

    float ds0 = parent_s - grandparent_s;
    float dl0 = parent_l - grandparent_l;

    float cost_obstacle = get_collision_cost(parent_ind, cur_ind);
    if (std::isinf(cost_obstacle)) {
        return std::make_pair(cur_s, cost_obstacle);
    }

    float cost_lateral =  std::abs(cur_l); // penalize deviate reference line
    float cost_lateral_change = std::abs(dl1) / (ds1 + planning::kMathEpsilon);  // penalize angulary velocity
    float cost_lateral_change_t = std::abs(dl1 - dl0) / _unit_time;
    float cost_longitudinal_velocity = std::abs(ds1 / _unit_time - _desired_speed_input);
    float cost_longitudinal_velocity_change = std::abs((ds1 - ds0) / _unit_time);

    float delta_cost = (
        _lateral_weight * cost_lateral +
        _lateral_change_weight * cost_lateral_change +
        _lateral_vel_change_weight * cost_lateral_change_t +
        _longitudinal_vel_weight * cost_longitudinal_velocity +
        _longitudinal_vel_change_weight * cost_longitudinal_velocity_change);

    return std::make_pair(cur_s, delta_cost + cost_obstacle);
}

// get lateral offset
float AP_SLTPlanner::get_lateral_offset(const float s, const int16_t l_ind)
{
    if (l_ind == NL - 1) {
        return 0.0f;
    }
    auto ref = _reference.evaluate_station(s);
    const float lb = -ref.right_bound + _safe_margin + _vehicle_radius;
    const float ub = ref.left_bound - _safe_margin - _vehicle_radius;
    return lb + (ub - lb) * _lateral[l_ind];
}

// linear polate
std::vector<planning::Vec2d> AP_SLTPlanner::interpolate_linearly(const float parent_s, const int16_t parent_l_ind, const int16_t current_s_ind, const int16_t current_l_ind)
{
    std::vector<planning::Vec2d> result(_nseg);
    float pl = _state.start_l;
    float ps = _state.start_s;
    if (parent_l_ind >= 0) {
        ps = parent_s;
        pl = get_lateral_offset(ps, parent_l_ind);
    }

    const float cur_s = ps + _station[current_s_ind];
    const float cur_l = get_lateral_offset(cur_s, current_l_ind);

    const float s_step = _station[current_s_ind] / _nseg;
    const float l_step = (cur_l - pl) / _nseg;

    for (int16_t i = 0; i < _nseg; i++) {
        result[i].set_x(ps + i * s_step);
        result[i].set_y(pl + i * l_step);
    }

    return result;
}

float AP_SLTPlanner::calculate_static_obstacle_cost(const object &obj)
{
    float obstacle_cost = 0.0f;
    for (auto &obstacle: _static_obstacles) {
        const float collision_distance = (obj.pos - obstacle.pos).length() -  (obj.radius + obstacle.radius);
        if (collision_distance < 0) {
            return std::numeric_limits<float>::infinity();
        } else if (collision_distance > 0 && collision_distance < 2.0f * _safe_margin){
            obstacle_cost += _obstacle_collision_weight * planning::Sigmoid(_safe_margin - collision_distance);
        } else {

        }
    }
    
    if (_road_barrier.empty()) {
        return obstacle_cost;
    }

    const float obj_max_x = obj.pos.x + obj.radius;
    const float obj_min_x = obj.pos.x - obj.radius;
    if (obj_max_x < _road_barrier.front().x() || obj_min_x > _road_barrier.back().x()) {
        return obstacle_cost;
    }

    auto comp = [](float val, const planning::Vec2d &a) {
        return val < a.x();
    };

    // binary search
    auto check_start = std::upper_bound(_road_barrier.begin(), _road_barrier.end(), obj_min_x, comp);
    auto check_end   = std::upper_bound(_road_barrier.begin(), _road_barrier.end(), obj_max_x, comp);

    if (check_start > _road_barrier.begin()) {
        std::advance(check_start, -1);
    }

    for (auto iter = check_start; iter != check_end; iter++) {
        const auto road_point = *iter;
        const float dist = std::hypot(obj.pos.x - road_point.x(), obj.pos.y - road_point.y());
        if (dist <= obj.radius) {
            return std::numeric_limits<float>::infinity();
        } 
    }

    return obstacle_cost;
}


float AP_SLTPlanner::calculate_dynamic_obstacle_cost(const float time, const object &obj)
{
    float obstacle_cost = 0.0f;
    for (auto &obs: _dynamic_obstacles) {
        const auto obs_pos_t = obs.pos + obs.vel * time;
        const float collision_distance = (obj.pos - obs_pos_t).length() -  (obj.radius + obs.radius);
        if (collision_distance < 0) {
            return std::numeric_limits<float>::infinity();
        } else if (collision_distance > 0 && collision_distance < 10 * _safe_margin){
             obstacle_cost += _obstacle_collision_weight * planning::Sigmoid(_safe_margin - collision_distance);
        } else {
            
        }
    }
    return obstacle_cost;
}

float AP_SLTPlanner::calculate_obstacle_cost(const float time, const planning::Pose &pose)
{
    struct object rob;
    rob.pos.x = pose.x();
    rob.pos.y = pose.y();
    rob.vel.zero();
    rob.radius = _vehicle_radius;

    float obstacle_cost = 0;
    // static obstacle cost
    obstacle_cost += calculate_static_obstacle_cost(rob);
    if (std::isinf(obstacle_cost)) {
        return obstacle_cost;
    }
    
    // dynamic obstacle cost 
    obstacle_cost += calculate_dynamic_obstacle_cost(time, rob);

    return obstacle_cost;
}


void AP_SLTPlanner::update_obstacles()
{
    // reset list
    _dynamic_obstacles.clear();
    _static_obstacles.clear();
    
    // update proximity 
    update_proximity_obstacles();
  
    update_fence_obstacles();
}

void AP_SLTPlanner::update_proximity_obstacles()
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    // proximity obstacles 
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector2f vel = item.vel.xy();
        struct object obj;
        obj.pos = item.pos.xy();
        obj.vel = item.vel.xy();
        obj.radius = item.radius;

        if (vel.length_squared() <= sq(0.2f)) {
            _static_obstacles.emplace_back(obj);
        } else {
            _dynamic_obstacles.emplace_back(obj);
        }
    }
  
}

 void AP_SLTPlanner::update_fence_obstacles()
 {
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return;
    }
    // exclusion circles enabled along with polygon fences
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }
    // return immediately if no inclusion nor exclusion circles
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if (num_exclusion_circles == 0) {
        return;
    }

    // iterate through exclusion circles
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {
            struct object obj;
            obj.pos.x = center_pos_cm.x * 0.01f;
            obj.pos.y = center_pos_cm.y * 0.01f;
            obj.vel.zero();
            obj.radius = radius;;
            _static_obstacles.emplace_back(obj);
        }
    }
}