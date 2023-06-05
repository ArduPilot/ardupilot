#pragma once

#include <string>
#include <vector>

namespace planning {

class SpeedPoint {
public:
    SpeedPoint():_s(0),_t(0),_v(0),_a(0),_da(0){}
    SpeedPoint(const float s, const float t, 
               const float v, const float a, 
               const float da) : _s(s),_t(t),_v(v),_a(a),_da(da){}

    float s() const { return _s; }
    float t() const { return _t; }
    float v() const { return _v; }
    float a() const { return _a; }
    float da() const { return _da; }
    
    void set_s(const float s) { _s = s; }
    void set_t(const float t) { _t = t; }
    void set_v(const float v) { _v = v; }
    void set_a(const float a) { _a = a; }
    void set_da(const float da) { _da = da; }

    static SpeedPoint ToSpeedPoint(const float s, const float t,
                                   const float v = 0, const float a = 0,
                                   const float da = 0)
    {
      return SpeedPoint(s, t, v, a, da);
    }

 private:
    float _s; // m
    float _t; // seconds
    float _v; // m/s
    float _a; // m/s^2
    float _da; // m/s^3
};



class SpeedData {
 public:
  SpeedData() = default;

  explicit SpeedData(std::vector<SpeedPoint> speed_points);

  virtual ~SpeedData() = default;

  const std::vector<SpeedPoint>& speed_vector() const;

  void set_speed_vector(std::vector<SpeedPoint> speed_points);

  void AppendSpeedPoint(const float s, const float time, const float v,
                        const float a, const float da);

  bool EvaluateByTime(const float time,
                      SpeedPoint* const speed_point) const;

  float TotalTime() const;

  bool Empty() const { return speed_vector_.empty(); }

  void Clear();

 private:
  std::vector<SpeedPoint> speed_vector_;
};

}  // namespace planning

