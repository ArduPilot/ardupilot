#pragma once

#include <AP_Math/AP_Math.h>


class Hysteresis
{
public:
	explicit Hysteresis(bool init_state) :
		_state(init_state),
		_requested_state(init_state)
	{}
	Hysteresis() = delete; // no default constructor

	~Hysteresis() = default;

	bool get_state() const { return _state; }

	void set_hysteresis_time_from(const bool from_state, const uint64_t new_hysteresis_time_us);

	void set_state_and_update(const bool new_state, const uint64_t &now_us);

	void update(const uint64_t &now_us);

private:

	uint64_t _last_time_to_change_state{0};

	uint64_t _time_from_true_us{0};
	uint64_t _time_from_false_us{0};

	bool _state;
	bool _requested_state;
};
