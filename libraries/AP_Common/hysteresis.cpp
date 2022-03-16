#include "hysteresis.h"


void Hysteresis::set_hysteresis_time_from(const bool from_state, const uint64_t new_hysteresis_time_us)
{
	if (from_state) {
		_time_from_true_us = new_hysteresis_time_us;

	} else {
		_time_from_false_us = new_hysteresis_time_us;
	}
}

void Hysteresis::set_state_and_update(const bool new_state, const uint64_t &now_us)
{
	if (new_state != _state) {
		if (new_state != _requested_state) {
			_requested_state = new_state;
			_last_time_to_change_state = now_us;
		}

	} else {
		_requested_state = _state;
	}

	update(now_us);
}

void Hysteresis::update(const uint64_t &now_us)
{
	if (_requested_state != _state) {

		const uint64_t elapsed = now_us - _last_time_to_change_state;

		if (_state && !_requested_state) {
			// true -> false
			if (elapsed >= _time_from_true_us) {
				_state = false;
			}

		} else if (!_state && _requested_state) {
			// false -> true
			if (elapsed >= _time_from_false_us) {
				_state = true;
			}
		}
	}
}


