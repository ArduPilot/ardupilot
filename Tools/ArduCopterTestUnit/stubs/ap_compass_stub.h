#ifndef AP_COMPASS_STUB_H_
#define AP_COMPASS_STUB_H_

class AP_COMPASS
{
	Vector3f _field;
	Vector3f _offsets;
public:
	bool healthy (uint8_t toto) {return true;}
	const Vector3f &get_field(void) const {return _field;}
	const Vector3f &get_field(uint8_t instance) const {return _field;}
	uint8_t get_count(void) const { return 1; }
	const Vector3f &get_offsets(uint8_t i) const { return _offsets; }
	const Vector3f &get_offsets() const { return _offsets; }
	float get_declination() const {return 3.14;}
	void set_offsets(const Vector3f &offsets) {};

};//AP_COMPASS

#endif /* AP_COMPASS_STUB_H_ */
