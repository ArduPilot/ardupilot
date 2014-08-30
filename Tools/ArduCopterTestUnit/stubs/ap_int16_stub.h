#ifndef AP_INT16_STUB_H_
#define AP_INT16_STUB_H_

class AP_Int16 {
	int16_t _value = 0;
public:
	int16_t get(void) {return _value;}
	void set(int16_t) {_value = 16;}
	int16_t operator &(const int v) {return _value & v;}
};//AP_Int16

#endif /* AP_INT16_STUB_H_ */
