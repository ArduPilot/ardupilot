/*
  RC input emulation
  Code by Andrew Tridgell November 2011
 */

#ifndef _SITL_RC_H
#define _SITL_RC_H

struct RC_ICR4 {
	uint16_t channels[9]; // 9th channel is sync
	uint32_t value;
	uint8_t idx;

	RC_ICR4() {
		// constructor
		channels[0] = channels[1] = channels[3] = 1500;
		channels[4] = channels[7] = 1800;
		channels[2] = channels[5] = channels[6] = 1000;
		channels[8] = 4500; // sync
		idx = 0;
	}

	/*
	  read from ICR4 fetches next channel
	 */
	operator int() {
		value += channels[idx++]*2;
		if (idx == 9) {
			idx = 0;
		}
		value = value % 40000;
		return (uint16_t)value;
	}


	/*
	   ignore rate assignment for now (needed for apm2
	   emulation)
	*/
	RC_ICR4& operator=(uint16_t rate) {
		return *this;
	}

	/*
	  interface to set a channel value from SITL
	 */
	void set(uint8_t i, uint16_t v) {
		channels[i] = v;
	}
};

#endif
