#ifndef ModeFilter_h
#define ModeFilter_h

#define MOD_FILTER_SIZE 6

#include <inttypes.h>

class ModeFilter
{
  private:
  public:
	ModeFilter();

	int 		get_filtered_with_sample(int _sample);
	int16_t 	_samples[MOD_FILTER_SIZE];

  private:
	void 		isort();
	int16_t		mode();
	int8_t		_sample_index;
};

#endif



