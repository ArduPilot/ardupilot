#include <AP_SmartAudio/AP_SmartAudio.h>
#include <AP_HAL/AP_HAL.h>


class SmartAudioTest{
  public:



    /**
 * Constructor
 */
SmartAudioTest()
{

}

  void monkey_testing();
  bool test_set_mode_req(uint8_t sm_mode);
  bool test_set_mode_res(uint8_t sm_mode);
  bool test_get_settings_req();
  bool test_get_settings_res(u_int8_t version);
  bool test_set_freq_req(uint16_t const freq,bool ispitMode);
  bool test_set_freq_res(uint16_t const freq,bool ispitMode);
  bool test_set_chan_req(uint8_t chan_idx);
  bool test_set_chan_res(uint8_t chan_idx);
  bool test_get_pit_mode_freq_req();
  bool test_set_power_req(uint8_t sm_version,uint8_t power);
  bool test_set_power_res(uint8_t sm_version,uint8_t power);
  void get_and_update_version_current_state(u_int8_t version);
  bool assert_request_last_inserted(uint8_t *expectedRequest,uint8_t const size);
};