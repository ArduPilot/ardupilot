#include <AP_gtest.h>
#include <AP_IrisOrca/AP_IrisOrca.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


/**
 * Test parsing the response from a write motor command stream. Truth
 * data for the test taken from the IRIS Orca Series Modbus RTU User Guide
 * section "Sleep Stream Command Frame":
 * https://irisdynamics.com/hubfs/Website/Downloads/Orca/Approved/UG210912_Orca_Series_Modbus_RTU_User_Guide.pdf
 * 
 * Note that this document has an error in the "Force Control Stream Command Frame" example listing only
 * 18 bytes for an expected 19 byte response.
*/
TEST(AP_IRIS_ORCA, parse_motor_command_rsp)
{
  uint8_t expected_response[orca::MOTOR_COMMAND_STREAM_MSG_RSP_LEN] = {
    0x01, // device id
    0x64, // function code 
    0x00, 0x03, 0x89, 0x65, // position: 231781 um = 231.781mm
    0x00, 0x00, 0x06, 0xBE, // force: 1726
    0x00, 0x00, // Power: (0 ???)
    0x0F, // temp: 15C
    0x0F, 0x01, // voltage: 3841 mV
    0x00, 0x00, // no errors
    0x88, 0xC2, //
  };

  orca::ActuatorState actuator_state;

  auto ret = orca::parse_motor_command_stream(
      expected_response, orca::MOTOR_COMMAND_STREAM_MSG_RSP_LEN,
      actuator_state);

  ASSERT_TRUE(ret);
  ASSERT_EQ(actuator_state.shaft_position, uint32_t(231781));
  ASSERT_EQ(actuator_state.force_realized, uint32_t(1726));
  ASSERT_EQ(actuator_state.power_consumed, uint16_t(0));
  ASSERT_EQ(actuator_state.temperature, uint8_t(15));
  ASSERT_EQ(actuator_state.voltage, uint16_t(3841));
  ASSERT_EQ(actuator_state.errors, uint16_t(0));
  // CRC tested separately in its own test case
}

/**
 * Test the functionality to add CRC to a message. Data
 * from orca user guide write register example frame.
 */
TEST(AP_IRIS_ORCA, crc_correct) {
  auto expected_crc_lo = 0xF9;
  auto expected_crc_hi = 0xF1;

  uint8_t rsp[orca::WRITE_REG_MSG_RSP_LEN] = {
      0x01, 0x06, 0x00, 0x8B, 0x00,
      0x3C, 0x00, 0x00, /* last two bytes
                                 crc*/
  };

  // Add the CRC to the response
  orca::add_crc_modbus(rsp, orca::WRITE_REG_MSG_RSP_LEN - orca::CRC_LEN);
  ASSERT_EQ(rsp[orca::WRITE_REG_MSG_RSP_LEN - 2], expected_crc_lo);
  ASSERT_EQ(rsp[orca::WRITE_REG_MSG_RSP_LEN - 1], expected_crc_hi);
}

AP_GTEST_MAIN()