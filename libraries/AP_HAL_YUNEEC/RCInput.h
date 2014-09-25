
#ifndef __AP_HAL_YUNEEC_RCINPUT_H__
#define __AP_HAL_YUNEEC_RCINPUT_H__

#include <AP_HAL_YUNEEC.h>

#define YUNEEC_RC_INPUT_NUM_CHANNELS 8
#define YUNEEC_RC_INPUT_MIN_CHANNELS 5     // for ppm sum we allow less than 8 channels to make up a valid packet

typedef void (*voidFuncPtr)(void);

//
// This class is for PPM receiver
//
class YUNEEC::YUNEECRCInputPPM : public AP_HAL::RCInput {
public:
    void init(void* machtnichts);
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    /* private callback for input capture ISR */
    static void _timer_capt_cb(void);
    static void _attachInterrupt(voidFuncPtr callback);
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[YUNEEC_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _valid_channels;
    static volatile bool  _new_input;

    /* override state */
    uint16_t _override[YUNEEC_RC_INPUT_NUM_CHANNELS];
};

//
// This class is for DSM receiver
//
#define DSM_FRAME_SIZE			16		/* DSM frame size in bytes*/
#define DSM_FRAME_CHANNELS		7		/* Max supported DSM channels*/
#define DSM_RC_INPUT_CHANNELS	18		/* Max DSM input channels */

// DSM falling edges counts for configuration
#define DSM_CONFIG_INT_DSM2_22MS	3
#define DSM_CONFIG_EXT_DSM2_22MS	4
#define DSM_CONFIG_INT_DSM2_11MS	5
#define DSM_CONFIG_EXT_DSM2_11MS	6
#define DSM_CONFIG_INT_DSMx_22MS	7
#define DSM_CONFIG_EXT_DSMx_22MS	8
#define DSM_CONFIG_INT_DSMx_11MS	9
#define DSM_CONFIG_EXT_DSMx_11MS	10

// DSM Internal Remote system assignments
#define DSM2_1024_22MS	0x01
#define DSM2_2048_11MS	0x12
#define DSMx_2048_22MS	0xa2
#define DSMx_2048_11MS	0xb2

#define DSM_UART_PORT				USART2_PORT
#define DSM_UART_RX_BIT				USART2_RX_BIT
#define DSM_UART_RX_PINSOURSE		USART2_RX_PINSOURCE

class YUNEEC::YUNEECRCInputDSM : public AP_HAL::RCInput {
public:
    void init(void* machtnichts);
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    static volatile uint8_t _dsm_frame[DSM_FRAME_SIZE];
    static uint16_t _dsm_data_mask;
    static uint8_t _dsm_channel_shift;
    static volatile uint32_t _dsm_last_rx_time;
    static volatile uint8_t _dsm_partial_frame_count;
    static uint8_t _pulses;

    /* which uart is used as dsm input port */
    static AP_HAL::UARTDriver* _dsm_uart;

    /* private variables to communicate with input capture isr */
    static volatile uint16_t _periods[DSM_RC_INPUT_CHANNELS];
    static volatile uint8_t  _valid_channels;
    static volatile bool  _new_input;

    /* override state */
    uint16_t _override[DSM_RC_INPUT_CHANNELS];

    /* used by _dsm_input() */
    static volatile uint32_t _last_time;
//    static void _timer12_config(void);
//    static void _attachInterrupt(voidFuncPtr callback);
    static void _dsm_init(AP_HAL::UARTDriver* uartX);
    static void _dsm_bind(void);
    static bool _dsm_check_binded(void);
    static void _dsm_output_pulses(void);
    void _dsm_input(void);
};

#endif // __AP_HAL_YUNEEC_RCINPUT_H__
