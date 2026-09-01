/*
  Minimal Zenoh pub/sub example

  Publishes a heartbeat *string* every second on "ardupilot/output"
  and prints any *string* message received on "ardupilot/receive".
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Zenoh/AP_Zenoh_config.h>

#if AP_ZENOH_ENABLED

#include <AP_Zenoh/AP_Zenoh.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static const AP_Zenoh_ConfigItem zenoh_config[] = {
    { Z_CONFIG_MODE_KEY, "client" },
};

static AP_Zenoh zenoh(zenoh_config, ARRAY_SIZE(zenoh_config));

class ZenohTest {
public:
    void on_receive(const AP_Zenoh_Payload &payload)
    {
        char key_buf[64];
        payload.key_expression(key_buf, sizeof(key_buf));

        char val_buf[128];
        memcpy(val_buf, payload.data(), payload.len());
        val_buf[payload.len()] = '\0';

        hal.console->printf("%s: received on '%s': '%s'\n",
                            AP_Zenoh::msg_prefix, key_buf, val_buf);
    }

    AP_Zenoh_Publisher pub;
    AP_Zenoh_Subscriber sub;
};

static ZenohTest test;

void setup(void)
{
    hal.console->printf("AP_Zenoh_test starting\n");
    hal.scheduler->delay(1000);

    if (!zenoh.init()) {
        hal.console->printf("AP_Zenoh_test: init failed\n");
        return;
    }

    zenoh.declare_subscriber("ardupilot/receive", test.sub,
                             FUNCTOR_BIND(&test, &ZenohTest::on_receive, void, const AP_Zenoh_Payload &));
    zenoh.declare_publisher("ardupilot/output", test.pub);
}

void loop(void)
{
    hal.scheduler->delay(1000);

    if (!test.pub.valid()) {
        return;
    }

    test.pub.publish("ping");
}

AP_HAL_MAIN();

#else

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

void setup() {}
void loop() {}

AP_HAL_MAIN();

#endif // AP_ZENOH_ENABLED
