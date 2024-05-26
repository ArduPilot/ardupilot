#include <bluefruit.h>

// This code uses the bluefruit libary52 from Adafruit
// Documentation here: https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/bluefruit-nrf52-api
// Should work with any nrf52 Bluetooth Arduino, tested with Seeed XIAO nRF52840 Sense

// Wind vane address, found with mobile app
#define WIND_VANE_ADDRESS 0xDDB7BEB45638

// Define services and charicteristics to moniter
BLEClientService        env_sense(0x181A);  // Environmental Sensing
BLEClientCharacteristic wind_speed(0x2A72); // Apparent Wind Speed
BLEClientCharacteristic wind_dir(0x2A73);   // Apparent Wind Direcetion

BLEClientService        bat_sense(0x180F);  // Battery Sensing
BLEClientCharacteristic bat_level(0x2A19);  // Battery level

ble_gap_addr_t addr;

float speed;
float direction;
uint8_t battery_level;

bool speed_new;
bool direction_new;
bool battery_new;

uint32_t last_connect_attempt;

#define RECONNECT_TIME_MS 1000
#define NMEA_BAUD 57600
#define LOW_BAT_BLINK_MS 200

void setup()
{
  // Disconected
  digitalWrite(LED_RED, LOW); 

  uint64_t address = WIND_VANE_ADDRESS;
  memcpy(addr.addr, &address, BLE_GAP_ADDR_LEN);

  addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;

  // Extra info and debugging on USB, NMEA on serial 1
  Serial.begin(115200);
  Serial1.begin(NMEA_BAUD);

  Serial.println("ArduPilot Bluetooth windvane to NMEA translator");

  Bluefruit.begin(0, 1);
  Bluefruit.setName("ArduPilot Bluetooth");

  // Initialize services client
  env_sense.begin();
  bat_sense.begin();

  // Set notify callbacks
  wind_speed.setNotifyCallback(speed_notify_callback);
  wind_dir.setNotifyCallback(direction_notify_callback);
  bat_level.setNotifyCallback(bat_notify_callback);

  // Initalize Caricterstics
  wind_speed.begin(&env_sense);
  wind_dir.begin(&env_sense);
  bat_level.begin(&bat_sense);

  // Turn of LEDs, we do our own
  Bluefruit.autoConnLed(false);

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);

  // Connect to known address
  Bluefruit.Central.connect(&addr);
  last_connect_attempt = millis();
}

bool low_battery;
uint32_t last_low_battery_blink;
char NMEA_buffer[50];
void loop()
{
  if (!Bluefruit.connected(0)) {
    // Disconected
    digitalWrite(LED_RED, LOW); 
    digitalWrite(LED_GREEN, HIGH); 

    const uint32_t now = millis();
    if (now - last_connect_attempt > RECONNECT_TIME_MS) {
      // try and re-connect
      Bluefruit.Central.connect(&addr);
      last_connect_attempt = now;
    }
    return;
  }

  // Connected, show green LED
  digitalWrite(LED_GREEN, LOW);

  // flash red LED for low battery
  if (low_battery) {
    const uint32_t now = millis();
    if (now - last_low_battery_blink > LOW_BAT_BLINK_MS) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      last_low_battery_blink = now;
    }
  } else {
    digitalWrite(LED_RED, HIGH); 
  }

  if (speed_new && direction_new) {
    speed_new = false;
    direction_new = false;

    Serial.printf("Wind Speed: %0.2f (m/s), Direction: %0.2f (deg), Battery: %u%%\n", speed, direction, battery_level);

    sprintf(NMEA_buffer, "APMWV,%0.2f,R,%0.2f,M,A", direction, speed);
    Serial1.printf("$%s*%02X\n",NMEA_buffer, NMEA_checksum(NMEA_buffer));
  }

  if (battery_new) {
    battery_new = false;

    sprintf(NMEA_buffer, "APXDR,G,%u,%%,BT", battery_level);
    Serial1.printf("$%s*%02X\n",NMEA_buffer, NMEA_checksum(NMEA_buffer));

    if (battery_level < 25) {
      // Windvane enters low power mode at less than 20% battery
      low_battery = true;
    } else if (battery_level > 30) {
      low_battery = false;
    }
  }

}

uint8_t NMEA_checksum(const char* NMEA_string)
{
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < strlen(NMEA_string); i++) {
    checksum ^= NMEA_string[i];
  }
  return checksum;
}

void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");

  Serial.print("Discovering Environmental characteristics ... ");
  if (!env_sense.discover(conn_handle) || !wind_speed.discover() || !wind_dir.discover()) {
    Serial.println("Failed");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Found");
  

  Serial.print("Discovering Battery characteristics ... ");
  if (!bat_sense.discover(conn_handle) || !bat_level.discover()) {
    Serial.println("failed");  
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Found");

  if (wind_speed.enableNotify() && wind_dir.enableNotify() && bat_level.enableNotify()) {
    Serial.println("Ready to receive");
  } else {
    Serial.println("Couldn't enable notify. Increase DEBUG LEVEL for troubleshooting");
  }

}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;

  speed_new = false;
  direction_new = false;
  battery_new = false;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}


void speed_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  if (len == 2) {
    uint16_t value;
    memcpy(&value, data, 2);
    speed = value * 0.01;
    speed_new = true;
  }
}

void direction_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  if (len == 2) {
    uint16_t value;
    memcpy(&value, data, 2);
    direction = value * 0.01;
    direction_new = true;
  }
}

void bat_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  if (len == 1) {
    battery_level = data[0];
    battery_new = true;
  }
}
