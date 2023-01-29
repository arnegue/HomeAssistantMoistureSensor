#include "wifi_settings.h"
#include "WiFi.h"
#include <EEPROM.h>
#include <ArduinoHA.h>

#define EEPROM_SIZE 4                        // 4 bytes to safe: 2*uint16

#define TIME_TO_SLEEP_S 5                    // Time ESP32 will go to sleep (in seconds)
#define SENSOR_POLL_SLEEP_TIME_MS 50         // Time to sleep when measuring values
#define MOISTURE_SENSOR_POWER_UP_TIME_MS 250 // Time the moisture sensor needs to measure valid values 
#define KEEP_ALIVE_TIME_MS 5000              // Time after publishing the sensor is kept alive for calibration

#define BATTERY_ANALOG_PIN 34
#define MOISTURE_SENSOR_ANALOG_PIN 35
#define MOISTURE_SENSOR_POWER_PIN 32

#define MAX_DAC_RAW 0xFFF                    // DAC is 12bit resolution.

#define SENSOR_NAME "MoistureSensor"
#define SENSOR_ID "1"

#define MQTT_BROKER_ADDRESS IPAddress(192,168,178,28)
#define MQTT_BROKER_PORT 1883

WiFiClient client;
HADevice device(SENSOR_NAME SENSOR_ID);
HAMqtt mqtt(client, device);

HASensorNumber mqtt_moisture_sensor("moisture", HASensorNumber::PrecisionP0);
HASensorNumber mqtt_moisture_raw_sensor("moisture_raw", HASensorNumber::PrecisionP0);
HANumber mqtt_moisture_max_raw("moisture_max_raw", HASensorNumber::PrecisionP0);
HANumber mqtt_moisture_min_raw("moisture_min_raw", HASensorNumber::PrecisionP0);

// Following both values get stored in flash of ESP32, so they will still be there after powercycle (which wouldn't work with RTC_DATA_ATTR)
#define DEFAULT_MAX_MOISTURE_RAW 1000
#define DEFAULT_MIN_MOISTURE_RAW 2400
#define MAX_EEPROM_ADDRESS 0
#define MIN_EEPROM_ADDRESS 1
uint16_t max_moisture_raw = DEFAULT_MAX_MOISTURE_RAW;            // Measure the raw value in oversaturated soil and put it here
uint16_t min_moisture_raw = DEFAULT_MIN_MOISTURE_RAW;            // Measure the raw value in dry soil and put it here

HASensorNumber mqtt_battery_sensor("battery", HASensorNumber::PrecisionP0);

void setup_pins() {
  pinMode(BATTERY_ANALOG_PIN, INPUT);
  pinMode(MOISTURE_SENSOR_ANALOG_PIN, INPUT);
  pinMode(MOISTURE_SENSOR_POWER_PIN, OUTPUT);
}

void setup_wifi() {
  Serial.println("Trying to connect to " WIFI_SSID " with pw " WIFI_PW);
  WiFi.begin(WIFI_SSID, WIFI_PW);
}

void wait_for_wifi() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
}

void on_set_raw_min_max_callback(HANumeric number, HANumber* sender)
{
  int eeprom_addr = -1;
  int eeprom_value = 0;
    Serial.print("Setting number ");
    Serial.println(sender->getName());
    if (sender == &mqtt_moisture_min_raw) {
      Serial.println("Setting min value");
      eeprom_addr = MIN_EEPROM_ADDRESS;
    } else if (sender == &mqtt_moisture_max_raw) {
      Serial.println("Setting max value");
      eeprom_addr = MAX_EEPROM_ADDRESS;
    } else {
      Serial.println("I don't know who called!");
      return;
    }
    
    Serial.print("Setting value ");
    if (!number.isSet()) {
      if (eeprom_addr == MIN_EEPROM_ADDRESS) {
        eeprom_value = DEFAULT_MIN_MOISTURE_RAW;
      } else {
        eeprom_value = DEFAULT_MAX_MOISTURE_RAW;
      }
    } else {
      eeprom_value = number.toInt16();
    }
    Serial.print(eeprom_value);

    EEPROM.write(eeprom_addr, eeprom_value);
    EEPROM.commit();
    sender->setState(number); // report the selected option back to the HA panel
}

void setup_mqtt() {  
    device.setName("ESP32 Moisture Sensor");
    device.setSoftwareVersion("1.0.0");
    device.setManufacturer("Frost-Freak");
    device.setModel("MoistureSensorV1");

    mqtt_moisture_sensor.setName("Soil moisture");
    mqtt_moisture_sensor.setUnitOfMeasurement("%");
    mqtt_moisture_sensor.setIcon("mdi:water-percent");
    mqtt_moisture_sensor.setDeviceClass("moisture");

    mqtt_moisture_raw_sensor.setName("raw moisture value");
    mqtt_moisture_sensor.setIcon("mdi:water");

    mqtt_moisture_max_raw.setName("Maximum raw moisture value");
    mqtt_moisture_max_raw.setIcon("mdi:water-plus");
    mqtt_moisture_max_raw.setMax(MAX_DAC_RAW);
    mqtt_moisture_max_raw.onCommand(on_set_raw_min_max_callback);

    mqtt_moisture_min_raw.setName("Minimum raw moisture value");
    mqtt_moisture_min_raw.setIcon("mdi:water-minus");
    mqtt_moisture_min_raw.setMax(MAX_DAC_RAW);
    mqtt_moisture_min_raw.onCommand(on_set_raw_min_max_callback);

    mqtt_battery_sensor.setName("Battery level");
    mqtt_battery_sensor.setUnitOfMeasurement("%");
    mqtt_battery_sensor.setIcon("mdi:battery");
    mqtt_battery_sensor.setDeviceClass("battery");

    mqtt.begin(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
    mqtt.loop();

    // Since these states get published, put them after loop
    mqtt_moisture_max_raw.setState(max_moisture_raw);
    mqtt_moisture_min_raw.setState(min_moisture_raw);
}


void send_to_mqtt(int moisture, int moisture_raw, int battery) {  
  int ret = mqtt_moisture_sensor.setValue(moisture, true);
  if (!ret) {
    Serial.println("Publishing moisture failed");
  } else {
    Serial.println("Publishing moisture went well!");
  }
  
  delay(50);  // Somehow it's needed, else battery will be ignored. Bug in Mosquitto? I tried several client-libraries...

  ret = mqtt_moisture_raw_sensor.setValue(moisture_raw, true);
  if (!ret) {
    Serial.println("Publishing raw moisture failed");
  } else {
    Serial.println("Publishing raw moisture went well!");
  }

  delay(50);

  ret = mqtt_battery_sensor.setValue(battery, true);
  if (!ret) {
    Serial.println("Publishing voltage failed");
  } else {
    Serial.println("Publishing voltage went well!");
  }
}

/// Takes x measurement-point in y seconds. Returns average of x measurments
int read_analog_values(int pin) {
  int meas_points = 10;
  int sum_values = 0;
  int meas_value = 0;
  for (int i = 0; i < meas_points; i++) {
    meas_value = analogRead(pin);
    //Serial.print("Measured value: ");
    //Serial.println(meas_value);
    sum_values += meas_value;  // TODO int-overflow?
    delay(SENSOR_POLL_SLEEP_TIME_MS);
  }
  return sum_values / meas_points;
}

int get_percentage(int value, int max_value, int min_value) {
  int temp_max = max_value;
  int temp_min = min_value;
  if (temp_min > temp_max) {
      temp_max = min_value;
      temp_min = max_value;
  }
  if (value >= temp_max) {
    return 100;
  } else if (value <= temp_min) {
    return 0;
  } else {
    return ((value - temp_max) * 100) / (temp_min - temp_max);  // TODO maybe use a better way without signed int
  }
}

void measure_soil_moisture(int* perc, int* raw) {
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, HIGH);
  delay(MOISTURE_SENSOR_POWER_UP_TIME_MS);
  *raw = read_analog_values(MOISTURE_SENSOR_ANALOG_PIN);
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, LOW);
  
  Serial.print("Moisture_raw:");
  Serial.print(*raw);

  Serial.print(",Moisture_p:");
  *perc = get_percentage(*raw, max_moisture_raw, min_moisture_raw);
  Serial.println(*perc);
}

void get_battery_voltage(int* perc, int* raw) {
  *raw = read_analog_values(BATTERY_ANALOG_PIN);

  Serial.print("Voltage_raw:");
  Serial.print(*raw);

  Serial.print(",Voltage_p:");
  *perc = get_percentage(*raw, MAX_DAC_RAW, 0);
  Serial.println(*perc);
}

void setup() {
  int moisture_perc, moisture_raw, battery_perc, battery_raw;

  Serial.begin(115200);
  
  EEPROM.begin(EEPROM_SIZE);
  max_moisture_raw = EEPROM.readUShort(MAX_EEPROM_ADDRESS);
  if (max_moisture_raw == 0xFFFF) {
    max_moisture_raw = DEFAULT_MAX_MOISTURE_RAW;
  }
  min_moisture_raw = EEPROM.readUShort(MIN_EEPROM_ADDRESS);
  if (min_moisture_raw == 0xFFFF) {
    min_moisture_raw = DEFAULT_MIN_MOISTURE_RAW;
  }
  // TODO write them into eeprom

  // Enable deep sleep
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_S * 1000000);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP_S) + " Seconds");

  // Setup wifi and pins
  setup_pins();
  setup_wifi();

  // Measure values and send them via mqtt
  measure_soil_moisture(&moisture_perc, &moisture_raw);
  get_battery_voltage(&battery_perc, &battery_raw);
  wait_for_wifi();
  setup_mqtt();
  send_to_mqtt(moisture_perc, moisture_raw, battery_perc);

  // Keep alive for x ms
  Serial.print("Keep alive for ");
  Serial.print(KEEP_ALIVE_TIME_MS);
  Serial.println(" ms");
  delay(KEEP_ALIVE_TIME_MS);
  
  // Go back to sleep
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
}
