#include "wifi_settings.h"
#include "WiFi.h"
#include <ArduinoHA.h>

#define TIME_TO_SLEEP_S 5                    // Time ESP32 will go to sleep (in seconds)
#define SENSOR_POLL_SLEEP_TIME_MS 50         // Time to sleep when measuring values
#define MOISTURE_SENSOR_POWER_UP_TIME_MS 250 // Time the moisture sensor needs to measure valid values 

#define BATTERY_ANALOG_PIN 34
#define MOISTURE_SENSOR_ANALOG_PIN 35
#define MOISTURE_SENSOR_POWER_PIN 32

#define MAX_MOISTURE_RAW 1000                // Measure the raw value in oversaturated soil and put it here
#define MIN_MOISTURE_RAW 2400                // Measure the raw value in dry soil and put it here
#define MAX_VOLTAGE_RAW 0xFFF                // DAC is 12bit resolution. So that's the maximum for measuring 3.3V

#define SENSOR_NAME "MoistureSensor"
#define SENSOR_ID "1"

#define MQTT_BROKER_ADDRESS IPAddress(192,168,178,28)
#define MQTT_BROKER_PORT 1883

WiFiClient client;
HADevice device(SENSOR_NAME SENSOR_ID);
HAMqtt mqtt(client, device);

HASensorNumber mqtt_moisture_sensor("moisture", HASensorNumber::PrecisionP0);
HASensorNumber mqtt_battery_sensor("battery", HASensorNumber::PrecisionP0);

// TODO Possible optimization: Measure battery and moisture at once
// !!! IMPORTANT !!! Manually change "MAXBUFFERSIZE" in "Adafruit_MQTT_Library\Adafruit_MQTT.h" or your mqtt-package will be stripped
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

void setup_mqtt() {
    mqtt.begin(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
    mqtt.loop();
}


void send_to_mqtt(int moisture, int battery) {
  
  int ret = mqtt_moisture_sensor.setValue(moisture);
  if (!ret) {
    Serial.println("Publishing moisture failed");
  } else {
    Serial.println("Publishing moisture went well!");
  }
  delay(50);  // Somehow it's needed, else battery will be ignored. Bug in Mosquitto? I tried several client-libraries...
  ret = mqtt_battery_sensor.setValue(battery);
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

int measure_soil_moisture() {
  int value;
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, HIGH);
  delay(MOISTURE_SENSOR_POWER_UP_TIME_MS);
  value = read_analog_values(MOISTURE_SENSOR_ANALOG_PIN);
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, LOW);
  
  Serial.print("Moisture_raw:");
  Serial.print(value);

  Serial.print(",Moisture_p:");
  value = get_percentage(value, MAX_MOISTURE_RAW, MIN_MOISTURE_RAW);
  Serial.println(value);

  return value;
}

int get_battery_voltage() {
  int value = read_analog_values(BATTERY_ANALOG_PIN);

  Serial.print("Voltage_raw:");
  Serial.print(value);

  Serial.print(",Voltage_p:");
  value = get_percentage(value, MAX_VOLTAGE_RAW, 0);
  Serial.println(value);

  return value;
}

void setup() {
  Serial.begin(115200);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_S * 1000000);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP_S) + " Seconds");

  setup_pins();
  setup_wifi();

  int moisture = measure_soil_moisture();
  int voltage = get_battery_voltage();
  wait_for_wifi();
  setup_mqtt();
  send_to_mqtt(moisture, voltage);

  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {
}
