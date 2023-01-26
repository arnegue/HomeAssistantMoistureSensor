#include "wifi_settings.h"
#include "WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

const char broker[] = "192.168.178.28";
const char topic[] = "real_unique_topic";
const char topic2[] = "real_unique_topic_2";
const int mqtt_port = 1883;
WiFiClient client;
Adafruit_MQTT_Client mqtt_client(&client, broker, mqtt_port);

Adafruit_MQTT_Publish mqtt_battery  = Adafruit_MQTT_Publish(&mqtt_client, "MoistureSensor/battery");
Adafruit_MQTT_Publish mqtt_moisture = Adafruit_MQTT_Publish(&mqtt_client, "MoistureSensor/moisture");

// TODO MQTT-Setup
// TODO MQTT-Publishing
// TODO Possible optimization: Measure battery and moisture at once
// TODO Possible optimization: Put wifi-status just before publishing to mqtt instead of using time for measuring values
// TODO calibration? Or work later on raw data? https://makersportal.com/blog/2020/5/26/capacitive-soil-moisture-calibration-with-arduino

#define uS_TO_S_FACTOR 1000000       /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP_S 5            /* Time ESP32 will go to sleep (in seconds) */
#define SENSOR_POLL_SLEEP_TIME_MS 50 /* Time to sleep when measuring values */
#define MOISTURE_SENSOR_POWER_UP_TIME_MS 250

#define BATTERY_ANALOG_PIN 34
#define MOISTURE_SENSOR_ANALOG_PIN 35
#define MOISTURE_SENSOR_POWER_PIN 32

void setup_pins() {
  pinMode(BATTERY_ANALOG_PIN, INPUT);
  pinMode(MOISTURE_SENSOR_ANALOG_PIN, INPUT);
  pinMode(MOISTURE_SENSOR_POWER_PIN, OUTPUT);
}

void setup_wifi() {
  Serial.println("Trying to connect to " WIFI_SSID " with pw " WIFI_PW);
  WiFi.begin(WIFI_SSID, WIFI_PW);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
}

void setup_mqtt() {
  int8_t ret;
  uint8_t retries = 3;
  while ((ret = mqtt_client.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt_client.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt_client.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1)
        ;
    }
  }
  Serial.println("MQTT Connected!");
}


void send_to_mqtt(int moisture, int battery) {
  int battery_send = mqtt_battery.publish(battery);
  int moisture_send = mqtt_moisture.publish(moisture);
  if (!moisture_send) {
    Serial.println("Publishing moisture failed");
  } else {
    Serial.println("Publishing moisture went well!");
  }
  if (!battery_send) {
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

int measure_soil_moisture() {
  int value;
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, HIGH);
  delay(MOISTURE_SENSOR_POWER_UP_TIME_MS);
  value = read_analog_values(MOISTURE_SENSOR_ANALOG_PIN);
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, LOW);
  Serial.print("Moisture:");
  Serial.println(value);
  return value;
}

int get_battery_voltage() {
  return read_analog_values(BATTERY_ANALOG_PIN);
}

void setup() {
  Serial.begin(115200);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_S * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP_S) + " Seconds");

  setup_pins();
  setup_wifi();
  setup_mqtt();

  int moisture = measure_soil_moisture();
  int voltage = get_battery_voltage();
  send_to_mqtt(moisture, voltage);

  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {
}
