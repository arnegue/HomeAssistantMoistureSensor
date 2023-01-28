#include "wifi_settings.h"
#include "WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define uS_TO_S_FACTOR 1000000       /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP_S 5            /* Time ESP32 will go to sleep (in seconds) */
#define SENSOR_POLL_SLEEP_TIME_MS 50 /* Time to sleep when measuring values */
#define MOISTURE_SENSOR_POWER_UP_TIME_MS 250

#define BATTERY_ANALOG_PIN 34
#define MOISTURE_SENSOR_ANALOG_PIN 35
#define MOISTURE_SENSOR_POWER_PIN 32

#define MAX_MOISTURE_RAW 1000  // Measure the raw value in oversaturated soil and put it here
#define MIN_MOISTURE_RAW 2400    // Measure the raw value in dry soil and put it here

#define MAX_VOLTAGE_RAW 0xFFF // DAC is 12bit resolution. So that's the maximum for measuring 3.3V
#define MQTT_TOPIC "homeassistant/sensor/MoistureSensor"
#define DEVICE_ID "1"
#define COMMON_STATE_TOPIC MQTT_TOPIC DEVICE_ID "/state"

const char broker[] = "192.168.178.28";
const int mqtt_port = 1883;

WiFiClient client;
Adafruit_MQTT_Client mqtt_client(&client, broker, mqtt_port);

const char moisture_config [] = "{\"device_class\": \"moisture\", \"name\": \"Moisture\", \"state_topic\": \"" COMMON_STATE_TOPIC "\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.moisture}}\" }";
const char battery_config []  = "{\"device_class\": \"battery\",  \"name\": \"Battery\",  \"state_topic\": \"" COMMON_STATE_TOPIC "\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.battery}}\" }";

// Configuration for homeassitant: https://www.home-assistant.io/integrations/mqtt/
Adafruit_MQTT_Publish mqtt_moisture_config = Adafruit_MQTT_Publish(&mqtt_client, MQTT_TOPIC DEVICE_ID "moisture/config"); // no slash infront of sensor
Adafruit_MQTT_Publish mqtt_battery_config  = Adafruit_MQTT_Publish(&mqtt_client, MQTT_TOPIC DEVICE_ID "battery/config");  // no slash infront of sensor
Adafruit_MQTT_Publish mqtt_common_state    = Adafruit_MQTT_Publish(&mqtt_client, COMMON_STATE_TOPIC);

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
  int8_t ret;
  while ((ret = mqtt_client.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt_client.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 2.5 seconds...");
    mqtt_client.disconnect();
    delay(2500);  // wait 2.5 seconds
  }
  Serial.println("MQTT Connected!");
}


void send_to_mqtt(int moisture, int battery) {  
  int ret = mqtt_battery_config.publish(battery_config);
  if (!ret) {
      Serial.println("Publishing battery config failed");
  } else {
      Serial.println("Publishing battery config went well!");
  }

  delay(50);  // Somehow it's needed, else battery will be ignored. Bug in Adafruit library?

  ret = mqtt_moisture_config.publish(moisture_config);
  if (!ret) {
      Serial.println("Publishing moisture config failed");
  } else {
      Serial.println("Publishing moisture config went well!");
  }
  
  delay(50);  // Somehow it's needed, else battery will be ignored. Bug in Adafruit library?

  // Finally publish sensor values
  char buf[50];
  String message = "{\"moisture\": " + String(moisture) + ", \"battery\":" + String(battery) + "}";
  //String message = "{\"moisture\": 10.3, \"battery\": 9.1}";
  Serial.println(message);
  message.toCharArray(buf, message.length() + 1);
  ret = mqtt_common_state.publish(buf);
  if (!ret) {
      Serial.println("Publishing payload failed");
  } else {
      Serial.println("Publishing payload went well!");
  }
/*
  int ret = mqtt_moisture.publish(moisture);
  if (!ret) {
    Serial.println("Publishing moisture failed");
  } else {
    Serial.println("Publishing moisture went well!");
  }
  delay(50);  // Somehow it's needed, else battery will be ignored. Bug in Adafruit library?
  ret = mqtt_battery.publish(battery);
  if (!ret) {
    Serial.println("Publishing voltage failed");
  } else {
    Serial.println("Publishing voltage went well!");
  }*/
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

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_S * uS_TO_S_FACTOR);
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
