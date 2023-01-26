#include "wifi_settings.h"
#include "WiFi.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

//RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void setup_wifi(){
    Serial.println("Trying to connect to WIFI_SSID with WIFI_PW");
    WiFi.begin(WIFI_SSID, WIFI_PW);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to WiFi");
}

void setup_mqtt() {

}

void send_to_mqtt(int moisture, int voltage) {

}

/// Takes x measurement-point in y seconds. Returns average of x measurments
int measure_soil_moisture() {
  int meas_points = 10;
  int sum_values = 0;
  int meas_value = 0;
  for(int i=0; i < meas_points; i++) {
    meas_value = 2; // TODO read analog value
    Serial.print("Measured value: ");
    Serial.println(meas_value);
    sum_values += meas_value;  // TODO int-overflow?
    // TODO sleep
  }
  return sum_values / meas_points;
}

int get_battery_voltage() {
  // TODO read analog value
}

void setup(){
  Serial.begin(115200);

  print_wakeup_reason();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

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

void loop(){
  //This is not going to be called
}

