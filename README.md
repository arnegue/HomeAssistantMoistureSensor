# HomeAssistantMoistureSensor
MQTT MoistureSensor HomeAssistant with ESP32

Simple MQTT-Sensor for capacitive soil moisture on a ESP32 (Wrover) using [Arduino Home Assistant integration by dawidchyrzynski](https://github.com/dawidchyrzynski/arduino-home-assistant)

# Setup
## Settings variables and defines
### MoistureMQTT.ino
* `SENSOR_ID` If you want to integrate multiple sensors, ensure that each has an unique ID
* `MQTT_BROKER_ADDRESS` IP-Address of the MQTT-Broker (usually a mosquito-Broker)


### wifi_settings.h
* `WIFI_SSID` name of your wifi network (ESP32 usually only suport 2.4 GHz)
* `WIFI_PW` password of that SSID


# TODOs
* Possible optimization: Measure battery and moisture at once
* send raw moisture value for calibration -> set-min, set-max?  // if so, how to save them internally after power cut?
