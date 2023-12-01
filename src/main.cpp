// Arduino imports
#include <Arduino.h>
#include <SPI.h>

// Third party imports
#include "Sodaq_wdt.h"
#include <WiFiNINA.h>
#include <ArduinoHA.h>

// Local imports
#include <secrets.h>

// Constants
#define RED_PIN 3
#define GREEN_PIN 5
#define BLUE_PIN 6

#define WIFI_TIMEOUT 240
#define HA_TIMEOUT 480

// Services
WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);

// HALight::BrightnessFeature enables support for setting brightness of the light.
// HALight::ColorTemperatureFeature enables support for setting color temperature of the light.
// Both features are optional and you can remove them if they're not needed.
// "bedroom_ili_2" is unique ID of the light. You should define your own ID.
HALight light("bedroom_ili", HALight::BrightnessFeature | HALight::RGBFeature);


uint32_t lastWDT = 0;
uint32_t lastWifiConnection = 0;
uint32_t lastHaConnection = 0;

HALight::RGBColor temperatureToRGB(uint16_t temperature) {
  HALight::RGBColor color;

  // Temperature range for color mapping
  const uint16_t minTemp = 153;
  const uint16_t maxTemp = 500;

  // Map the temperature to the range of 0-255 for RGB values
  float mappedTemp = map(temperature, minTemp, maxTemp, 0, 255);

  // Calculate RGB values based on the mapped temperature
  if (mappedTemp <= 127.5) {
    color.red = 255;
    color.green = map(mappedTemp, 0, 127.5, 0, 255);
    color.blue = 0;
  } else {
    color.red = map(mappedTemp, 127.5, 255, 255, 0);
    color.green = 255;
    color.blue = map(mappedTemp, 127.5, 255, 0, 255);
  }

  return color;
}

// A random number used to validate the integrity of the values in InitialConfig
#define MEMORY_ID 478295

struct InitialConfig
{
    int id;
    HALight::RGBColor color;
    uint8_t brightness;
    bool state;
    void init() { id = MEMORY_ID; }
    operator bool() { return id == MEMORY_ID; }
    void updateLED()
    {
        if (state)
        {
            float b = brightness/255.;
            color = HALight::RGBColor((uint8_t)(color.red*b), (uint8_t)(color.green*b), (uint8_t)(color.blue*b));
            analogWrite(RED_PIN, color.red);
            analogWrite(GREEN_PIN, color.green);
            analogWrite(BLUE_PIN, color.blue);
        }
        else
        {
            analogWrite(RED_PIN, 0);
            analogWrite(GREEN_PIN, 0);
            analogWrite(BLUE_PIN, 0);
        }
    }
};

InitialConfig* initConfig = nullptr;

void forceExit()
{
    Serial.println("Restarting");
    Serial.flush();
    sodaq_wdt_disable();
    sodaq_wdt_enable(WDT_PERIOD_1DIV64);
    exit(1);
}

void update()
{
    if (millis() - lastWDT > 1000)
    {
        sodaq_wdt_reset();
        lastWDT = millis();
    }
}

void safeDelay(uint32_t ms)
{
    uint32_t end = millis() + ms;
    while (millis() < end) update();
}

bool ensureConnected()
{
    if (WiFi.status() == wl_status_t::WL_CONNECTED)
    {
        lastWifiConnection = millis();
        return true;
    }
    if (millis() - lastWifiConnection > WIFI_TIMEOUT) 
    {
        forceExit();
    }

    // connect to wifi
    const char* wifi_ssid = WIFI_SSID;
    const char* wifi_password = WIFI_PASSWORD;
    WiFi.begin(wifi_ssid, wifi_password);
    safeDelay(500); // waiting for the connection
    return false;
}

void mqttUpdate()
{
    mqtt.loop();
    sodaq_wdt_reset();
    if (mqtt.isConnected())
    {
        lastHaConnection = millis();
    }
    else
    {
        if (millis() - lastHaConnection > HA_TIMEOUT) 
        {
            forceExit();
        }
    }
}

void onStateCommand(bool state, HALight* sender) {
    Serial.print("State: ");
    Serial.println(state);

    sender->setState(state); // report state back to the Home Assistant
    initConfig->state = state;
    initConfig->updateLED();
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
    Serial.print("Brightness: ");
    Serial.println(brightness);

    auto color = light.getCurrentRGBColor();
    initConfig->brightness = brightness;
    initConfig->color = color;
    initConfig->updateLED();

    sender->setBrightness(brightness); // report brightness back to the Home Assistant
}

void onColorTemperatureCommand(uint16_t temperature, HALight* sender) {
    Serial.print("Color temperature: ");
    Serial.println(temperature);
    auto color = temperatureToRGB(temperature);

    analogWrite(RED_PIN, color.red);
    analogWrite(GREEN_PIN, color.green);
    analogWrite(BLUE_PIN, color.blue);

    sender->setColorTemperature(temperature); // report color temperature back to the Home Assistant
    initConfig->color = color;
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
    Serial.print("Red: ");
    Serial.println(color.red);
    Serial.print("Green: ");
    Serial.println(color.green);
    Serial.print("Blue: ");
    Serial.println(color.blue);

    initConfig->color = color;
    initConfig->updateLED();

    sender->setRGBColor(color); // report color back to the Home Assistant

}

void setup() 
{
    initConfig = (InitialConfig*)malloc(sizeof(InitialConfig));
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    if (*initConfig)
    {
        initConfig->updateLED();
    }
    sodaq_wdt_enable(WDT_PERIOD_2X);
    Serial.begin(9600);
    for (int i = 0; i < 10 && !Serial; ++i)
    {
        safeDelay(100);
    }
    safeDelay(100);
    Serial.print("ID: ");
    Serial.println(initConfig->id);
    Serial.print("Init: ");
    Serial.println(*initConfig);
    Serial.print("R: "); Serial.print(initConfig->color.red); Serial.print("  G: "); Serial.print(initConfig->color.green); Serial.print("  B: "); Serial.println(initConfig->color.blue);
    
    Serial.println("Starting...");

    sodaq_wdt_reset();
    // Unique ID must be set!
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));
    sodaq_wdt_reset();
    

    // connect to wifi
    lastWifiConnection = millis();
    while (!ensureConnected()) sodaq_wdt_reset();
    Serial.println();
    Serial.println("Connected to the network");

    // set device's details (optional)
    device.setName("Nano 33 IoT");
    device.setSoftwareVersion("1.1.0");

    // configure light (optional)
    light.setName("Bedroom Ili");

    // Optionally you can set retain flag for the HA commands
    light.setRetain(true);

    // Maximum brightness level can be changed as follows:
    // light.setBrightnessScale(50);

    // Optionally you can enable optimistic mode for the HALight.
    // In this mode you won't need to report state back to the HA when commands are executed.
    // light.setOptimistic(true);

    // Color temperature range (optional)
    // light.setMinMireds(50);
    // light.setMaxMireds(200);

    // handle light states
    light.onStateCommand(onStateCommand);
    light.onBrightnessCommand(onBrightnessCommand); // optional
    // light.onColorTemperatureCommand(onColorTemperatureCommand); // optional
    light.onRGBColorCommand(onRGBColorCommand); // optional

    mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);

    // Wait until mqtt is connected.
    lastHaConnection = millis();
    while (!mqtt.isConnected()) mqttUpdate();
    Serial.println("Connected to mqtt");

    // mqtt.addDeviceType(&light);

    if (*initConfig)
    {
        light.setCurrentRGBColor(initConfig->color);
        light.setCurrentBrightness(initConfig->brightness);
        light.setCurrentState(initConfig->state);
    }
    else
    {
        initConfig->init();
        initConfig->brightness = light.getCurrentBrightness();
        initConfig->state = light.getCurrentState();
        initConfig->color = light.getCurrentRGBColor();
        initConfig->updateLED();
    }

    
}



void loop() 
{
    mqtt.loop();
    ensureConnected();
    mqttUpdate();
    if (Serial.available())
    {
        long c = Serial.parseInt();
        switch (c)
        {
        case 1: // Erase RAM config
            initConfig->id = 0;
            forceExit();
            break;
        default:
            break;
        }
        while (Serial.available()) Serial.read();
        Serial.println("Command done");
    }
}