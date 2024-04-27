// Arduino imports
#include <Arduino.h>
#include <SPI.h>

// Third party imports
#include "Sodaq_wdt.h"
#include <WiFiNINA.h>
#include <ArduinoHA.h>

// Local imports
#include <secrets.h>
#include <common.h>

// Constants
#define RED_PIN 3
#define GREEN_PIN 5
#define BLUE_PIN 6

#define WIFI_TIMEOUT 240000
#define HA_TIMEOUT 480000

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

struct LED
{
    pin_size_t pin;
    int status = 0;
    LED(pin_size_t p) : pin(p) {}
    void begin() { pinMode(pin, PinMode::OUTPUT); }
    void write(int status) { digitalWrite(pin, status); this->status=status; }
    void on() { write(HIGH); }
    void off() { write(LOW); }
};

InitialConfig* initConfig = nullptr;
LED led{LED_BUILTIN};

void forceExit()
{
    dbgln("Restarting");
    dbgflush();
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
        led.on();
        return true;
    }
    led.off();
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
    dbg("State: ");
    dbgln(state);

    sender->setState(state); // report state back to the Home Assistant
    initConfig->state = state;
    initConfig->updateLED();
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
    dbg("Brightness: ");
    dbgln(brightness);

    auto color = light.getCurrentRGBColor();
    initConfig->brightness = brightness;
    initConfig->color = color;
    initConfig->updateLED();

    sender->setBrightness(brightness); // report brightness back to the Home Assistant
}

void onColorTemperatureCommand(uint16_t temperature, HALight* sender) {
    dbg("Color temperature: ");
    dbgln(temperature);
    auto color = temperatureToRGB(temperature);

    analogWrite(RED_PIN, color.red);
    analogWrite(GREEN_PIN, color.green);
    analogWrite(BLUE_PIN, color.blue);

    sender->setColorTemperature(temperature); // report color temperature back to the Home Assistant
    initConfig->color = color;
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
    dbg("Red: ");
    dbgln(color.red);
    dbg("Green: ");
    dbgln(color.green);
    dbg("Blue: ");
    dbgln(color.blue);

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
    else 
    {
        initConfig->color = {0,0,0};
        initConfig->updateLED();
    }
    dbginit(9600);
    led.begin();
    led.write(LOW);
    #if defined(DEBUG) && DEBUG > 0
    for (int i = 0; i < 10 && !dbgstream; ++i)
    {
        safeDelay(100);
    }
    #endif // defined(DEBUG) && DEBUG > 0

    safeDelay(100);
    dbg("ID: ");
    dbgln(initConfig->id);
    dbg("Init: ");
    dbgln(*initConfig);
    dbg("R: "); dbg(initConfig->color.red); dbg("  G: "); dbg(initConfig->color.green); dbg("  B: "); dbgln(initConfig->color.blue);
    
    dbgln("Starting...");

    sodaq_wdt_reset();
    // Unique ID must be set!
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));
    sodaq_wdt_reset();
    

    // connect to wifi
    lastWifiConnection = millis();
    while (!ensureConnected()) sodaq_wdt_reset();
    dbgln();
    dbgln("Connected to the network");

    // set device's details (optional)
    device.setName("Nano 33 IoT");
    device.setSoftwareVersion(__XSTRING(VERSION));

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

    dbgln("Starting MQTT");
    dbgflush();

    mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
    // handle light states
    light.onStateCommand(onStateCommand);
    light.onBrightnessCommand(onBrightnessCommand); // optional
    // light.onColorTemperatureCommand(onColorTemperatureCommand); // optional
    light.onRGBColorCommand(onRGBColorCommand); // optional

    // Wait until mqtt is connected.
    lastHaConnection = millis();
    while (!mqtt.isConnected()) mqttUpdate();
    dbgln("Connected to mqtt");

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

    sodaq_wdt_enable(WDT_PERIOD_2X);
}



void loop() 
{
    mqtt.loop();
    ensureConnected();
    mqttUpdate();

    #if defined(DEBUG) && DEBUG > 0
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
    #endif // defined(DEBUG) && DEBUG > 0
}