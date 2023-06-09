#include <Arduino.h>
#include <WiFiNINA.h>
#include <ArduinoHA.h>
#include <secrets.h>
#include <SPI.h>

#define RED_PIN 3
#define GREEN_PIN 5
#define BLUE_PIN 6
#define BROKER_ADDR     IPAddress(192,168,1,162)



WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);

// HALight::BrightnessFeature enables support for setting brightness of the light.
// HALight::ColorTemperatureFeature enables support for setting color temperature of the light.
// Both features are optional and you can remove them if they're not needed.
// "prettyLight" is unique ID of the light. You should define your own ID.
HALight light("prettyLight", HALight::BrightnessFeature | HALight::ColorTemperatureFeature | HALight::RGBFeature);

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

void onStateCommand(bool state, HALight* sender) {
    Serial.print("State: ");
    Serial.println(state);

    sender->setState(state); // report state back to the Home Assistant
}

void onBrightnessCommand(uint8_t brightness, HALight* sender) {
    Serial.print("Brightness: ");
    Serial.println(brightness);
    auto color = light.getCurrentRGBColor();
    float b = brightness/255.;
    color = HALight::RGBColor((uint8_t)(color.red*b), (uint8_t)(color.green*b), (uint8_t)(color.blue*b));
    analogWrite(RED_PIN, color.red);
    analogWrite(GREEN_PIN, color.green);
    analogWrite(BLUE_PIN, color.blue);

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
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender) {
    Serial.print("Red: ");
    Serial.println(color.red);
    Serial.print("Green: ");
    Serial.println(color.green);
    Serial.print("Blue: ");
    Serial.println(color.blue);

    analogWrite(RED_PIN, color.red);
    analogWrite(GREEN_PIN, color.green);
    analogWrite(BLUE_PIN, color.blue);

    sender->setRGBColor(color); // report color back to the Home Assistant
}

void setup() 
{
    Serial.begin(9600);
    for (int i = 0; i < 10 && !Serial; ++i)
    {
        delay(100);
    }

    
    Serial.println("Starting...");

    // Unique ID must be set!
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));
    Serial.print("MAC:");
    Serial.print(mac[0],HEX); Serial.print(':');
    Serial.print(mac[1],HEX); Serial.print(':');
    Serial.print(mac[2],HEX); Serial.print(':');
    Serial.print(mac[3],HEX); Serial.print(':');
    Serial.print(mac[4],HEX); Serial.print(':');
    Serial.println(mac[5],HEX);

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    // connect to wifi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) 
    {
        Serial.print(".");
        delay(500); // waiting for the connection
    }
    Serial.println();
    Serial.println("Connected to the network");

    // set device's details (optional)
    device.setName("Nano 33 IoT");
    device.setSoftwareVersion("1.0.0");

    // configure light (optional)
    light.setName("Bedroom Ili");

    // Optionally you can set retain flag for the HA commands
    // light.setRetain(true);

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
    light.onColorTemperatureCommand(onColorTemperatureCommand); // optional
    light.onRGBColorCommand(onRGBColorCommand); // optional

    mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);
}

void ensureConnected()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
}

void loop() 
{
    mqtt.loop();
    ensureConnected();

    // You can also change the state at runtime as shown below.
    // This kind of logic can be used if you want to control your switch using a button connected to the device.
    // led.setState(true); // use any state you want
}