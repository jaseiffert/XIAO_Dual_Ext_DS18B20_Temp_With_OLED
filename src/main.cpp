/*
# ################################################################################################ #
# File: main.cpp                                                                                   #
# Project: XIAO ESP32C6 External Temperature Sensor With OLED Display                              #
# Created Date: Thursday, August 28th 2025, 09:30 pm                                               #
# Author: Jeffery Seiffert                                                                         #
# -----                                                                                            #
# Last Modified: Sun Aug 31 2025                                                                   #
# Modified By: Jeffery Seiffert                                                                    #
# -----                                                                                            #
# Copyright (c) 2025 Jeffery A. Seiffert                                                           #
#                                                                                                  #
# GNU General Public License v3.0                                                                  #
# -----                                                                                            #
# HISTORY:                                                                                         #
# Date      	By	Comments                                                                       #
# ----------	---	----------------------------------------------------------                     #
#                                                                                                  #
# ################################################################################################ #
*/

#include <Arduino.h>
#include <Wire.h>
#include <Esp.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <soc/gpio_struct.h>

#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TaskScheduler.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "configuration.h"
#include "secrets.h"

#define PRINT_DEBUG 1

#if PRINT_DEBUG == 1
#define debugp(x) Serial.print(x)
#define debugpln(x, ...) Serial.println(x)
#define debugpf(x, ...) Serial.printf(x)
#else
#define debugp(x)
#define debugpln(x, ...)
#define debugpf(x, ...)
#endif

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)

#define I2C_SDA D4
#define I2C_SCL D5

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS D3

#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for microseconds to seconds
#define TIME_TO_SLEEP_uS 60       // Time ESP32 will go to sleep (in seconds)

#define mS_TO_S_FACTOR 1000ULL // Conversion factor for milliseconds to seconds
#define TIME_TO_SLEEP_mS 20    // Time ESP32 will go to sleep (in seconds)

#define mS_TO_M_FACTOR 60000ULL // Conversion factor for milliseconds to minute
#define SLEEP_DELAY_M 1         // Time ESP32 will go to sleep (in Minutes)

// MQTT: ID, server IP, port, username and password
const PROGMEM char *MQTT_CLIENT_ID = MQTT_CLIENTID;
const PROGMEM char *MQTT_SERVER_IP = MQTT_SERVERIP;
const PROGMEM uint16_t MQTT_SERVER_PORT = MQTT_SERVERPORT;
const PROGMEM char *MQTT_USER = MQTTUSER;
const PROGMEM char *MQTT_PASSWORD = MQTTPWD;

// MQTT: topic
const PROGMEM char *MQTT_SENSOR_TOPIC = MQTT_SENSORTOPIC;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Text Size 1 (default): 6x8 pixels per character.
// Text Size 2: 12x16 pixels per character (each pixel in the default font becomes a 2x2 block).
// Text Size 3: 18x24 pixels per character (each pixel becomes a 3x3 block

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
int count = 0;

// Callback methods prototypes
void t1Callback();
void t2Callback();

const int SECONDS_15 = 15000; // in milliseconds
const int SECONDS_30 = 30000; // in milliseconds
const int MINUTES_1 = 60000;  // in milliseconds
const int MINUTES_5 = 300000; // in milliseconds

// Tasks
Task t1(DISPLAY_TEMP, TASK_FOREVER, &t1Callback); // adding task to the chain on creation
Task t2(MQTT_SEND, TASK_FOREVER, &t2Callback);  // adding task to the chain on creation

void sendMQTT();
void callback(char *p_topic, byte *p_payload, unsigned int p_length);
String checkTemperature();
void displayTemperature();
void setupWiFi();
void WiFiEvent(WiFiEvent_t event);
void setupLCD();
int wifiSignalStrength();
uint8_t findDevices(int pin);

/* Your WiFi Credentials */
const char *ssid = WIFI_USER;    // SSID
const char *password = WIFI_PWD; // Password

unsigned long sleep_time = 0;
unsigned long last_sleep_time = 0;

// TaskScheduler
Scheduler runner;

void setup()
{
    Serial.begin(115200);
    delay(1000); // Take some time to open up the Serial Monitor

    // Enable External Antenna on XIAO ESP32C6
    pinMode(WIFI_ENABLE, OUTPUT);   // pinMode(3, OUTPUT);
    digitalWrite(WIFI_ENABLE, LOW); // digitalWrite(3, LOW); // Activate RF switch control
    delay(100);
    pinMode(WIFI_ANT_CONFIG, OUTPUT);    // pinMode(14, OUTPUT);
    digitalWrite(WIFI_ANT_CONFIG, HIGH); // digitalWrite(14, HIGH); // Use external antenna

    setupWiFi();

    setupLCD();

    // init the MQTT connection
    mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);

    runner.init();
    debugpln("Initialized scheduler");

    runner.addTask(t1);
    debugpln("added t1");

    runner.addTask(t2);
    debugpln("added t2");

    delay(5000);

    t1.enable();
    debugpln("Enabled t1");

    t2.enable();
    debugpln("Enabled t2");

    // Start up the library
    sensors.begin();

    displayTemperature();
    // sendMQTT();

    // findDevices(ONE_WIRE_BUS);

} // void setup()

void loop()
{
    runner.execute();
} // void loop()

// function called when a MQTT message arrived
void callback(char *p_topic, byte *p_payload, unsigned int p_length)
{
}

void t1Callback()
{
    debugp("Display Temp: ");
    displayTemperature();
}

void t2Callback()
{
    debugp("Send MQTT: ");
    sendMQTT();
}

void sendMQTT()
{

    String mqttData = "{";
    mqttData += checkTemperature();
    mqttData += ",\"wifisig\":\"" + String(WiFi.RSSI()) + "\"";
    mqttData += "}";

    debugpln("Attempting MQTT connection...");

    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
    {
        debugpln("Connected to MQTT Server");
    }
    else
    {
        debugpln("Error Connecting to MQTT Server");
    }
    if (mqttClient.connected())
    {
        debugpln(mqttData);
        if (mqttClient.publish(MQTT_SENSOR_TOPIC, mqttData.c_str(), true))
        {
            debugpln("MQTT Message sent successfully");
        }
        else
        {
            debugpln("Error sending MQTT Message");
        }

        debugpln("INFO: Closing the MQTT connection");
        mqttClient.disconnect();
    } // if (mqttClient.connected())
}

void displayTemperature()
{
    String tempUnit = "";
    float currentTemp[] = {0.00, 0.00};
    int sensorCount = 0;

    sensorCount = sensors.getDS18Count();
    sensors.requestTemperatures();

    debugpln("Sensor Count: " + String(sensorCount));

    if (FAHRENHEIT)
    {
        for (int i = 0; i < sensorCount; i++)
        {
            currentTemp[i] = sensors.getTempFByIndex(i);
        }
        tempUnit = "F";
    }
    else
    {
        for (int i = 0; i < sensorCount; i++)
        {
            currentTemp[i] = sensors.getTempCByIndex(i);
        }
        tempUnit = "C";
    }
    // currentHumidity = humidity.relative_humidity;

    debugp("Temperature: ");
    for (int i = 0; i < sensorCount; i++)
    {
        debugp(currentTemp[i]);
        debugp(", ");
    }
    debugpln(tempUnit);

    display.setTextSize(1); // Normal 1:1 pixel scale
    display.setCursor(0, 11);
    display.fillRect(0, 11, display.width() - 40, 10, SSD1306_BLACK);
    display.print(wifiSignalStrength());
    if (SIGNAL_STRENGTH_DB)
    {
        display.print("db");
    }
    else
    {
        display.print("%");
    }

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.fillRect(0, 23, display.width() - 10, 26, SSD1306_BLACK);
    // display.display(); // Update screen with each newly-drawn rectangle
    display.setCursor(0, 23); // Start at top-left corner
    // display.print("Temperature: ");
    display.print(currentTemp[0]);
    display.print(" ");
    display.print((char)247);
    display.print(tempUnit);
    display.fillRect(0, 43, display.width() - 10, 26, SSD1306_BLACK);
    display.setCursor(0, 43); // Start at top-left corner
    display.print(currentTemp[1]);
    display.print(" ");
    display.print((char)247);
    display.print(tempUnit);
    //display.fillRect(0, 63, 127, 1, SSD1306_WHITE); // Bottom White Line
    display.display();

} // void displayTemperature()

String checkTemperature()
{
    String tempUnit = "";
    float currentTemp[] = {0.00, 0.00};
    int sensorCount = 0;

    sensorCount = sensors.getDS18Count();
    sensors.requestTemperatures();

    if (FAHRENHEIT)
    {
        for (int i = 0; i < sensorCount; i++)
        {
            currentTemp[i] = sensors.getTempFByIndex(i);
        }
        tempUnit = "F";
    }
    else
    {
        for (int i = 0; i < sensorCount; i++)
        {
            currentTemp[i] = sensors.getTempCByIndex(i);
        }
        tempUnit = "C";
    }
    // currentHumidity = humidity.relative_humidity;

    debugp("Temperature: ");
    for (int i = 0; i < sensorCount; i++)
    {
        debugp(currentTemp[i]);
        debugp(", ");
    }
    debugpln(tempUnit);

    String data = "";
    data += "\"temp_01\":\"" + String(currentTemp[0]) + "\",";
    data += "\"temp_02\":\"" + String(currentTemp[1]) + "\"";
    debugp("Data String: ");
    debugpln((data));

    return data;
}

void setupWiFi()
{
    // Connect WiFi
    debugpln("---------- Start WiFi Connect ----------");
    int connectCount = 0;
    // Auto reconnect is set true as default
    // To set auto connect off, use the following function
    WiFi.setAutoReconnect(true);

    WiFi.onEvent(WiFiEvent);

    debugpln(F("Connect to WiFi"));

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    debugpln("Connecting");
    while (WiFi.status() != WL_CONNECTED && connectCount < 101)
    {
        delay(500);
        debugp(".");
        connectCount++;
    }
    debugpln("");
    debugp("Connected to WiFi network with IP Address: ");
    debugpln(WiFi.localIP());
    debugpln("---------- End WiFi Connect ----------");

} // void setupWiFi()

void WiFiEvent(WiFiEvent_t event)
{
    // debugpf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case ARDUINO_EVENT_WIFI_READY:
        debugpln("WiFi interface ready");
        break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        debugpln("Completed scan for access points");
        break;
    case ARDUINO_EVENT_WIFI_STA_START:
        debugpln("WiFi client started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        debugpln("WiFi clients stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        debugpln("Connected to access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        debugpln("Disconnected from WiFi access point");
        // digitalWrite(LED_BLUE, HIGH);
        break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
        debugpln("Authentication mode of access point has changed");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        debugp("Obtained IP address: ");
        debugpln(WiFi.localIP());
        debugp("WiFi Signal Strength: ");
        debugpln(WiFi.RSSI());
        // digitalWrite(LED_BLUE, LOW);
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        debugpln("Lost IP address and IP address is reset to 0");
        break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
        debugpln("WiFi Protected Setup (WPS): succeeded in enrollee mode");
        break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
        debugpln("WiFi Protected Setup (WPS): failed in enrollee mode");
        break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
        debugpln("WiFi Protected Setup (WPS): timeout in enrollee mode");
        break;
    case ARDUINO_EVENT_WPS_ER_PIN:
        debugpln("WiFi Protected Setup (WPS): pin code in enrollee mode");
        break;
    case ARDUINO_EVENT_WIFI_AP_START:
        debugpln("WiFi access point started");
        break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
        debugpln("WiFi access point  stopped");
        break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        debugpln("Client connected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        debugpln("Client disconnected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
        debugpln("Assigned IP address to client");
        break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
        debugpln("Received probe request");
        break;
    case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
        debugpln("AP IPv6 is preferred");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
        debugpln("STA IPv6 is preferred");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP6:
        debugpln("Ethernet IPv6 is preferred");
        break;
    case ARDUINO_EVENT_ETH_START:
        debugpln("Ethernet started");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        debugpln("Ethernet stopped");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        debugpln("Ethernet connected");
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        debugpln("Ethernet disconnected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        debugpln("Obtained IP address");
        break;
    default:
        break;
    }
} // void WiFiEvent(WiFiEvent_t event)

void setupLCD()
{
    debugpln("------------- SSD1306 LCD --------------");

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        debugpln(F("SSD1306 allocation failed"));
        // for(;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2

    // Clear the buffer
    display.clearDisplay();

    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.print(F(DISPLAY_TITLE));
    display.display();
} // void setupLCD()

int wifiSignalStrength()
{
    int signalStrength = WiFi.RSSI();

    if (!SIGNAL_STRENGTH_DB)
    {
        int percentage = map(signalStrength, -100, -30, 0, 100);
        signalStrength = constrain(percentage, 0, 100);
    }
    return signalStrength;
} // int wifiSignalStrength()

uint8_t findDevices(int pin)
{
    OneWire ow(pin);

    uint8_t address[8];
    uint8_t count = 0;

    if (ow.search(address))
    {
        Serial.print("\nuint8_t pin");
        Serial.print(pin, DEC);
        Serial.println("[][8] = {");
        do
        {
            count++;
            Serial.println("  {");
            for (uint8_t i = 0; i < 8; i++)
            {
                Serial.print("0x");
                if (address[i] < 0x10)
                    Serial.print("0");
                Serial.print(address[i], HEX);
                if (i < 7)
                    Serial.print(", ");
            }
            Serial.println("  },");
        } while (ow.search(address));

        Serial.println("};");
        Serial.print("// nr devices found: ");
        Serial.println(count);
    }

    return count;
} // uint8_t findDevices(int pin)

