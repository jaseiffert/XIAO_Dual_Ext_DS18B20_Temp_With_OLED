// Program Configuration

// MQTT Configuration
#define MQTT_CLIENTID "frigfreezer_01"
#define MQTT_SERVERIP "192.168.15.92"
#define MQTT_SERVERPORT 1883
#define MQTT_SENSORTOPIC "kitchen/temp/ext_temp_01"

// Temperature
#define FAHRENHEIT true // Convert Temperatures to Fahrenheit

// Display
#define DISPLAY_TITLE "DS18B20 Temp Sensors" // Max of 20 characters
#define SIGNAL_STRENGTH_DB true // Display the signal strength on display in DB (true) or Percent (false)

// Schedule
#define DISPLAY_TEMP 15000 // 15 Seconds in milliseconds
#define MQTT_SEND 60000 // 5 minutes in milliseconds

// 1 Second = 1000 miliseconds
// 15 Seconds = 15000 milliseconds
// 30 Seconds = 30000 milliseconds
// 1 Minute = 60000 milliseconds
// 5 Minutes = 300000 milliseconds
