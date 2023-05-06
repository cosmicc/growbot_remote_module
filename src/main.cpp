// Growbot Remote ESP32 Plant Sensor
//
// Created by: Ian Perry (ianperry99@gmail.com)
// https://github.com/cosmicc/growbot_remote_module

// soil moisture value ref:  2860 open air, 2400 dry, 1000 submerged in water,

#include <EEPROM.h>
#include <Arduino.h>
#include <nvs_flash.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <SPIFFS.h>
#include <Mapf.h>

// compile definitions
#define VERSION 1                // firmware version
#undef RESET_DATA                // reset datafile on boot
#undef DEBUG_SERIAL              // enable serial debug
#define CPU_FREQ_MHZ 80          // set CPU frequency in MHz Lower then 80 seems to fail wifi
#define SLEEP_MIN 60             // Sleep time in minutes
#define UPLOAD_EVERY 12          // Upload data every x sleep cycles
#define MOISTURE_WARN_PCT 30     // Warning moisture percentage
#define BATTERY_WARN_VOLTAGE 3.35// Warning battery voltage
#define BATTERY_MIN_VOLTAGE 3.30 // Minimum battery voltage to operate
#define BATTERY_MAX_VOLTAGE 4.20 // Maximum battery voltage for percentage calulation
#define SENSOR_SAMPLES 5         // Number of sensor samples to take for avg
#define SENSOR_DELAY_MS 10       // Delay between sensor reads in milliseconds
#define WIFI_TIMEOUT_SECS 30     // Wifi connection timeout in seconds
#define NTP_TIMEOUT_SECS 10      // set NTP server timeout in seconds
#define WDT_TIMEOUT_SECS 20      // watchdog timer timeout in seconds
#define API_SEND_DELAY_MS 10     // Delay between API calls in milliseconds
#define uS_TO_S_FACTOR 1000000   // Conversion factor for micro seconds to seconds
#define BATTERY_PIN 39           // Analog input pin to read battery voltage

// Only GPIO 32-36 (5 total) are ADC1 channels and are the only pins that can be used for soil_sensors
// GPIO 39 is ADC1 also but is used for battery monitoring
// ADC2 channels cannot be used because they are used by wifi
int sensor_pins[] = {36}; // list of each soil sensor gpio pin this device will monitor

// Global variables
int tz_offset_min = -300;                                         // Timezone offset in seconds (-5 hours)
int dst_offset_min = 60;                                          // daylight savings time offset in minutes
int httpResponseCode;                                             // variable to hold http response code
int sensor_length = sizeof(sensor_pins) / sizeof(sensor_pins[0]); // number of sensors
bool spiff_ready = false;                                         // SPIFFS ready flag
bool http_success_bit = true;                                     // HTTP success flag
bool system_problem = false;                                      // System problem flag
String problem_reason;                                            // System problem reason
char *api_url;                                                    // API URL
char *ntp_server;                                                 // NTP server
int iter;                                                         // loop iterator

// function definitions
void show_last_restart_reason();
void writeStringToEEPROM(int addrOffset, const char *str);
char *readStringFromEEPROM(int addrOffset);
void writeIntToEEPROM(int addrOffset, int value);
int readIntfromEEPROM(int addrOffset);
String getMacLast4();
String encryptPayload(String payload);
void send_payload(String jsonPayload);
void check_datafile();
void write_spiff(String data);
int get_avg_moisture(int aout_pin);
bool set_time();
tm get_time();
float get_battery_voltage();
int get_battery_pct(float volt);
String removeNewlines(const char *inputString);
#ifdef DEBUG_SERIAL
void show_time();
void show_battery_voltage();
#endif

// display the reason for the last restart
void show_last_restart_reason()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    switch (reset_reason)
    {
    case ESP_RST_UNKNOWN:
#ifdef DEBUG_SERIAL
        log_w("Last reset reason: unknown");
#endif
        problem_reason = "Last reset: unknown";
        system_problem = true;
        break;
    case ESP_RST_POWERON:
#ifdef DEBUG_SERIAL
        log_i("Last reset reason: power-on reset");
#endif
        break;
    case ESP_RST_EXT:
#ifdef DEBUG_SERIAL
        log_w("Last reset reason: external reset");
#endif
        problem_reason = "external reset";
        system_problem = true;
        break;
    case ESP_RST_SW:
#ifdef DEBUG_SERIAL
        log_w("Last reset reason: software reset");
#endif
        problem_reason = "software reset";
        system_problem = true;
        break;
    case ESP_RST_PANIC:
#ifdef DEBUG_SERIAL
        log_e("Last reset reason: panic reset");
#endif
        problem_reason = "panic reset";
        system_problem = true;
        break;
    case ESP_RST_INT_WDT:
#ifdef DEBUG_SERIAL
        log_e("Last reset reason: interrupt watchdog reset");
#endif
        problem_reason = "interrupt watchdog reset";
        system_problem = true;
        break;
    case ESP_RST_TASK_WDT:
#ifdef DEBUG_SERIAL
        log_e("Last reset reason: task watchdog reset");
#endif
        problem_reason = "task watchdog reset";
        system_problem = true;
        break;
    case ESP_RST_WDT:
#ifdef DEBUG_SERIAL
        log_e("Last reset reason: other watchdog reset");
#endif
        problem_reason = "other watchdog reset";
        system_problem = true;
        break;
    case ESP_RST_DEEPSLEEP:
#ifdef DEBUG_SERIAL
        log_i("Last reset reason: deep sleep reset");
#endif
        break;
    case ESP_RST_BROWNOUT:
#ifdef DEBUG_SERIAL
        log_e("Last reset reason: brownout reset");
#endif
        problem_reason = "brownout reset";
        system_problem = true;
        break;
    case ESP_RST_SDIO:
#ifdef DEBUG_SERIAL
        log_w("Last reset reason: SDIO reset");
#endif
        problem_reason = "SDIO reset";
        system_problem = true;
        break;
    default:
#ifdef DEBUG_SERIAL
        log_w("Last reset reason: unknown");
#endif
        problem_reason = "Last reset: unknown";
        system_problem = true;
        break;
    }
}

// write string to EEPROM
void writeStringToEEPROM(int addrOffset, const char *str)
{
    int strLength = strlen(str) + 1; // Add 1 to include null terminator
    for (int i = 0; i < strLength; i++)
    {
        EEPROM.write(addrOffset + i, str[i]);
    }
    EEPROM.commit(); // Commit changes to flash memory
}

// read string from EEPROM
char *readStringFromEEPROM(int addrOffset)
{
    int maxLength = 48; // Maximum length of SSID or password
    char *str = new char[maxLength];
    char c = EEPROM.read(addrOffset);
    int i = 0;
    while (c != '\0' && i < maxLength - 1)
    {
        str[i] = c;
        i++;
        addrOffset++;
        c = EEPROM.read(addrOffset);
    }
    str[i] = '\0'; // Add null terminator
    return str;
}

// write integer to EEPROM
void writeIntToEEPROM(int addrOffset, int value)
{
    EEPROM.put(addrOffset, value); // write the integer value to address 192
    EEPROM.commit();               // save the changes to the EEPROM
}

// read integer from EEPROM
int readIntFromEEPROM(int addrOffset)
{
    int value = 0;                 // initialize variable to store the value read from EEPROM
    EEPROM.get(addrOffset, value); // read the integer value from address 192
    return value;                  // return the integer value
}

// Get the last 4 bytes of the MAC address
String getMacLast4()
{
    byte mac[6];
    WiFi.macAddress(mac); // Get MAC address
    // Convert last 4 MAC address bytes to string
    String macLast4 = "";
    for (int i = 2; i < 6; i++)
    {
        String hex = String(mac[i], HEX);
        macLast4 += hex;
    }
    return macLast4;
}

String removeNewlines(String inputString)
{
    String outputString = "";
    char *token;
    char inputCopy[inputString.length() + 1]; // Make a copy of the input string
    inputString.toCharArray(inputCopy, sizeof(inputCopy));
    token = strtok(inputCopy, "\n"); // Get the first token
    outputString = token;            // Set the output string to the first token
    while (token != NULL)
    {
        token = strtok(NULL, "\n"); // Get the next token
        if (token != NULL)
        {
            outputString += token; // Append the next token to the output string
        }
    }
    return outputString;
}

// Send json sensor data to API
void send_payload(String jsonPayload, bool writespiff)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        api_url = readStringFromEEPROM(96);
        #ifdef DEBUG_SERIAL
        log_i("Using API URL: %s", api_url);
        #endif
        HTTPClient httpClient;
        httpClient.begin(api_url); // Replace with your API endpoint
        httpClient.addHeader("Content-Type", "application/json");
        httpClient.addHeader("Accept", "application/json");
        esp_task_wdt_reset();
        #ifdef DEBUG_SERIAL
        log_i("Sending Payload: %s", jsonPayload.c_str());
        #endif
        esp_task_wdt_reset();
        httpResponseCode = httpClient.POST(jsonPayload);
        #ifdef DEBUG_SERIAL
        log_i("HTTP Response Code: %s", String(httpResponseCode));
        #endif
        if (httpResponseCode != 500)
        {
            String response = httpClient.getString();
            String resp = removeNewlines(response);
            #ifdef DEBUG_SERIAL
            log_i("HTTP Response: %s", resp.c_str());
            #endif
        }
        if (httpResponseCode == 200)
        {
            #ifdef DEBUG_SERIAL
            log_i("Sensor data sent to API successfully");
            #endif
            http_success_bit = true;
        }
        else
        {
            #ifdef DEBUG_SERIAL
            log_e("Failed to send sensor data to API");
            #endif
            http_success_bit = false;
            if (writespiff)
            {
                #ifdef DEBUG_SERIAL
                log_w("Saving sensor data to SPIFFS because API is unreachable");
                #endif
                write_spiff(jsonPayload);
            }
        }
        //httpClient.end();
    }
    else
    {
        if (writespiff)
        {
            if (iter != UPLOAD_EVERY)
            #ifdef DEBUG_SERIAL
            {
            #endif
                write_spiff(jsonPayload);
            #ifdef DEBUG_SERIAL
                log_w("Saving sensor data to SPIFFS");
            }
            else
                log_i("Saving sensor data to SPIFFS because its not time to upload yet");
            #endif
        }
    }
}

// Check if there is any sensor data stored on the spiff partition
void check_datafile()
{
    // Check if the data file exists on the spiff partition
    if (!SPIFFS.exists("/data.txt"))
    {
        #ifdef DEBUG_SERIAL
        log_i("No archived sensor data was found on SPIFFS");
        #endif
    }
    else
    {
        File data_file = SPIFFS.open("/data.txt", FILE_READ);
        if (!data_file)
        {
            #ifdef DEBUG_SERIAL
            log_i("No archived sensor data was found on SPIFFS");
            #endif
        }
        else
        {
            #ifdef DEBUG_SERIAL
            log_i("Processing archived sensor data entries found on SPIFFS");
            #endif
            for (String line = data_file.readStringUntil('\n'); line != ""; line = data_file.readStringUntil('\n'))
            {
                esp_task_wdt_reset();
                send_payload(line, false);
                delay(API_SEND_DELAY_MS);
            }
            data_file.close();
            if (http_success_bit)
            {
                // Delete the file
                #ifdef DEBUG_SERIAL
                log_i("Removing completed data file from SPIFFS");
                #endif
                SPIFFS.remove("/data.txt");
            }
        }
    }
}

// write sensor data to the spiff partition
void write_spiff(String data)
{
    if (spiff_ready)
    {
        File data_file = SPIFFS.open("/data.txt", FILE_APPEND);
        data_file.println(data);
        data_file.close();
        #ifdef DEBUG_SERIAL
        log_i("Data saved to SPIFFS successfully");
        #endif
    }
    else
    {
        #ifdef DEBUG_SERIAL
        log_e("SPIFFS not ready, cannot save data");
        #endif
    }
}

// Get the average moisture value from the sensor
int get_avg_moisture(int aout_pin)
{
    if (aout_pin < 32 || aout_pin > 39)
    {
        #ifdef DEBUG_SERIAL
        log_e("Invalid sensor pin [%d]", aout_pin);
#       endif
        return 0;
    }
    int total = 0;
    int avg = 0;
    #ifdef DEBUG_SERIAL
    log_d("Reading sensor [GPIO%d] for [%d]x[%dms] samples...", aout_pin, SENSOR_SAMPLES, SENSOR_DELAY_MS);
    int start_time = millis();
    #endif
    for (int i = 0; i < SENSOR_SAMPLES; i++)
    {
        int value = analogRead(aout_pin); // read the analog value from sensor
        total += value;
        esp_task_wdt_reset();
        delay(SENSOR_DELAY_MS);
    }
    avg = total / SENSOR_SAMPLES;
    #ifdef DEBUG_SERIAL
    log_d("Sensor [%d] read time for [%d]x[%dms] samples: [%dms]", aout_pin, SENSOR_SAMPLES, SENSOR_DELAY_MS, (millis() - start_time));
    #endif
    return avg;
}

bool is_wifi_connected()
{
    if (WiFi.status() == WL_CONNECTED)
        return true;
    else
        return false;
}

void connect_wifi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        char *ssid = readStringFromEEPROM(0);
        char *password = readStringFromEEPROM(48);
        WiFi.begin(ssid, password);
        #ifdef DEBUG_SERIAL
        log_i("Connecting to WiFi...");
        #endif
        for (int i = 0; i < WIFI_TIMEOUT_SECS * 10; i++)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                #ifdef DEBUG_SERIAL
                log_i("Connected to WiFi");
                #endif
                break;
            }
            esp_task_wdt_reset();
            delay(100);
        }
        if (WiFi.status() != WL_CONNECTED)
        {
            #ifdef DEBUG_SERIAL
            log_e("Failed to connect to WiFi");
            #endif
        }
    }
    // Sync time with NTP server
    if (WiFi.status() == WL_CONNECTED)
    {
        #ifdef DEBUG_SERIAL
        log_i("Syncing time with NTP server");
        #endif
        esp_task_wdt_reset();
        ntp_server = readStringFromEEPROM(144);
        #ifdef DEBUG_SERIAL
        log_i("Using NTP server: %s", ntp_server);
        #endif
        configTime(0, 0, ntp_server);
    }
    #ifdef DEBUG_SERIAL
    else   
        log_i("Cannot Sync NTP time, no wifi connection");
    #endif
}

// Show current datetime
void show_time()
{
    struct tm time = get_time();
    log_i("Current time: %02d-%02d-%04d %02d:%02d:%02d", time.tm_mon, time.tm_mday, time.tm_year, time.tm_hour, time.tm_min, time.tm_sec);
}

// Get current datetime from RTC
tm get_time()
{
    struct tm time;
    if (!getLocalTime(&time))
    {
        #ifdef DEBUG_SERIAL
        log_e("Could not obtain time info from RTC, trying NTP sync");
        #endif
        // Initialize wifi
        connect_wifi();
        esp_task_wdt_reset();
        // Log the current time to console
        getLocalTime(&time);
        time.tm_year += 1900;
        time.tm_mon += 1;
        return time;
    }
    time.tm_year += 1900;
    time.tm_mon += 1;
    return time;
}

void show_battery_voltage()
{
    float batteryVoltage = get_battery_voltage();
    int pct = get_battery_pct(batteryVoltage);
    if (batteryVoltage < BATTERY_MIN_VOLTAGE && batteryVoltage > 0.5)
    {
        log_e("Battery voltage is critical! [%0.2fv] (%d%%) Sleeping indefinately", batteryVoltage, pct);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, HIGH);
        esp_deep_sleep_start();
    }
    else if (batteryVoltage < 0.5)
    {
        log_e("Battery is disconnected [%0.2fv] (%d%%)", batteryVoltage, pct);
    }
    else if (batteryVoltage < BATTERY_WARN_VOLTAGE)
    {
        log_w("Battery voltage is LOW! [%0.2fv] (%d%%)", batteryVoltage, pct);
    }
    else
    {
        log_i("Battery voltage is normal [%0.2fv] (%d%%)", batteryVoltage, pct);
    }
}

float get_battery_voltage()
{
    const float R1 = 100000.0f;
    const float R2 = 220000.0f;
    float sum = 0.0f;
    for (int i = 0; i < 5; i++)
    {
        sum += analogRead(BATTERY_PIN);
    }
    float vs = sum / 5.0f;
    float vin = vs * (3.3f / 4095.0f) * (R1 + R2) / R2;
    return vin;
}

int get_battery_pct(float voltage)
{
    int pct = mapf(voltage, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100);
    if (pct < 0)
        pct = 0;
    else if (pct > 100)
        pct = 100;
    return pct;
}

extern "C" void app_main()
{
    setCpuFrequencyMhz(CPU_FREQ_MHZ);
    // Set internal statu LED pin
    pinMode(22, OUTPUT);
    // Turn on internal status LED
    digitalWrite(22, LOW);
    #ifdef DEBUG_SERIAL
    Serial.begin(115200);
    // Show the last restart reason
    show_last_restart_reason();
    log_i("CPU Freq: %sMhz", String(getCpuFrequencyMhz()));
    log_i("Initializing NVS Flash...");
    #endif
    // Initialize flash
    nvs_flash_init();
    // Initialize the time struct
    struct tm time;
    // Initialize the Hardware Watchdog
    #ifdef DEBUG_SERIAL
    log_i("Initializing Hardware Watchdog...");
    #endif
    // Configure the deep sleep timer
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_MIN * 60 * uS_TO_S_FACTOR);
    esp_task_wdt_init(WDT_TIMEOUT_SECS, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);                    // add current thread to WDT watch
    // Initialize the EEPROM
    #ifdef DEBUG_SERIAL
    log_i("Initializing EEPROM...");
    #endif
    EEPROM.begin(512);
    #ifdef DEBUG_SERIAL
    log_i("Initializing SPIFFS partition...");
    #endif
    // Mount SPIFFS file system
    if (!SPIFFS.begin(true))
    {
        system_problem = true;
        problem_reason = "SPIFFS mount failed";
    #ifdef DEBUG_SERIAL
        log_e("An Error has occurred while mounting the SPIFFS partition");
    #endif
    }
    else
        spiff_ready = true;
    #ifdef RESET_DATA
    SPIFFS.remove("/data.txt");
    #endif
    esp_task_wdt_reset();
    // Set ADC resolution to 12 bits (4096 levels)
    analogReadResolution(12);
    // Set ADC attenuation to 11dB for full range of 0-3.3V
    analogSetAttenuation(ADC_11db);
    // Get the last 4 digits of the mac address for the device id
    String device_id = getMacLast4().substring(getMacLast4().length() - 4);
    // Log the mac address to console
    #ifdef DEBUG_SERIAL
    log_i("Initialization Complete.");
    log_i("Device Address: %s", WiFi.macAddress().c_str());
    show_time();
    show_battery_voltage();
    #endif
    // Read iter from EEPROM 
    iter = readIntFromEEPROM(192);
    #ifdef DEBUG_SERIAL
    log_i("Sensor Iteration %d of %d before API upload", iter, UPLOAD_EVERY);
    #endif
    if (!getLocalTime(&time))
    {
        #ifdef DEBUG_SERIAL
        log_i("Time not set, setting time...");
        #endif
        time = get_time();
    }
    if (iter >= UPLOAD_EVERY || !spiff_ready)
    {
        #ifdef DEBUG_SERIAL
        log_i("API upload time!");
        #endif
        iter = 1;
        // Initialize wifi
        connect_wifi();
        check_datafile();
    }
    else
        iter++;
    writeIntToEEPROM(192, iter);
    EEPROM.commit(); // save the changes to the EEPROM
    int avg;
    // iterate through the sensors and get the average moisture value
    #ifdef DEBUG_SERIAL
    log_i("Starting sensor read loop for %d sensors", sensor_length);
    #endif
    for (int i = 0; i < sensor_length; i++)
    {
        esp_task_wdt_reset();
        // Get the average moisture value from the sensor
        avg = get_avg_moisture(sensor_pins[i]);
        // initialize memory for the timestamp
        char *ts = (char *)malloc(20);
        // Format the timestamp for the payload
        time = get_time();
        sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d", time.tm_year, time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
        String timestamp = String(ts);
        // prevent ts from memory leak
        free(ts);
        char batteryVoltage[10];
        float bv = get_battery_voltage();
        sprintf(batteryVoltage, "%.2f", bv);
        String status_bit = "0";
        int batt_pct = get_battery_pct(bv);
        if (avg == 0)
        {
            #ifdef DEBUG_SERIAL
            log_e("Sensor %d is not connected!", sensor_pins[i]);
            #endif
            if (system_problem)
                status_bit = "S";
            else
                status_bit = "D";
            // Build the json payload
            String jsonPayload = "{\"device_id\":\"" + String(device_id) + "\",\"sensor_id\":" + String(sensor_pins[i]) + ",\"soil_value\":" + String(avg) + ",\"status_bit\":\"" + String(status_bit) + "\",\"batt_volt\":\"" + String(batteryVoltage) + "\",\"batt_pct\":" + String(batt_pct) + ",\"timestamp\":\"" + timestamp.c_str() + "\",\"reason\":\"" + problem_reason + "\",\"version\":" + String(VERSION) + "}";
            // Send the payload to the API
            send_payload(jsonPayload, true);
            continue;
        }
        else
        {
            if (system_problem)
                status_bit = "S";
            else if (atof(batteryVoltage) < BATTERY_WARN_VOLTAGE)
                status_bit = "B";
            else
                status_bit = "A";
            #ifdef DEBUG_SERIAL
            log_i("Sensor:%d  Moisture value:%d  Battery:%sv(%d%%)  Status:%s  Timestamp:%s", sensor_pins[i], avg, String(batteryVoltage), batt_pct, status_bit, timestamp.c_str());
            #endif
            // Build the json payload
            String jsonPayload = "{\"device_id\":\"" + String(device_id) + "\",\"sensor_id\":" + String(sensor_pins[i]) + ",\"soil_value\":" + String(avg) + ",\"status_bit\":\"" + String(status_bit) + "\",\"batt_volt\":\"" + String(batteryVoltage) + "\",\"batt_pct\":" + batt_pct + ",\"timestamp\":\"" + timestamp.c_str() + "\",\"reason\":\"" + problem_reason + "\",\"version\":" + String(VERSION) + "}";
            // Send the payload to the API
            send_payload(jsonPayload, true);
        continue;
        }
    }
    // After all sensors are read, goto sleep until next reading
    EEPROM.end();
    #ifdef DEBUG_SERIAL
    if (system_problem)
        log_w("System problem detected: %s", problem_reason);
    log_w("Tasks complete, going to sleep for %d minutes", SLEEP_MIN);
    #endif
    // Turn off the status LED
    digitalWrite(22, HIGH);
    esp_deep_sleep_start();
}