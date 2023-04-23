// Growbot Remote ESP32 Plant Sensor
//
// Created by: Ian Perry (ianperry99@gmail.com)
// https://github.com/cosmicc/growbot_remote_module

// ref:  2860 open air, 2230 dry, 1000 submerged in water,

#include <EEPROM.h>
#include <Arduino.h>
#include <nvs_flash.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <SPIFFS.h>

// compile definitions
#undef RESET_DATA               // reset datafile on boot
#define SLEEP_MIN 1              // Sleep time in minutes
#define UPLOAD_EVERY 5           // Upload data every x sleep cycles
#define MOISTURE_WARN_PCT 20     // Warning moisture percentage
#define BATTERY_WARN_VOLTAGE 3.6 // Warning battery voltage
#define BATTERY_MIN_VOLTAGE 3.1  // Minimum battery voltage to operate
#define BATTERY_MAX_VOLTAGE 4.2  // Maximum battery voltage for percentage calulation
#define SENSOR_SAMPLES 20        // Number of sensor samples to take for avg
#define SENSOR_HI 2860           // Highest sensor reading
#define SENSOR_LOW 1000          // Lowest sensor reading
#define SENSOR_DELAY_MS 10       // Delay between sensor reads in milliseconds
#define WIFI_TIMEOUT_SECS 30     // Wifi connection timeout in seconds
#define NTP_TIMEOUT_SECS 10      // NTP server timeout in seconds
#define WDT_TIMEOUT_SECS 10      // watchdog timer timeout in seconds
#define API_SEND_DELAY_MS 100    // Delay between API calls in milliseconds
#define uS_TO_S_FACTOR 1000000   // Conversion factor for micro seconds to seconds
#define BATTERY_PIN 39           // Analog input pin to read battery voltage  

// Only GPIO 32-36 are ADC1 channels and can only be used for soil_sensors
// GPIO 39 is ADC1 also but is used for battery monitoring
// ADC2 channels cannot be used because they are used by the wifi interference
int sensor_pins[] = {36}; // list of each soil sensor pin this device will monitor

// Global variables
int tz_offset_min = -300; // Timezone offset in seconds (-5 hours)
int dst_offset_min = 60;  // daylight savings time offset in minutes
int httpResponseCode;
int sensor_length = sizeof(sensor_pins) / sizeof(sensor_pins[0]);
bool spiff_ready = false;
bool http_success_bit = true;
bool system_problem = false;
String problem_reason;
char *api_url;
char *ntp_server;
int iter;

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
void show_time();
void show_battery_voltage();
float get_battery_voltage();
int get_battery_pct(float volt);

// display the reason for the last restart
void show_last_restart_reason()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    switch (reset_reason)
    {
    case ESP_RST_UNKNOWN:
        log_w("Last reset reason: unknown");
        problem_reason = "Last reset: unknown";
        system_problem = true;
        break;
    case ESP_RST_POWERON:
        log_i("Last reset reason: power-on reset");
        break;
    case ESP_RST_EXT:
        log_w("Last reset reason: external reset");
        problem_reason = "Last reset: external reset";
        system_problem = true;
        break;
    case ESP_RST_SW:
        log_w("Last reset reason: software reset");
        problem_reason = "Last reset: software reset";
        system_problem = true;
        break;
    case ESP_RST_PANIC:
        log_e("Last reset reason: panic reset");
        problem_reason = "Last reset: panic reset";
        system_problem = true;
        break;
    case ESP_RST_INT_WDT:
        log_e("Last reset reason: interrupt watchdog reset");
        problem_reason = "Last reset: interrupt watchdog reset";
        system_problem = true;
        break;
    case ESP_RST_TASK_WDT:
        log_e("Last reset reason: task watchdog reset");
        problem_reason = "Last reset: task watchdog reset";
        system_problem = true;
        break;
    case ESP_RST_WDT:
        log_e("Last reset reason: other watchdog reset");
        problem_reason = "Last reset: other watchdog reset";
        system_problem = true;
        break;
    case ESP_RST_DEEPSLEEP:
        log_i("Last reset reason: deep sleep reset");
        break;
    case ESP_RST_BROWNOUT:
        log_e("Last reset reason: brownout reset");
        problem_reason = "Last reset: brownout reset";
        system_problem = true;
        break;
    case ESP_RST_SDIO:
        log_w("Last reset reason: SDIO reset");
        problem_reason = "Last reset: SDIO reset";
        system_problem = true;
        break;
    default:
        log_w("Last reset reason: unknown");
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
    EEPROM.commit();        // save the changes to the EEPROM
}

// read integer from EEPROM
int readIntFromEEPROM(int addrOffset)
{
    int value = 0;          // initialize variable to store the value read from EEPROM
    EEPROM.get(addrOffset, value); // read the integer value from address 192
    return value;           // return the integer value
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

// Send json sensor data to API
void send_payload(String jsonPayload, bool writespiff)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient httpClient;
        httpClient.begin(api_url); // Replace with your API endpoint
        httpClient.addHeader("Content-Type", "application/json");
        httpClient.addHeader("Accept", "application/json");
        esp_task_wdt_reset();
        log_i("Sending Payload: %s", jsonPayload.c_str());
        //httpResponseCode = httpClient.POST(encryptPayload(jsonPayload));
        httpResponseCode = httpClient.POST(jsonPayload);
        esp_task_wdt_reset();
        String response = httpClient.getString();
        log_i("HTTP Response Code: %s", String(httpResponseCode));
        log_i("HTTP Response: %s", response.c_str());
        httpClient.end();
        if (httpResponseCode == 200)
        {
            log_i("Sensor data sent to API successfully");
            http_success_bit = true;
        }
        else
        {
            log_e("Failed to send sensor data to API");
            http_success_bit = false;
            if (writespiff)
            {
                log_w("Saving sensor data to SPIFFS because API is unreachable");
                write_spiff(jsonPayload);
            }
        }
    }
    else
    {
        if (writespiff)
        {
            if (iter != UPLOAD_EVERY)
                log_i("Saving sensor data to SPIFFS because its not time to upload yet");
            else
                log_w("Saving sensor data to SPIFFS because wifi is down");
            write_spiff(jsonPayload);
        }
    }
}

// Check if there is any sensor data stored on the spiff partition
void check_datafile()
{
    // Check if the data file exists on the spiff partition
    if (!SPIFFS.exists("/data.txt"))
    {
        log_i("No archived sensor data was found on SPIFFS");
    }
    else
    {
        File data_file = SPIFFS.open("/data.txt", FILE_READ);
        if (!data_file)
        {
            log_i("No archived sensor data was found on SPIFFS");
        }
        else
        {
            log_i("Processing archived sensor data entries found on SPIFFS");
            File data_file = SPIFFS.open("/data.txt", FILE_READ);
            while (data_file.available() && http_success_bit)
            {
                esp_task_wdt_reset();
                String line = data_file.readStringUntil('\n');
                send_payload(line, false);
                delay(API_SEND_DELAY_MS);
            }
            data_file.close();
            if (http_success_bit)
            {
                // Delete the file
                log_i("Removing completed data file from SPIFFS");
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
        log_i("Data saved to SPIFFS successfully");
    }
    else
    {
        log_e("SPIFFS not ready, cannot save data");
    }
}

// Get the average moisture value from the sensor
int get_avg_moisture(int aout_pin)
{
    if (aout_pin < 32 || aout_pin > 39)
    {
        log_e("Invalid sensor pin [%d]", aout_pin);
        return 0;
    }
    int total = 0;
    int avg = 0;
    log_d("Reading sensor [GPIO%d] for [%d]x[%dms] samples...", aout_pin, SENSOR_SAMPLES, SENSOR_DELAY_MS);
    int start_time = millis();
    for (int i = 0; i < SENSOR_SAMPLES; i++)
    {
        int value = analogRead(aout_pin); // read the analog value from sensor
        total += value;
        esp_task_wdt_reset();
        delay(SENSOR_DELAY_MS);
    }
    avg = total / SENSOR_SAMPLES;
    log_d("Sensor [%d] read time for [%d]x[%dms] samples: [%dms]", aout_pin, SENSOR_SAMPLES, SENSOR_DELAY_MS, (millis() - start_time));
    return avg;
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
        log_e("Could not obtain time info from RTC, trying NTP sync");
        // Initialize wifi
        char *ssid = readStringFromEEPROM(0);
        char *password = readStringFromEEPROM(48);
        if (WiFi.status() != WL_CONNECTED)
        {
            WiFi.begin(ssid, password);
            log_i("Connecting to WiFi...");
            for (int i = 0; i < WIFI_TIMEOUT_SECS*10; i++)
            {
                if (WiFi.status() == WL_CONNECTED)
                {
                    log_i("Connected to WiFi");
                    break;
                }
                esp_task_wdt_reset();
                delay(100);
            }
            if (WiFi.status() != WL_CONNECTED)
            {
                log_e("Failed to connect to WiFi");
                time.tm_year += 1900;
                time.tm_mon += 1;
                problem_reason = "Failed to get any time";
                system_problem = true;
                return time;
            }
        }
        // Sync time with NTP server
        log_i("Syncing time with NTP server");
        configTime(tz_offset_min*60, dst_offset_min*60, ntp_server);
        // Log the current time to console
        show_time();
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
        log_e("Battery voltage is critical! [%0.1fv] (%d%%) Sleeping indefinately", batteryVoltage, pct);
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, HIGH);
        esp_deep_sleep_start();
    }
    else if (batteryVoltage < 0.5)
    {
        log_e("Battery is disconnected [%0.1fv] (%d%%)", batteryVoltage, pct);
    }
    else if (batteryVoltage < BATTERY_WARN_VOLTAGE)
    {
        log_w("Battery voltage is LOW! [%0.1fv] (%d%%)", batteryVoltage, pct);
    }
    else
    {
        log_i("Battery voltage is normal [%0.1fv] (%d%%)", batteryVoltage, pct);
    }

}

float get_battery_voltage()
{
    int adcReading = analogRead(BATTERY_PIN);            // Read ADC value
    float batteryVoltage = (adcReading / 4095.0) * 3.3; // Convert ADC reading to voltage
    log_d("adcReading: [%d] Battery voltage: [%0.1fv]", adcReading, batteryVoltage);
    return batteryVoltage;
}

int get_battery_pct(float volt = 0.0)
{
    float voltage;
    if (volt == 0.0)
        voltage = get_battery_voltage();              // Read ADC value
    else
        voltage = volt;
    int pct = map(voltage, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100);
    if (pct < 0)
        pct = 0;
    else if (pct > 100)
        pct = 100;
    return pct;
}

extern "C" void app_main()
{
    Serial.begin(115200);
    // Initialize flash
    nvs_flash_init();
    // Initialize the time struct
    struct tm time;
    // Configure the deep sleep timer
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_MIN * 60 * uS_TO_S_FACTOR);
    // Show the last restart reason
    show_last_restart_reason();
    // Initialize the Hardware Watchdog
    log_d("Initializing Hardware Watchdog...");
    esp_task_wdt_init(WDT_TIMEOUT_SECS, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);                    // add current thread to WDT watch
    // Initialize the EEPROM
    log_d("Initializing EEPROM...");
    EEPROM.begin(512);
    log_d("Initializing SPIFFS partition...");
    // Mount SPIFFS file system
    if (!SPIFFS.begin(true))
        {
        system_problem = true;
        problem_reason = "SPIFFS mount failed";
        log_e("An Error has occurred while mounting the SPIFFS partition");
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
    log_i("Device Address: %s", WiFi.macAddress().c_str());
    // Read saved data from EEPROM
    char *ssid = readStringFromEEPROM(0);
    char *password = readStringFromEEPROM(48);
    api_url = readStringFromEEPROM(96);
    ntp_server = readStringFromEEPROM(144);
    iter = readIntFromEEPROM(192);
    log_i("Using NTP server: %s", ntp_server);
    log_i("Using API URL: %s", api_url);
    log_i("Sensor Iteration %d of %d before API upload", iter, UPLOAD_EVERY);
    if (iter >= UPLOAD_EVERY || !getLocalTime(&time) || !spiff_ready)
    {
        log_i("API upload time!");
        iter = 1;
        // Initialize wifi
        WiFi.begin(ssid, password);
        log_i("Connecting to [%s] WiFi...", String(ssid));
        for (int i = 0; i < WIFI_TIMEOUT_SECS*10; i++)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                log_i("Connected to [%s] WiFi", String(ssid));
                break;
            }
            esp_task_wdt_reset();
            delay(100);
        }
        if (WiFi.status() != WL_CONNECTED)
            log_e("Failed to connect to [%s] WiFi", String(ssid));
        else
        {
            // Sync time with NTP server
            log_i("Syncing time with NTP server [%s]", ntp_server);
            configTime(tz_offset_min*60, dst_offset_min*60, ntp_server);
            // Log the current time to console
            show_time();
            // Check if the data file exists on the spiff partition and process archived sensor data
            check_datafile();
        }
    }
    else
        iter++;
    writeIntToEEPROM(192, iter);
    EEPROM.commit(); // save the changes to the EEPROM
    show_time();
    show_battery_voltage();
    int avg;
    int pct;
    // iterate through the sensors and get the average moisture value
    log_i("Starting sensor read loop for %d sensors", sensor_length);
    for (int i = 0; i < sensor_length; i++)
    {
        esp_task_wdt_reset();
        // Get the average moisture value from the sensor
        avg = get_avg_moisture(sensor_pins[i]);
        // get the current time
        struct tm time = get_time();
        // initialize memory for the timestamp
        char *ts = (char *)malloc(20);
        // Format the timestamp for the payload
        sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d", time.tm_year, time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
        String timestamp = String(ts);
        // prevent ts from memory leak
        free(ts);
        char batteryVoltage[10];
        float bv = get_battery_voltage();
        sprintf(batteryVoltage, "%.1f", bv);
        String status_bit = "0";
        int batt_pct = get_battery_pct(bv);
        if (avg == 0)
        {
            log_e("Sensor %d is not connected!", sensor_pins[i]);
            if (system_problem)
                status_bit = "S";
            else    
                status_bit = "D";
            // Build the json payload
            String jsonPayload = "{\"device_id\":\"" + String(device_id) + "\",\"sensor_id\":" + String(sensor_pins[i]) + ",\"soil_value\":" + String(avg) + ",\"soil_pct\":0,\"status_bit\":\"" + String(status_bit) + "\",\"batt_volt\":\"" + String(batteryVoltage) + "\",\"batt_pct\":" + String(batt_pct) + ",\"timestamp\":\"" + timestamp.c_str() + "\",\"reason\":\"" + problem_reason + "\"}";
            // Send the payload to the API
            send_payload(jsonPayload, true);
            continue;
        }
        else
        {
            // Calculate the moisture percentage
            pct = map(avg, SENSOR_HI, SENSOR_LOW, 0, 100);
            if (system_problem)
                status_bit = "S";
            else if (atof(batteryVoltage) < BATTERY_WARN_VOLTAGE)
                status_bit = "B";
            else if (pct < MOISTURE_WARN_PCT)
                status_bit = "M";
            else
                status_bit = "A";
            log_i("Sensor:%d  Moisture value:%d  Moisture percentage:%d%%  Battery:%sv(%d%%)  Status:%s  Timestamp:%s", sensor_pins[i], avg, pct, String(batteryVoltage), batt_pct, status_bit, timestamp.c_str());
            // Build the json payload
            String jsonPayload = "{\"device_id\":\"" + String(device_id) + "\",\"sensor_id\":" + String(sensor_pins[i]) + ",\"soil_value\":" + String(avg) + ",\"soil_pct\":" + String(pct) + ",\"status_bit\":\"" + String(status_bit) + "\",\"batt_volt\":\"" + String(batteryVoltage) + "\",\"batt_pct\":" + batt_pct + ",\"timestamp\":\"" + timestamp.c_str() + "\",\"reason\":\"" + problem_reason + "\"}";
            // Send the payload to the API
            send_payload(jsonPayload, true);
            continue;
        }
    }
    // After all sensors are read, goto sleep until next reading
    EEPROM.end();
    if (system_problem)
        log_w("System problem detected: %s", problem_reason);
    log_w("Tasks complete, going to sleep for %d minutes", SLEEP_MIN);
    esp_deep_sleep_start();
}