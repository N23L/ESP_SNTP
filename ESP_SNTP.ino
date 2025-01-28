#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <DHT.h>
#include <DHT_U.h>
#include <SimpleTimer.h>
#include <time.h>
#include "esp_sntp.h"
#include <sys/time.h>

#define DEFAULT_POWER_MODE   true
#define DEFAULT_DIMMER_LEVEL 0
#define DHTPIN 14

char light_status[20];
char rain_status[20];
char water_status[20];

const char *service_name = "PROV_1234";
const char *pop = "abcd1234";


// DHT Sensor
DHT dht(DHTPIN, DHT22);

// Timers
SimpleTimer Timer;

// GPIO
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
static int gpio_0 = 9;
static int gpio_dimmer = 7;
#else
static int gpio_0 = 0;
static int gpio_dimmer = 13;
static int photores = 32;
static int rain_sensor = 26;
static int water_sensor = 35;
static int pump = 12;
#endif

int position = DEFAULT_DIMMER_LEVEL;
bool dimmer_state;
float temperature;
int analog_res;
int rain_state;
int water_state;
static int minute = 60000;

// RainMaker Devices
static Device *my_device;
static Device *my_second_device;
static Device *my_third_device;
static Device *my_fourth_device;
static Device *my_fifth_device;
static Device *my_sixth_device;

// Time synchronization function
void timeSync() {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "time.google.com");
    sntp_servermode_dhcp(true);
    sntp_set_time_sync_notification_cb([](struct timeval *tv) {
        Serial.println("Time synchronized with SNTP server.");
    });

    sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 50;

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        Serial.println("Waiting for system time to be set...");
        delay(2000);
    }

    if (retry < retry_count) {
        time(&now);
        localtime_r(&now, &timeinfo);
        Serial.printf("Time synchronized: %s", asctime(&timeinfo));
    } else {
        Serial.println("Failed to synchronize time.");
    }
}

// Event handler for Wi-Fi provisioning
void sysProvEvent(arduino_event_t *sys_event) {
    switch (sys_event->event_id) {
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32S2
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
            WiFiProv.printQR(service_name, pop, "softap");
#else
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
            WiFiProv.printQR(service_name, pop, "ble");
#endif
            break;
        case ARDUINO_EVENT_PROV_INIT:         WiFiProv.disableAutoStop(10000); break;
        case ARDUINO_EVENT_PROV_CRED_SUCCESS: WiFiProv.endProvision(); break;
        default:                              ;
    }
}

// Write callback for RainMaker device
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();
    if (strcmp(param_name, "Power") == 0) {
        Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
        dimmer_state = val.val.b;
        param->updateAndReport(val);
    } else if (strcmp(param_name, "Level") == 0) {
        Serial.printf("\nReceived value = %d for %s - %s\n", val.val.i, device_name, param_name);
        position = val.val.i;
        param->updateAndReport(val);
    }

    if (dimmer_state == LOW) {
        digitalWrite(gpio_dimmer, LOW);
    } else {
        analogWrite(gpio_dimmer, position);
    }
}

// Setup function
void setup() {
    Serial.begin(115200);

    // Manually set the timestamp (e.g., 2021-12-05 00:00:00)
    struct timeval tv;
    tv.tv_sec = 1638500000;  // Seconds since Unix epoch (e.g., 1638500000 = 2021-12-05 00:00:00)
    tv.tv_usec = 0;          // Microseconds (set to 0)

    // Set the system time using settimeofday
    settimeofday(&tv, NULL);

    // Verify the time is set correctly
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    Serial.printf("Manually set time: %s", asctime(&timeinfo));

    // GPIO initialization
    pinMode(gpio_0, INPUT);
    pinMode(gpio_dimmer, OUTPUT);
    pinMode(rain_sensor, INPUT);
    digitalWrite(gpio_dimmer, DEFAULT_POWER_MODE);

    // Initialize peripherals
    dht.begin();

    // Connect to Wi-Fi
    WiFi.begin("PLDTHOMEFIBRSGSBT", "PLDTWIFIr2CmS");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to Wi-Fi...");
    }
    Serial.println("Wi-Fi connected.");

    // Time synchronization
    timeSync();

    // Initialize RainMaker
    RMaker.setTimeSync(true);
    esp_rmaker_time_wait_for_sync(1000);

        Node my_node = RMaker.initNode("ESP RainMaker Node");

    // Create devices
    my_device = new Device("Light", "esp.device.light", &gpio_dimmer);
    my_second_device = new Device("Temperature", "esp.device.temperature-sensor");
    my_third_device = new Device("Rain", "esp.device.garage-door");
    my_fourth_device = new Device("Luminosity", "esp.device.light");
    my_fifth_device = new Device("Water Level", "esp.device.other");
    my_sixth_device = new Device("Pump", "esp.device.switch", &pump);

    my_device->addPowerParam(DEFAULT_POWER_MODE);
    my_device->assignPrimaryParam(my_device->getParamByName(ESP_RMAKER_DEF_POWER_NAME));

    Param level_param("Level", ESP_RMAKER_PARAM_SATURATION, value(DEFAULT_DIMMER_LEVEL), PROP_FLAG_READ | PROP_FLAG_WRITE);
    level_param.addBounds(value(0), value(255), value(1));
    level_param.addUIType(ESP_RMAKER_UI_SLIDER);
    my_device->addParam(level_param);

    Param temp("Temperature", ESP_RMAKER_PARAM_TEMPERATURE, value(temperature), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    my_second_device->addParam(temp);
    my_second_device->assignPrimaryParam(my_second_device->getParamByName("Temperature"));

    Param rain("Rain", ESP_RMAKER_PARAM_MODE, value(rain_status), PROP_FLAG_READ);
    my_third_device->addParam(rain);
    my_third_device->assignPrimaryParam(my_third_device->getParamByName("Rain"));   

    Param light("Luminosity", ESP_RMAKER_PARAM_MODE, value(light_status), PROP_FLAG_READ);
    my_fourth_device->addParam(light);
    my_fourth_device->assignPrimaryParam(my_fourth_device->getParamByName("Luminosity")); 

    Param water("Water Level", ESP_RMAKER_DEVICE_OTHER, value(water_status), PROP_FLAG_READ);
    my_fifth_device->addParam(water);
    my_fifth_device->assignPrimaryParam(my_fifth_device->getParamByName("Water Level"));

    my_sixth_device->addPowerParam(DEFAULT_POWER_MODE);
    my_sixth_device->assignPrimaryParam(my_sixth_device->getParamByName(ESP_RMAKER_DEF_POWER_NAME));


    my_device->addCb(write_callback);

    // Add devices to the node
    my_node.addDevice(*my_device);
    my_node.addDevice(*my_second_device);
    my_node.addDevice(*my_third_device);
    my_node.addDevice(*my_fourth_device);
    my_node.addDevice(*my_fifth_device);
    my_node.addDevice(*my_sixth_device);

    RMaker.setTimeZone("Asia/Manila");
    RMaker.enableSchedule();
    RMaker.enableScenes();
    RMaker.start();

    Timer.setInterval(1*minute);

    WiFi.onEvent(sysProvEvent);
}

// Loop function
void loop() {
    if (Timer.isReady() && esp_rmaker_time_check()) {
        Serial.println("Sending Sensor's Data");
        Send_Sensor();
        lightstat();
        rainstat();
        Timer.reset();
    }

    if (digitalRead(gpio_0) == LOW) {
        delay(100);
        int startTime = millis();
        while (digitalRead(gpio_0) == LOW) {
            delay(50);
        }
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
            Serial.printf("Reset to factory.\n");
            RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
            Serial.printf("Reset Wi-Fi.\n");
            RMakerWiFiReset(2);
        }
    }
    delay(100);
}

// Send sensor data to RainMaker
void Send_Sensor() {
    temperature = dht.readTemperature();
    my_second_device->updateAndReportParam("Temperature", temperature);
}

void lightstat() {
  analog_res = analogRead(photores);
  Serial.println(analog_res);
  if (analog_res <= 1365) {
    strcpy(light_status, "High");
  } else if (analog_res <= 2730) {
    strcpy(light_status, "Mid");
  } else {
    strcpy(light_status, "Low");
  }

  my_fourth_device->updateAndReportParam("Luminosity", light_status); 
}

void rainstat() {
  rain_state = digitalRead(rain_sensor);
  Serial.println(rain_state);
  if (rain_state == HIGH) {
    strcpy(rain_status, "Dry");
  } else {
    strcpy(rain_status, "Wet");
  }

  my_third_device->updateAndReportParam("Rain", rain_status); 
}

void waterstat() {
  water_state = analogRead(water_sensor);
  Serial.println(water_state);
  if (water_state <= 1365) {
    strcpy(water_status, "High");
  } else if (water_state <= 2730) {
    strcpy(water_status, "Mid");
  } else {
    strcpy(water_status, "Low");
  }

  my_fifth_device->updateAndReportParam("Water Level", water_status); 
}
