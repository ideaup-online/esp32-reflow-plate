#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MAX31855.h>
#include <LittleFS.h>

#include "config.hpp"
#include "data.hpp"

struct ReflowCurvePoint
{
    int time_ms;
    int temp_c;
};
ReflowCurvePoint chipQuikCurve[] = {
    {0, 25},
    {90000, 90},
    {180000, 130},
    {210000, 138},
    {240000, 165},
    {270000, 138},
    {-1, -1},
};
volatile bool startReflowCurve = false;
volatile bool cancelReflowCurve = false;
bool reflowCurveRunning = false;
unsigned long reflowStartMillis = 0;

// Pin definitions
#define TC_DO_PIN 19
#define TC_CLK_PIN 18
#define TC1_CS_PIN 4
#define TC2_CS_PIN 33

#define FET_PIN 27

#define BTN_PIN 0

#define SDA_PIN 32
#define SCL_PIN 25

// i2c address of ADC
#define ADC_ADDR 0x36

// i2c address of OLED
// and OLED screen size
#define OLED_ADDR 0x3c
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Reads config information from
// a JSON file stored in flash
// (LittleFS)
Config config;

// Object to hold all sensor data
Data data;

// Mutex to serialize access
// to the i2c bus
SemaphoreHandle_t i2cMutex;

// Objects to communicate with
// thermocouple amplifiers
// (MAX31855) via SPI
Adafruit_MAX31855 thermocouple1(TC_CLK_PIN, TC1_CS_PIN, TC_DO_PIN);
Adafruit_MAX31855 thermocouple2(TC_CLK_PIN, TC2_CS_PIN, TC_DO_PIN);

// The delay between each iteration
// of the loop() function. This
// is the sample frequency of the
// PID controller (ms)
const int loopDelay = 100;

// Thermocouple reader task
TaskHandle_t tcTaskHandle;
const int tcNumSamplesToAvg = 4;
const int tcDelay = loopDelay / tcNumSamplesToAvg;

// LMT85 reader task
TaskHandle_t lmt85TaskHandle;
const int lmt85NumSamplesToAvg = 4;
const int lmt85Delay = loopDelay / lmt85NumSamplesToAvg;

// OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
TaskHandle_t updateDisplayTaskHandle;
int displayRefreshPeriod = 500;

// GPIO0 button
esp_timer_handle_t btnTimer;
esp_timer_create_args_t btnTimerArgs;
const int debounceTime_us = 25000;
const int lowTemp = 0;
const int highTemp = 150;

// PID controller
volatile double pendingSetpoint = lowTemp;
double Kp = 500.0;
double Ki = 0.625;
double Kd = 1.0;
double pidInput = 0.0;
double pidOutput = 0.0;
double pidSetpoint = 0.0;
PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// setting PWM properties
const int freq = 15;
const int ledChannel = 0;
const int resolution = 12;

// CSV server
const int csvServerPort = 2112;
TaskHandle_t csvServerTaskHandle;
const int csvReportingDelay = loopDelay;
struct Connection
{
    WiFiClient client;
    unsigned long zeroMillis;
};

// Prototypes
double c2f(double celsius);
void readThermocouples(void *);
void readLMT85(void *);
void updateDisplay(void *);
void csvServer(void *);
void IRAM_ATTR btnHandler();
void IRAM_ATTR btnDebounce(void *);
double getLMT85Temp(int lmt85_mV);

void setup()
{
    Serial.begin(115200);

    // Give the serial monitor a couple of
    // seconds to start up
    delay(2000);

    Serial.println("Solder Reflow Plate Controller V1.0");

    // Set up heater pin and ensure
    // heater is off to start
    Serial.printf("Initializing heater to off...");
    pinMode(FET_PIN, OUTPUT);
    digitalWrite(FET_PIN, LOW);
    if (ledcSetup(ledChannel, freq, resolution) == 0)
    {
        Serial.printf("ledcSetup failed!");
        while (true)
        {
            delay(10);
        }
    }
    ledcAttachPin(FET_PIN, ledChannel);
    ledcWrite(ledChannel, pidOutput);
    Serial.printf("done.\n");

    // Start LittleFS
    Serial.printf("Initializing LittleFS...");
    if (!LittleFS.begin())
    {
        Serial.printf("failed\n");
        while (true)
        {
            delay(10);
        }
    }
    else
    {
        Serial.printf("done.\n");
    }

    // Open config file for reading
    Serial.printf("Opening config file...");
    File configFile = LittleFS.open("/config.json", "r");
    if (!configFile)
    {
        Serial.printf("failed\n");
        while (true)
        {
            delay(10);
        }
    }
    else
    {
        Serial.printf("done.\n");
    }

    // Read config file into config object
    Serial.printf("Reading config file...");
    if (!config.readConfig(configFile))
    {
        Serial.printf("failed\n");
        while (true)
        {
            delay(10);
        }
    }
    else
    {
        configFile.close();
        Serial.printf("done.\n");
    }

    WiFi.begin(config.getSSID(), config.getKey());

    Serial.printf("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
    }
    Serial.printf("done.\n");

    IPAddress localAddr = WiFi.localIP();
    Serial.printf("IP: %s\n", localAddr.toString().c_str());

    if (!MDNS.begin(config.getMDNS()))
    {
        Serial.println("Error setting up mDNS responder");
    }
    else
    {
        Serial.printf("mDNS: %s\n", config.getMDNS());
    }

    Serial.printf("Initializing thermocouple 1...");
    if (!thermocouple1.begin())
    {
        Serial.println("ERROR.");
        while (true)
        {
            delay(10);
        }
    }
    Serial.printf("done.\n");

    Serial.printf("Initializing thermocouple 2...");
    if (!thermocouple2.begin())
    {
        Serial.println("ERROR.");
        while (true)
        {
            delay(10);
        }
    }
    Serial.printf("done.\n");

    // Start thermocouple reader task
    if (xTaskCreate(readThermocouples,
                    "Read TCs",
                    1024,
                    0,
                    1,
                    &tcTaskHandle) == pdPASS)
    {
        Serial.println("Thermocouple task started");
    }
    else
    {
        Serial.println("Failed to start thermocouple task");
        while (true)
        {
            delay(10);
        }
    }

    Serial.printf("Initializing OLED...");
    Wire.setPins(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("SSD1306 allocation failed");
        while (1)
            delay(10);
    }
    Serial.printf("done.\n");

    // Set up ADC (MAX11645)
    Serial.printf("Initializing external ADC...");
    Wire.begin();
    Wire.beginTransmission((uint16_t)ADC_ADDR);
    Wire.write((uint8_t)0b10100000);
    Wire.write((uint8_t)0b01100001);
    Wire.endTransmission();
    Serial.printf("done.\n");

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();

    // Create the i2c mutex
    i2cMutex = xSemaphoreCreateMutex();

    // Start lmt85 reader task
    if (xTaskCreate(readLMT85,
                    "Read LMT85",
                    2048,
                    &i2cMutex,
                    1,
                    &lmt85TaskHandle) == pdPASS)
    {
        Serial.println("LMT85 reader task started");
    }
    else
    {
        Serial.println("Failed to start LMT85 reader task");
        while (true)
        {
            delay(10);
        }
    }

    // Start display update task
    if (xTaskCreate(updateDisplay,
                    "Display Update",
                    4096,
                    &i2cMutex,
                    1,
                    &updateDisplayTaskHandle) == pdPASS)
    {
        Serial.println("display update task started");
    }
    else
    {
        Serial.println("Failed to start display update task");
        while (true)
        {
            delay(10);
        }
    }

    // Set up on board GPIO0 button
    // Note that its interrupt will
    // be attached in btnDebounce()
    // after a 2 second timer
    // expires
    pinMode(BTN_PIN, INPUT);
    btnTimerArgs.callback = btnDebounce;
    if (esp_timer_create(&btnTimerArgs, &btnTimer) != ESP_OK)
    {
        Serial.println("Failed to create esp timer for button debouncing");
        while (true)
        {
            delay(10);
        }
    }
    esp_timer_start_once(btnTimer, 2000);

    // Start CSV server task
    if (xTaskCreate(csvServer,
                    "CSV Server",
                    4096,
                    &i2cMutex,
                    1,
                    &csvServerTaskHandle) == pdPASS)
    {
        Serial.println("CSV server task started");
    }
    else
    {
        Serial.println("Failed to start CSV server task");
        while (true)
        {
            delay(10);
        }
    }

    // Set PID output limits based on
    // PWM resolution, sample time based on
    // loop delay time, and automatic mode
    int maxForResolution = (1 << resolution) - 1;
    Serial.printf("resolution: %d maxLimit: %d\n", resolution, maxForResolution);
    pid.SetOutputLimits(0, maxForResolution);
    pid.SetSampleTime(loopDelay);
    pid.SetMode(AUTOMATIC);
}

void loop()
{
    if (reflowCurveRunning)
    {
        if (cancelReflowCurve)
        {
            cancelReflowCurve = false;
            reflowCurveRunning = false;
            data.setSetpoint(0.0);
            Serial.println("Canceling reflow curve");
        }
        else
        {
            unsigned long curveTime = millis() - reflowStartMillis;
            double newSetpoint = 0.0;
            for (int i = 0; chipQuikCurve[i].time_ms != -1; i++)
            {
                if (curveTime < chipQuikCurve[i].time_ms)
                {
                    if (i > 0)
                    {
                        double timePct = (double)(curveTime - chipQuikCurve[i - 1].time_ms) /
                                         (double)(chipQuikCurve[i].time_ms - chipQuikCurve[i - 1].time_ms);
                        newSetpoint = chipQuikCurve[i - 1].temp_c +
                                      ((chipQuikCurve[i].temp_c - chipQuikCurve[i - 1].temp_c) * timePct);
                        break;
                    }
                }
            }
            data.setSetpoint(newSetpoint);
            if (newSetpoint == 0.0)
            {
                reflowCurveRunning = false;
                Serial.println("Reflow curve completed");
            }
        }
    }
    else
    {
        if (startReflowCurve)
        {
            startReflowCurve = false;
            reflowCurveRunning = true;
            reflowStartMillis = millis();
            Serial.println("Starting reflow curve");
        }
    }

    // Compute output power based on TC1
    // input and apply it
    pidInput = data.getTc1Temp();
    pidSetpoint = data.getSetpoint();
    pid.Compute();
    ledcWrite(ledChannel, pidOutput);

    delay(loopDelay);
}

double c2f(double celsius)
{
    return (celsius * (9.0 / 5.0)) + 32;
}

void readThermocouples(void *)
{
    double c;
    double tc1Samples[tcNumSamplesToAvg];
    double tc2Samples[tcNumSamplesToAvg];
    int sampleIdx = 0;
    int count = 0;

    while (true)
    {
        // Read TC1 input
        c = thermocouple1.readCelsius();
        if (isnan(c))
        {
            Serial.println("Thermocouple 1 fault(s) detected!");
            uint8_t e = thermocouple1.readError();
            if (e & MAX31855_FAULT_OPEN)
            {
                Serial.println("FAULT: Thermocouple is open - no connections.");
            }
            if (e & MAX31855_FAULT_SHORT_GND)
            {
                Serial.println("FAULT: Thermocouple is short-circuited to GND.");
            }
            if (e & MAX31855_FAULT_SHORT_VCC)
            {
                Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
            }
        }
        else
        {
            tc1Samples[sampleIdx] = c;

            if (count == tcNumSamplesToAvg)
            {
                double sum = 0.0;
                for (int i = 0; i < tcNumSamplesToAvg; i++)
                {
                    sum += tc1Samples[i];
                }

                double avg = sum / tcNumSamplesToAvg;
                data.setTc1Temp(avg);
            }
        }

        // Read TC2 input
        c = thermocouple2.readCelsius();
        if (isnan(c))
        {
            Serial.println("Thermocouple 2 fault(s) detected!");
            uint8_t e = thermocouple1.readError();
            if (e & MAX31855_FAULT_OPEN)
            {
                Serial.println("FAULT: Thermocouple is open - no connections.");
            }
            if (e & MAX31855_FAULT_SHORT_GND)
            {
                Serial.println("FAULT: Thermocouple is short-circuited to GND.");
            }
            if (e & MAX31855_FAULT_SHORT_VCC)
            {
                Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
            }
        }
        else
        {
            tc2Samples[sampleIdx] = c;

            if (count == tcNumSamplesToAvg)
            {
                double sum = 0.0;
                for (int i = 0; i < tcNumSamplesToAvg; i++)
                {
                    sum += tc2Samples[i];
                }

                double avg = sum / tcNumSamplesToAvg;
                data.setTc2Temp(avg);
            }
        }

        // Update count until we have
        // enough samples to start averaging
        if (count < tcNumSamplesToAvg)
        {
            count++;
        }

        // Update sampleIdx
        sampleIdx = (sampleIdx + 1) % tcNumSamplesToAvg;

        // Wait for next sample interval
        vTaskDelay(tcDelay / portTICK_PERIOD_MS);
    }
}

void readLMT85(void *)
{
    double c;
    double samples[lmt85NumSamplesToAvg];
    int sampleIdx = 0;
    int count = 0;

    while (true)
    {
        // Read the value of the LMT85
        uint16_t lmt85Counts = 0;

        // Take the mutex
        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        // Request 2 bytes from the external ADC
        uint8_t bytesReceived = Wire.requestFrom(ADC_ADDR, 2);
        if (bytesReceived == 2)
        {
            uint8_t tmp[2];
            Wire.readBytes(tmp, 2);
            lmt85Counts = ((tmp[0] << 8) | tmp[1]) & 0x0fff;
        }

        // Give the mutex back
        xSemaphoreGive(i2cMutex);

        // Voltage reference is 2.048V, ADC is 12-bit
        // (4096 counts); thus, each count represents
        // 0.5mV
        int tmp = lmt85Counts / 2;
        samples[sampleIdx] = tmp;

        if (count == lmt85NumSamplesToAvg)
        {
            int sum = 0.0;
            for (int i = 0; i < tcNumSamplesToAvg; i++)
            {
                sum += samples[i];
            }

            int avg = sum / lmt85NumSamplesToAvg;
            data.setLmt85_mV(avg);
        }

        // Update count until we have
        // enough samples to start averaging
        if (count < lmt85NumSamplesToAvg)
        {
            count++;
        }

        // Update sampleIdx
        sampleIdx = (sampleIdx + 1) % lmt85NumSamplesToAvg;

        // Wait for next sample interval
        vTaskDelay(lmt85Delay / portTICK_PERIOD_MS);
    }
}

void updateDisplay(void *)
{
    const int tc1TempX = 0;
    const int tc1TempY = 1;
    const int tc1TempWidth = SCREEN_WIDTH;
    const int tc1TempHeight = 8;
    const int tc2TempX = 0;
    const int tc2TempY = 11;
    const int tc2TempWidth = SCREEN_WIDTH;
    const int tc2TempHeight = 8;
    const int lmt85X = 0;
    const int lmt85Y = 21;
    const int lmt85Width = SCREEN_WIDTH;
    const int lmt85Height = 8;
    const int setpointX = 0;
    const int setpointY = 31;
    const int setpointWidth = SCREEN_WIDTH;
    const int setpointHeight = 8;

    double currentTc1TempC = -1.0;
    double currentTc2TempC = -1.0;
    int currentLmt85_mV = -1;
    double currentSetpoint = -1.0;
    bool displayNeedsRefresh = false;

    while (true)
    {
        // TC1
        double tmp = data.getTc1Temp();
        if (tmp != currentTc1TempC)
        {
            currentTc1TempC = tmp;

            display.fillRect(tc1TempX, tc1TempY, tc1TempWidth, tc1TempHeight, SSD1306_BLACK);
            display.setCursor(tc1TempX, tc1TempY);
            display.printf("T1: %6.2f C %6.2f F", currentTc1TempC, c2f(currentTc1TempC));

            displayNeedsRefresh = true;
        }

        // TC2
        tmp = data.getTc2Temp();
        if (tmp != currentTc2TempC)
        {
            currentTc2TempC = tmp;

            display.fillRect(tc2TempX, tc2TempY, tc2TempWidth, tc2TempHeight, SSD1306_BLACK);
            display.setCursor(tc2TempX, tc2TempY);
            display.printf("T2: %6.2f C %6.2f F", currentTc2TempC, c2f(currentTc2TempC));

            displayNeedsRefresh = true;
        }

        // LMT85
        int tmp2 = data.getLmt85_mV();
        if (tmp2 != currentLmt85_mV)
        {
            currentLmt85_mV = tmp2;

            // Look up temp for this voltage
            double c = getLMT85Temp(currentLmt85_mV);

            display.fillRect(lmt85X, lmt85Y, lmt85Width, lmt85Height, SSD1306_BLACK);
            display.setCursor(lmt85X, lmt85Y);
            display.printf("LM: %6.2f C %6.2f F", c, c2f(c));

            displayNeedsRefresh = true;
        }

        // Set point
        tmp = data.getSetpoint();
        if (tmp != currentSetpoint)
        {
            currentSetpoint = tmp;

            display.fillRect(setpointX, setpointY, setpointWidth, setpointHeight, SSD1306_BLACK);
            display.setCursor(setpointX, setpointY);
            display.printf("SP: %6.2f C %6.2f F", currentSetpoint, c2f(currentSetpoint));

            displayNeedsRefresh = true;
        }

        // Actually update the display if anything
        // has changed
        if (displayNeedsRefresh)
        {
            displayNeedsRefresh = false;

            // Take the semaphore
            xSemaphoreTake(i2cMutex, portMAX_DELAY);

            // Send updates to display via i2c
            display.display();

            // Give the mutex back
            xSemaphoreGive(i2cMutex);
        }

        // Wait for the next refresh interval
        vTaskDelay(displayRefreshPeriod / portTICK_PERIOD_MS);
    }
}

void csvServer(void *)
{
    // Accept up to 10 connections
    const int maxConns = 10;
    Connection conns[maxConns];

    // Create a server object
    WiFiServer server(csvServerPort);

    // Start server
    server.begin();

    while (true)
    {
        unsigned long loopStart = millis();

        // Handle new connections
        while (server.hasClient())
        {
            bool connectionAccepted = false;
            for (int i = 0; i < maxConns; i++)
            {
                if (!conns[i].client.connected())
                {
                    // Accept connection
                    conns[i].client = server.available();
                    conns[i].zeroMillis = loopStart;

                    // Send CSV headers
                    conns[i].client.printf("Time,\"Set Point\",\"Under Heater\",\"Target Board\",\"Built-In Temp\",\"PID Output\",\"Kp=%0.2f Ki=%0.2f Kd=%0.2f\"\n",
                                           Kp, Ki, Kd);

                    // Set flag
                    connectionAccepted = true;
                }
            }

            if (!connectionAccepted)
            {
                // No slots left; reject
                // connection
                server.available().stop();
            }
        }

        // Send CSV data to connected
        // clients
        for (int i = 0; i < maxConns; i++)
        {
            if (conns[i].client.connected())
            {
                unsigned long reportTime = loopStart - conns[i].zeroMillis;
                conns[i].client.printf("%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",
                                       (double)reportTime / 1000.0,
                                       data.getSetpoint(),
                                       data.getTc1Temp(),
                                       data.getTc2Temp(),
                                       getLMT85Temp(data.getLmt85_mV()),
                                       pidOutput / 40.95);
            }
        }

        // Wait for next reporting interval
        int delay = csvReportingDelay - (millis() - loopStart);
        if (delay > 0)
        {
            vTaskDelay(delay / portTICK_PERIOD_MS);
        }
    }
}

void IRAM_ATTR btnHandler()
{
    // Software switch debounce:
    // detach interrupt and start a
    // timer to re-attach it after
    // debounceTime_us microseconds
    detachInterrupt(BTN_PIN);
    esp_timer_start_once(btnTimer, debounceTime_us);

    // Start or cancel the reflow curve
    if (!reflowCurveRunning)
    {
        startReflowCurve = true;
    }
    else
    {
        cancelReflowCurve = true;
    }
}

void IRAM_ATTR btnDebounce(void *)
{
    if (digitalRead(BTN_PIN) == HIGH)
    {
        attachInterrupt(BTN_PIN, btnHandler, FALLING);
    }
    else
    {
        esp_timer_start_once(btnTimer, debounceTime_us);
    }
}

const int lmt85Lookup[] = {301, 150,
                           310, 149,
                           319, 148,
                           328, 147,
                           337, 146,
                           346, 145,
                           354, 144,
                           363, 143,
                           372, 142,
                           381, 141,
                           390, 140,
                           399, 139,
                           408, 138,
                           416, 137,
                           425, 136,
                           434, 135,
                           443, 134,
                           452, 133,
                           460, 132,
                           469, 131,
                           478, 130,
                           487, 129,
                           495, 128,
                           504, 127,
                           513, 126,
                           521, 125,
                           530, 124,
                           539, 123,
                           547, 122,
                           556, 121,
                           565, 120,
                           573, 119,
                           582, 118,
                           591, 117,
                           599, 116,
                           608, 115,
                           617, 114,
                           625, 113,
                           634, 112,
                           642, 111,
                           651, 110,
                           660, 109,
                           668, 108,
                           677, 107,
                           685, 106,
                           694, 105,
                           702, 104,
                           711, 103,
                           720, 102,
                           728, 101,
                           737, 100,
                           745, 99,
                           754, 98,
                           762, 97,
                           771, 96,
                           779, 95,
                           788, 94,
                           797, 93,
                           805, 92,
                           814, 91,
                           822, 90,
                           831, 89,
                           839, 88,
                           848, 87,
                           856, 86,
                           865, 85,
                           873, 84,
                           881, 83,
                           890, 82,
                           898, 81,
                           907, 80,
                           915, 79,
                           924, 78,
                           932, 77,
                           941, 76,
                           949, 75,
                           957, 74,
                           966, 73,
                           974, 72,
                           983, 71,
                           991, 70,
                           1000, 69,
                           1008, 68,
                           1017, 67,
                           1025, 66,
                           1034, 65,
                           1042, 64,
                           1051, 63,
                           1059, 62,
                           1067, 61,
                           1076, 60,
                           1084, 59,
                           1093, 58,
                           1101, 57,
                           1109, 56,
                           1118, 55,
                           1126, 54,
                           1134, 53,
                           1143, 52,
                           1151, 51,
                           1159, 50,
                           1167, 49,
                           1176, 48,
                           1184, 47,
                           1192, 46,
                           1201, 45,
                           1209, 44,
                           1217, 43,
                           1225, 42,
                           1234, 41,
                           1242, 40,
                           1250, 39,
                           1258, 38,
                           1267, 37,
                           1275, 36,
                           1283, 35,
                           1291, 34,
                           1299, 33,
                           1308, 32,
                           1316, 31,
                           1324, 30,
                           1332, 29,
                           1340, 28,
                           1348, 27,
                           1356, 26,
                           1365, 25,
                           1373, 24,
                           1381, 23,
                           1389, 22,
                           1397, 21,
                           1405, 20,
                           1413, 19,
                           1421, 18,
                           1430, 17,
                           1438, 16,
                           1446, 15,
                           1454, 14,
                           1462, 13,
                           1470, 12,
                           1478, 11,
                           1486, 10,
                           1494, 9,
                           1502, 8,
                           1511, 7,
                           1519, 6,
                           1527, 5,
                           1535, 4,
                           1543, 3,
                           1551, 2,
                           1559, 1,
                           1567, 0,
                           1575, -1,
                           1583, -2,
                           1591, -3,
                           1599, -4,
                           1607, -5,
                           1615, -6,
                           1623, -7,
                           1631, -8,
                           1639, -9,
                           1648, -10,
                           1656, -11,
                           1663, -12,
                           1671, -13,
                           1679, -14,
                           1687, -15,
                           1695, -16,
                           1703, -17,
                           1711, -18,
                           1719, -19,
                           1727, -20,
                           1735, -21,
                           1743, -22,
                           1751, -23,
                           1759, -24,
                           1767, -25,
                           1775, -26,
                           1783, -27,
                           1790, -28,
                           1798, -29,
                           1806, -30,
                           1814, -31,
                           1822, -32,
                           1830, -33,
                           1838, -34,
                           1845, -35,
                           1853, -36,
                           1861, -37,
                           1869, -38,
                           1877, -39,
                           1885, -40,
                           1892, -41,
                           1900, -42,
                           1908, -43,
                           1915, -44,
                           1921, -45,
                           1928, -46,
                           1935, -47,
                           1942, -48,
                           1949, -49,
                           1955, -50,
                           0, 0};

double getLMT85Temp(int lmt85_mV)
{
    int idx = -1;
    int lastValue = -10000;
    for (int i = 0; lmt85Lookup[i] != 0; i += 2)
    {
        lastValue = lmt85Lookup[i + 1];
        if (lmt85Lookup[i] > lmt85_mV)
        {
            idx = i;
            break;
        }
    }

    if (idx == 0)
    {
        // Too low; return lowest value
        return lmt85Lookup[1];
    }

    if (idx == -1)
    {
        // Too high; return highest value
        return lastValue;
    }

    return lmt85Lookup[idx - 1] - (((double)(lmt85_mV - lmt85Lookup[idx - 2])) / (lmt85Lookup[idx] - lmt85Lookup[idx - 2]));
}