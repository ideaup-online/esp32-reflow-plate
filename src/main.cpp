#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_MAX31855.h"

#define TC_DO_PIN 19
#define TC_CLK_PIN 18
#define TC1_CS_PIN 4
#define TC2_CS_PIN 33

#define FET_PIN 27

#define BTN_PIN 0
#define SDA_PIN 32
#define SCL_PIN 25

#define ADC_ADDR 0x36

#define OLED_ADDR 0x3c
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Initialize the Thermocouples
Adafruit_MAX31855 thermocouple1(TC_CLK_PIN, TC1_CS_PIN, TC_DO_PIN);
Adafruit_MAX31855 thermocouple2(TC_CLK_PIN, TC2_CS_PIN, TC_DO_PIN);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
const int lmt85Height = 20;

double tc1TempC = 0.0;
double tc2TempC = 0.0;
int lmt85_mV = 0;

double c2f(double celcuis);
double getLMT85Temp(int lmt85_mV);

bool btnState = false;

void setup()
{
    Serial.begin(115200);

    // wait for Serial on Leonardo/Zero, etc
    while (!Serial)
    {
        delay(1);
    }

    Serial.println("Solder Reflow Plate Controller V1.0");

    // Set up heater pin and ensure
    // heater is off to start
    Serial.print("Initializing heater to off...");
    pinMode(FET_PIN, OUTPUT);
    digitalWrite(FET_PIN, HIGH);
    delay(5);
    digitalWrite(FET_PIN, LOW);
    Serial.println("DONE.");

    // Wait for thermocouple chips to stabilize
    delay(500);
    Serial.print("Initializing thermocouple 1...");
    if (!thermocouple1.begin())
    {
        Serial.println("ERROR.");
        while (1)
            delay(10);
    }
    Serial.println("DONE.");
    Serial.print("Initializing thermocouple 2...");
    if (!thermocouple2.begin())
    {
        Serial.println("ERROR.");
        while (1)
            delay(10);
    }
    Serial.println("DONE.");
    Serial.print("Initializing OLED...");
    Wire.setPins(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println("SSD1306 allocation failed");
        while (1)
            delay(10);
    }
    Serial.println("DONE.");

    // Set up ADC (MAX11645)
    Serial.print("Initializing external ADC...");
    Wire.begin();
    Wire.beginTransmission((uint16_t)ADC_ADDR);
    Wire.write((uint8_t)0b10100000);
    Wire.write((uint8_t)0b01100001);
    Wire.endTransmission();
    Serial.println("DONE.");

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();

    pinMode(BTN_PIN, INPUT);
    btnState = digitalRead(BTN_PIN) == HIGH;
}

void loop()
{
    double c = 0.0;
    bool displayStuffHappened = false;

    c = thermocouple1.readCelsius();
    if (isnan(c))
    {
        Serial.println("Thermocouple 1 fault(s) detected!");
        uint8_t e = thermocouple1.readError();
        if (e & MAX31855_FAULT_OPEN)
            Serial.println("FAULT: Thermocouple is open - no connections.");
        if (e & MAX31855_FAULT_SHORT_GND)
            Serial.println("FAULT: Thermocouple is short-circuited to GND.");
        if (e & MAX31855_FAULT_SHORT_VCC)
            Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
    }
    else
    {
        if (c != tc1TempC)
        {
            // Save new value
            tc1TempC = c;

            // Update display
            display.fillRect(tc1TempX, tc1TempY, tc1TempWidth, tc1TempHeight, SSD1306_BLACK);
            display.setCursor(tc1TempX, tc1TempY);
            display.printf("T1: %6.2f C %6.2f F", tc1TempC, c2f(tc1TempC));
            displayStuffHappened = true;
        }
    }
    c = thermocouple2.readCelsius();
    if (isnan(c))
    {
        Serial.println("Thermocouple 2 fault(s) detected!");
        uint8_t e = thermocouple2.readError();
        if (e & MAX31855_FAULT_OPEN)
            Serial.println("FAULT: Thermocouple is open - no connections.");
        if (e & MAX31855_FAULT_SHORT_GND)
            Serial.println("FAULT: Thermocouple is short-circuited to GND.");
        if (e & MAX31855_FAULT_SHORT_VCC)
            Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
    }
    else
    {
        if (c != tc2TempC)
        {
            // Save new value
            tc2TempC = c;

            // Update display
            display.fillRect(tc2TempX, tc2TempY, tc2TempWidth, tc2TempHeight, SSD1306_BLACK);
            display.setCursor(tc2TempX, tc2TempY);
            display.printf("T2: %6.2f C %6.2f F", tc2TempC, c2f(tc2TempC));
            displayStuffHappened = true;
        }
    }

    // Read the value of the LMT85
    uint16_t lmt85Counts = 0;

    // Request 2 bytes from the external ADC
    uint8_t bytesReceived = Wire.requestFrom(ADC_ADDR, 2);
    if (bytesReceived == 2)
    {
        uint8_t tmp[2];
        Wire.readBytes(tmp, 2);
        lmt85Counts = ((tmp[0] << 8) | tmp[1]) & 0x0fff;
    }

    // Voltage reference is 2.048V, ADC is 12-bit
    // (4096 counts); thus, each count represents
    // 0.5mV
    int tmp = lmt85Counts / 2;
    if (tmp != lmt85_mV)
    {
        // Save new value
        lmt85_mV = tmp;

        // Look up temp for this voltage
        double c = getLMT85Temp(lmt85_mV);

        // Update display
        display.fillRect(lmt85X, lmt85Y, lmt85Width, lmt85Height, SSD1306_BLACK);
        display.setCursor(lmt85X, lmt85Y);
        display.printf("LM: %6.2f C %6.2f F", c, c2f(c));
        display.setCursor(lmt85X, lmt85Y + 10);
        display.printf("      %4d mV", lmt85_mV);
        displayStuffHappened = true;
    }

    // Update actual display
    if (displayStuffHappened)
    {
        display.display();
    }

    bool tmp2 = digitalRead(BTN_PIN) == HIGH;
    if (tmp2 != btnState)
    {
        btnState = tmp2;

        if (btnState == false)
        {
            Serial.println("Heater ON");
            digitalWrite(FET_PIN, HIGH);
        }
        else
        {
            Serial.println("Heater OFF");
            digitalWrite(FET_PIN, LOW);
        }
    }

    delay(250);
}

double c2f(double celcuis)
{
    return (celcuis * (9.0 / 5.0)) + 32;
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