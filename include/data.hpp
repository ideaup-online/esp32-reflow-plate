#pragma once

#include <Arduino.h>

class Data
{
public:
    Data();
    ~Data();

    double getTc1Temp() const;
    double getTc2Temp() const;
    int getLmt85_mV() const;
    double getSetpoint() const;

    void setTc1Temp(double temp);
    void setTc2Temp(double temp);
    void setLmt85_mV(int mv);
    void setSetpoint(double setpoint);

private:
    double _tc1Temp;
    double _tc2Temp;
    int _lmt85_mV;
    double _setpoint;

    SemaphoreHandle_t _tc1TempMutex;
    SemaphoreHandle_t _tc2TempMutex;
    SemaphoreHandle_t _lmt85Mutex;
    SemaphoreHandle_t _setpointMutex;
};

inline Data::Data()
    : _tc1Temp(0.0),
      _tc2Temp(0.0),
      _lmt85_mV(0),
      _setpoint(0.0),
      _tc1TempMutex(NULL),
      _tc2TempMutex(NULL),
      _lmt85Mutex(NULL),
      _setpointMutex(NULL)
{
    _tc1TempMutex = xSemaphoreCreateMutex();
    if (_tc1TempMutex == NULL)
    {
        Serial.println("Failed to create TC1 mutex");
        while (true)
        {
            delay(10);
        }
    }
    _tc2TempMutex = xSemaphoreCreateMutex();
    if (_tc2TempMutex == NULL)
    {
        Serial.println("Failed to create TC2 mutex");
        while (true)
        {
            delay(10);
        }
    }
    _lmt85Mutex = xSemaphoreCreateMutex();
    if (_lmt85Mutex == NULL)
    {
        Serial.println("Failed to create LMT85 mutex");
        while (true)
        {
            delay(10);
        }
    }
    _setpointMutex = xSemaphoreCreateMutex();
    if (_setpointMutex == NULL)
    {
        Serial.println("Failed to create setpoint mutex");
        while (true)
        {
            delay(10);
        }
    }
}

inline Data::~Data()
{
    vSemaphoreDelete(_tc1TempMutex);
    vSemaphoreDelete(_tc2TempMutex);
    vSemaphoreDelete(_lmt85Mutex);
    vSemaphoreDelete(_setpointMutex);
}

inline double Data::getTc1Temp() const
{
    double tmp = 0.0;

    xSemaphoreTake(_tc1TempMutex, portMAX_DELAY);
    tmp = _tc1Temp;
    xSemaphoreGive(_tc1TempMutex);

    return tmp;
}

inline double Data::getTc2Temp() const
{
    double tmp = 0.0;

    xSemaphoreTake(_tc2TempMutex, portMAX_DELAY);
    tmp = _tc2Temp;
    xSemaphoreGive(_tc2TempMutex);

    return tmp;
}

inline int Data::getLmt85_mV() const
{
    int tmp = 0;

    xSemaphoreTake(_lmt85Mutex, portMAX_DELAY);
    tmp = _lmt85_mV;
    xSemaphoreGive(_lmt85Mutex);

    return tmp;
}

inline double Data::getSetpoint() const
{
    double tmp = 0.0;

    xSemaphoreTake(_setpointMutex, portMAX_DELAY);
    tmp = _setpoint;
    xSemaphoreGive(_setpointMutex);

    return tmp;
}

inline void Data::setTc1Temp(double temp)
{
    xSemaphoreTake(_tc1TempMutex, portMAX_DELAY);
    _tc1Temp = temp;
    xSemaphoreGive(_tc1TempMutex);
}

inline void Data::setTc2Temp(double temp)
{
    xSemaphoreTake(_tc1TempMutex, portMAX_DELAY);
    _tc2Temp = temp;
    xSemaphoreGive(_tc1TempMutex);
}

inline void Data::setLmt85_mV(int mv)
{
    xSemaphoreTake(_lmt85Mutex, portMAX_DELAY);
    _lmt85_mV = mv;
    xSemaphoreGive(_lmt85Mutex);
}

inline void Data::setSetpoint(double setpoint)
{
    xSemaphoreTake(_setpointMutex, portMAX_DELAY);
    _setpoint = setpoint;
    xSemaphoreGive(_setpointMutex);
}
