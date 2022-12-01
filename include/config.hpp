#pragma once

#include <ArduinoJson.h>
#include <LittleFS.h>

class Config
{
public:
    Config();

public:
    bool readConfig(File configFile);

    const char *getSSID();
    const char *getKey();
    const char *getMDNS();

private:
    DynamicJsonDocument _doc;
};
