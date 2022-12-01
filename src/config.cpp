#include <LittleFS.h>
#include <ArduinoJson.h>
#include "config.hpp"

Config::Config()
    : _doc(1024) {}

bool Config::readConfig(File configFile)
{
    DeserializationError error = deserializeJson(_doc, configFile);

    return !error;
}

const char *Config::getSSID()
{
    return _doc["ssid"];
}

const char *Config::getKey()
{
    return _doc["key"];
}

const char *Config::getMDNS()
{
    return _doc["mdns"];
}
