#include "Persistent_Configuration_System.h"

ConfigStore ConfigStore::instance;

ConfigStore::ConfigStore()
    : initialized(false)
{
}

ConfigStore& ConfigStore::getInstance()
{
    return instance;
}

bool ConfigStore::init()
{
    initialized = true;
    return true;
}

bool ConfigStore::validateEndurance(ConfigKey /* key */)
{
    return true;
}
