/*
* <locals.h>
* This file defines the translation strings for the application.
*   It uses a map to store translations for different languages.
*   The current language can be set dynamically.
*   Each translation key corresponds to a specific string in the application.
*   The translate function retrieves the translation for the current language.
*   If the translation is not found, it defaults to English.
* To use it, simply call translate(TranslationKey) with the desired key and add c_str() to convert to a string
*/
#include <map>
#include <string>

// Define supported languages
#define LANG_EN 0
#define LANG_FR 1

// Define the current language (can be dynamically set)
int current_language = LANG_FR;

enum TranslationKey {
    ENGINE_REVOLUTIONS,
    FUEL_RATE,
    ENGINE_STATE,
    MOTION_SENSOR_OFFSETS,
    MOTION_SENSOR_HEADING,
    EXHAUST,
    PORTSIDE,
    EXHAUST_FULL,
    ENGINE,
    STARBOARD,
    ROOM,
    WATER,
    TEMPERATURE,
    PRESSURE,
    OIL,
    TANK,
    LEVEL,
    CHAIN_COUNTER,
    FRESH,
    CONSUMPTION
};

std::map<TranslationKey, std::map<int, std::string>> translations_enum = {
    {ENGINE_REVOLUTIONS, {
        {LANG_EN, "Engine revolutions (x60 for RPM)"},
        {LANG_FR, "Révolutions moteur (x60 pour RPM)"}
    }},
    {FUEL_RATE, {
        {LANG_EN, "Fuel rate of consumption"},
        {LANG_FR, "Consommation de carburant"}
    }},
    {ENGINE_STATE, {
        {LANG_EN, "Engine state"},
        {LANG_FR, "État du moteur"}
    }},
    {MOTION_SENSOR_OFFSETS, {
        {LANG_EN, "Gyroscope - Offsets"},
        {LANG_FR, "Gyroscope - Décalages"}
    }},
    {MOTION_SENSOR_HEADING, {
        {LANG_EN, "Current magnetic heading received from the compass"},
        {LANG_FR, "Cap magnétique actuel reçu de la boussole"}
    }},
    {EXHAUST, {
        {LANG_EN, "Exhaust"},
        {LANG_FR, "Echappt"}
    }},
    {PORTSIDE, {
        {LANG_EN, "Portside"},
        {LANG_FR, "Babord"}
    }},
    {EXHAUST_FULL, {
        {LANG_EN, "Exhaust"},
        {LANG_FR, "Echappement"}
    }},
    {ENGINE, {
        {LANG_EN, "Engine"},
        {LANG_FR, "Moteur"}
    }},
    {STARBOARD, {
        {LANG_EN, "Starboard"},
        {LANG_FR, "Tribord"}
    }},
    {ROOM, {
        {LANG_EN, "Room"},
        {LANG_FR, "Compartiment"}
    }},
    {WATER, {
        {LANG_EN, "Water"},
        {LANG_FR, "Eau"}
    }},
    {TEMPERATURE, {
        {LANG_EN, "Temperature"},
        {LANG_FR, "Température"}
    }},
    {PRESSURE, {
        {LANG_EN, "Pressure"},
        {LANG_FR, "Pression"}
    }},
    {OIL, {
        {LANG_EN, "Oil"},
        {LANG_FR, "Huile"}
    }},
    {TANK, {
        {LANG_EN, "Tank"},
        {LANG_FR, "Cuve"}
    }},
    {LEVEL, {
        {LANG_EN, "Level"},
        {LANG_FR, "Niveau"}
    }},
    {CHAIN_COUNTER, {
        {LANG_EN, "Chain Counter"},
        {LANG_FR, "Compteur Chaine"}
    }},
    {FRESH, {
        {LANG_EN, "Fresh"},
        {LANG_FR, "Inox"}
    }},
    {CONSUMPTION, {
        {LANG_EN, "Cons."},
        {LANG_FR, "Conso"}
    }}
};

std::string translate(TranslationKey key) {
    if (translations_enum.find(key) != translations_enum.end() &&
        translations_enum[key].find(current_language) != translations_enum[key].end()) {
        return translations_enum[key][current_language];
    }
    return translations_enum[key][LANG_EN];
}