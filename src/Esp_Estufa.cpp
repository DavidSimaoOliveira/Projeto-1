#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>

void setup()
{
    Serial.begin(9600);
};

void loop()
{
    Serial.println("Olá");
    delay(5);
};