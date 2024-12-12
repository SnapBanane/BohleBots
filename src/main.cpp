#include <Wire.h>
#include <Arduino.h>
#include <bohlebots.h>

BohleBots bot;

void setup() {
    Wire.begin();
    Serial.begin(115200);
}

void loop() {
    bot.warte(5);
    Serial.print(bot.ballDistance);
    Serial.print(" : ");
    Serial.println(bot.ballDirection);
}