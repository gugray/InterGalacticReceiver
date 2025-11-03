#include <Arduino.h>
#include <Wire.h>

#include "magic.h"

static const char *msg = "beep";
static uint8_t cmd;
static void receiveData(int bytecount);
static void sendData();

void setup()
{
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, HIGH);

    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
}

void loop()
{
    // digitalWrite(ONBOARD_LED_PIN, LOW);
    // delay(100);
    // digitalWrite(ONBOARD_LED_PIN, HIGH);
    // delay(900);
}

static void receiveData(int bytecount)
{
    for (int i = 0; i < bytecount; i++)
        cmd = Wire.read();

    digitalWrite(ONBOARD_LED_PIN, LOW);
    delay(100);
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    delay(900);
}

static void sendData()
{
    Wire.write(msg, strlen(msg));
}
