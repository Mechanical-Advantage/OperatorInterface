#include "src/lib/Adafruit_TLC5947/Adafruit_TLC5947.h"
#include "src/lib/MCP23017/Adafruit_MCP23017.h"
#include "src/lib/ArduinoJoystickLibrary/Joystick/src/Joystick.h"
#include "src/lib/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include <Keyboard.h>

#define E_STOP_PIN 12
#define BUTTON_COUNT 24
#define FIRST_BUTTON_PIN 1
#define DIRECT_BUTTON_COUNT 8
#define SERIAL_BAUD_RATE 9600

Joystick_ Joystick = Joystick_(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, BUTTON_COUNT, 0, true, 
    true, true, true, true, true, false, false, false, false, false);

boolean eStopPressed;

// digitalReads are inverted because pullups are enabled

void setup() {
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    for (byte pin = FIRST_BUTTON_PIN; pin < DIRECT_BUTTON_COUNT + FIRST_BUTTON_PIN; pin++) {
        pinMode(pin, INPUT_PULLUP);
    }
    Serial.begin(SERIAL_BAUD_RATE);
    Keyboard.begin();
    Joystick.begin(false);
}

void loop() {
    // Directly connected buttons (1-8)
    for (byte button = 0; button < DIRECT_BUTTON_COUNT; button++) {
        Joystick.setButton(button, !digitalRead(button + FIRST_BUTTON_PIN));
    }
    
    Joystick.sendState();

    // E Stop
    if (!eStopPressed && !digitalRead(E_STOP_PIN)) {
        Keyboard.press(' ');
        eStopPressed = true;
    } else if (eStopPressed && digitalRead(E_STOP_PIN)) {
        Keyboard.release(' ');
        eStopPressed = false;
    }
}