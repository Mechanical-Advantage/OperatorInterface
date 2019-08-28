#include "src/lib/Adafruit_TLC5947/Adafruit_TLC5947.h"
#include "src/lib/MCP23017/Adafruit_MCP23017.h"
#include "src/lib/ArduinoJoystickLibrary/Joystick/src/Joystick.h"
#include "src/lib/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include <Keyboard.h>

#define E_STOP_PIN 12
#define BUTTON_COUNT 24
#define FIRST_BUTTON_PIN 1
#define DIRECT_BUTTON_COUNT 8
#define ANALOG_INPUT_COUNT 6
#define SERIAL_BAUD_RATE 9600

Joystick_ Joystick = Joystick_(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, BUTTON_COUNT, 0, true, 
    true, true, true, true, true, false, false, false, false, false);
typedef void (Joystick_::*joystickFunc)(int16_t);
const joystickFunc axisFuncs[] = {&Joystick_::setXAxis, &Joystick_::setYAxis, &Joystick_::setZAxis, &Joystick_::setRxAxis, 
    &Joystick_::setRyAxis, &Joystick_::setRzAxis};

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

    // Analog inputs
    // The default axis range is 0-1023, which matches the ADC
    for (byte input = 0; input < ANALOG_INPUT_COUNT; input++) {
        (Joystick.*axisFuncs[input])(analogRead(input));
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