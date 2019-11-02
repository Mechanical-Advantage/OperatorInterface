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
#define MCP_INTERUPT_PIN 0
#define MCP_ADDR 0x20
#define SERIAL_BAUD_RATE 9600

Joystick_ Joystick = Joystick_(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, BUTTON_COUNT, 0, true, 
    true, true, true, true, true, false, false, false, false, false);
typedef void (Joystick_::*joystickFunc)(int16_t);
const joystickFunc axisFuncs[] = {&Joystick_::setXAxis, &Joystick_::setYAxis, &Joystick_::setZAxis, &Joystick_::setRxAxis, 
    &Joystick_::setRyAxis, &Joystick_::setRzAxis};

Adafruit_MCP23017 mcp;

boolean eStopPressed;
volatile uint16_t mcpIntCap; // Captured interupt state
volatile uint16_t mcpIntF; // Flags for which mcp buttons have changed

// digitalReads are inverted because pullups are enabled

void setup() {
    pinMode(E_STOP_PIN, INPUT_PULLUP);
    for (byte pin = FIRST_BUTTON_PIN; pin < DIRECT_BUTTON_COUNT + FIRST_BUTTON_PIN; pin++) {
        pinMode(pin, INPUT_PULLUP);
    }
    Serial.begin(SERIAL_BAUD_RATE);
    Keyboard.begin();
    Joystick.begin(false);
    mcp.begin(MCP_ADDR & B00000111);
    mcp.setupInterrupts(true, false, LOW);
    for (byte pin = 0; pin < 16; pin++) {
        mcp.pinMode(pin, INPUT);
        mcp.pullUp(pin, HIGH);
        mcp.setupInterruptPin(pin, CHANGE);
    }
    // Make sure the initial state is correct
    readAllMCPButtons();
    enableMCPInterupt();
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

    // MCP Buttons (9-24)
    noInterrupts(); // Disable interupts to ensure this has a proper copy
    if (mcpIntF) {
        uint16_t localIntF = mcpIntF;
        uint16_t localIntCap = mcpIntCap;
        mcpIntF = 0;
        interrupts();
        for (byte button = 0; button < 16; button++) {
            if (localIntF & _BV(button)) {
                uint8_t shiftVal = button > 7 ? 8 : 0;
                Joystick.setButton(mcpPinToButton(button)+DIRECT_BUTTON_COUNT, !((localIntCap & _BV(button)) >> shiftVal));
            }
        }
    } else {
        interrupts();
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

void readAllMCPButtons() {
    // Invert because of pullup
    uint16_t values = ~mcp.readGPIOAB();
    for (byte mcpPin = 0; mcpPin < 16; mcpPin++) {
        uint8_t shiftVal = mcpPin > 7 ? 8 : 0;
        Joystick.setButton(mcpPinToButton(mcpPin)+DIRECT_BUTTON_COUNT, (_BV(mcpPin) & values) >> shiftVal);
    }
}

void enableMCPInterupt() {
    attachInterrupt(digitalPinToInterrupt(MCP_INTERUPT_PIN), mcpIsr, FALLING);
}

void mcpIsr() {
    // See https://www.best-microcontroller-projects.com/mcp23017.html
     noInterrupts();

    // Debounce. Slow I2C: extra debounce between interrupts anyway.
    // Can not use delay() in interrupt code.
    delayMicroseconds(1000); 
 
    // Stop interrupts from external pin.
    detachInterrupt(digitalPinToInterrupt(MCP_INTERUPT_PIN));
    interrupts(); // re-start interrupts for mcp

    uint16_t curIntF = readTwoMCPRegisters(MCP23017_INTFA);
    mcpIntF |= curIntF; // mcpIntF will contain bits for all changes since last processed
    uint16_t curIntCap = readTwoMCPRegisters(MCP23017_INTCAPA);
    mcpIntCap = (mcpIntCap & ~curIntF) | (curIntCap & curIntF); // Only update bits flagged currently

    enableMCPInterupt();
}

uint8_t mcpPinToButton(uint8_t pin) {
    // The 9-16 and 17-24 labels on the PCB are backwards relative to the chip (A vs B) so this swaps them
    if (pin > 7) {
        return pin - 8;
    } else {
        // GPA is in backwards order
        return (7 - pin) + 8;
    }
}

uint16_t readTwoMCPRegisters(uint8_t reg) {
    Wire.beginTransmission(MCP_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MCP_ADDR, 2);
    return Wire.read() | (Wire.read() << 8);
}