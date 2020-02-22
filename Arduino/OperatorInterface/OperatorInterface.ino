#include "src/lib/Adafruit_TLC5947/Adafruit_TLC5947.h"
#include "src/lib/MCP23017/Adafruit_MCP23017.h"
#include "src/lib/ArduinoJoystickLibrary/Joystick/src/Joystick.h"
#include "src/lib/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#include "OISerialProtocol.h"
#include <Keyboard.h>
#include "src/lib/FastLED/FastLED.h" // Using FastLED for broad compatibility and general feature set
#include "LEDState.h"                // Custom header that provides tracking of LED states

#define E_STOP_PIN 12
#define BUTTON_COUNT 24
#define FIRST_BUTTON_PIN 1
#define DIRECT_BUTTON_COUNT 8
#define ANALOG_INPUT_COUNT 6
#define MCP_INTERUPT_PIN 0
#define MCP_ADDR 0x20
#define SERIAL_BAUD_RATE 115200

#define MCP23017_INPUTS 16
#define MCP23017_USE_INTERRUPTS 0

#define TLC5947_LATCH_PIN 13
#define NUM_TLC5947 1
#define MAX_LED 20
#define LED_PWM_MAX 4095
#define LED_PWM_DIM 20
#define LED_PWM_MED 500
#define LED_UPDATE_INTERVAL 10 // EVERY_N_MILLISECONDS
#define LED_SLOW_INTERVAL_COUNT 50
#define LED_FAST_INTERVAL_COUNT 15
#define LED_PULSE_SLOW_DELTA (LED_PWM_MAX / LED_SLOW_INTERVAL_COUNT)
#define LED_PULSE_FAST_DELTA (LED_PWM_MAX / LED_FAST_INTERVAL_COUNT)

#define NEOPIXEL_PIN 9
#define NEOPIXEL_COUNT 7
#define NEOPIXEL_TYPE WS2812B
#define NEOPIXEL_COLOR_ORDER GRB

#define MCP_PRIMARY_BUTTONS ((BUTTON_COUNT / 2) - DIRECT_BUTTON_COUNT)

DECLARE_LEDSTATE_STORAGE;

Joystick_ primaryJoystick = Joystick_(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, BUTTON_COUNT/2, 0, true,
                               true, true, true, true, true, false, false, false, false, false);
Joystick_ secondaryJoystick = Joystick_(0x04, JOYSTICK_TYPE_GAMEPAD, BUTTON_COUNT/2, 0, false,
                               false, false, false, false, false, false, false, false, false, false);
typedef void (Joystick_::*joystickFunc)(int16_t);
const joystickFunc axisFuncs[] = {&Joystick_::setXAxis, &Joystick_::setYAxis, &Joystick_::setZAxis, &Joystick_::setRxAxis,
                                  &Joystick_::setRyAxis, &Joystick_::setRzAxis};

Adafruit_TLC5947 *tlc = new Adafruit_TLC5947(NUM_TLC5947, TLC5947_LATCH_PIN);
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_MCP23017 mcp;
CRGB pixel[NEOPIXEL_COUNT];

boolean eStopPressed;
volatile uint16_t mcpIntCap; // Captured interupt state
volatile uint16_t mcpIntF;   // Flags for which mcp buttons have changed

byte button2input[DIRECT_BUTTON_COUNT] = {1, 10, 11, 4, 5, 6, 7, 8};

// digitalReads are inverted because pullups are enabled

void setup()
{
    byte pin;

    pinMode(E_STOP_PIN, INPUT_PULLUP);
    for (pin = 0; pin < DIRECT_BUTTON_COUNT; pin++)
    {
        pinMode(button2input[pin], INPUT_PULLUP);
    }
#if MCP23017_USE_INTERRUPTS
    mcp.begin(MCP_ADDR & B00000111);
    mcp.setupInterrupts(true, false, LOW);
    for (byte pin = 0; pin < 16; pin++)
    {
        mcp.pinMode(pin, INPUT);
        mcp.pullUp(pin, HIGH);
        mcp.setupInterruptPin(pin, CHANGE);
    }

    // Make sure the initial state is correct
    readAllMCPButtons();
    enableMCPInterupt();
#else
    mcp.begin();
    for (pin = 0; pin < MCP23017_INPUTS; pin++)
    {
        mcp.pinMode(pin, INPUT); // Button i/p to GND
        mcp.pullUp(pin, HIGH);   // Enable pullup
    }
#endif

    Serial.begin(SERIAL_BAUD_RATE);
    Keyboard.begin();
    primaryJoystick.begin(false);
    secondaryJoystick.begin(false);

    tlc->begin();

    FastLED.addLeds<NEOPIXEL_TYPE, NEOPIXEL_PIN, NEOPIXEL_COLOR_ORDER>(pixel, NEOPIXEL_COUNT).setCorrection(TypicalLEDStrip);

    lcd.init(); // initialize the lcd
    lcd.clear();
    lcd.backlight();
    // Print a message to the LCD.
    lcd.setCursor(3, 1);
    lcd.print(F("FRC Team 6328"));
    lcd.setCursor(1, 2);
    lcd.print(F("Operator Interface"));

    // Flash all LEDs on and off
    SET_ALL_LEDS_ON;
    updateLEDState();
    delay(1000);
    SET_ALL_LEDS_OFF;
    updateLEDState(); 
    // Clear lcd
    lcd.clear();

    // Light up the neopixels (test)
    //pixelTest();

    // Test all the possible LED states
   /*  SET_LEDSTATE(14, LEDSTATE_DIM);
    SET_LEDSTATE(16, LEDSTATE_MED);
    SET_LEDSTATE(18, LEDSTATE_ON);
    SET_LEDSTATE(15, LEDSTATE_DIM);
    SET_LEDSTATE(17, LEDSTATE_MED);
    SET_LEDSTATE(19, LEDSTATE_ON);
    SET_LEDSTATE(6, LEDSTATE_BLINK_SLOW);
    SET_LEDSTATE(7, LEDSTATE_BLINK_FAST);
    SET_LEDSTATE(8, LEDSTATE_PULSE_SLOW);
    SET_LEDSTATE(9, LEDSTATE_PULSE_FAST); */
}

void pixelTest(void)
{
    pixel[0].setRGB(0, 150, 0);
    pixel[1].setRGB(150, 0, 0);
    pixel[2].setRGB(0, 0, 150);
    pixel[3].setRGB(150, 150, 0);
    pixel[4].setRGB(0, 150, 150);
    pixel[5].setRGB(150, 0, 150);
    pixel[6].setRGB(150, 150, 150);
    FastLED.show();
}

void pixelSetAll(CRGB color)
{
    fill_solid(pixel, NEOPIXEL_COUNT, color);
    FastLED.show();
}

void updateLEDState()
{
    static bool blinkSlowState, blinkFastState;
    static uint8_t slowCount, fastCount;
    static uint16_t pulseSlowVal, pulseFastVal;
    uint8_t state;

    slowCount++;
    fastCount++;
    if (slowCount > LED_SLOW_INTERVAL_COUNT)
    {
        blinkSlowState = blinkSlowState ? 0 : 1;
        slowCount = 0;
        pulseSlowVal = LED_PWM_MAX * !blinkSlowState; // Reset to max or min val
    }
    else
    {
        pulseSlowVal += LED_PULSE_SLOW_DELTA * (blinkSlowState ? 1 : -1);
    }

    if (fastCount > LED_FAST_INTERVAL_COUNT)
    {
        blinkFastState = blinkFastState ? 0 : 1;
        fastCount = 0;
        pulseFastVal = LED_PWM_MAX * !blinkFastState; // Reset to max or min val
    }
    else
    {
        pulseFastVal += LED_PULSE_FAST_DELTA * (blinkFastState ? 1 : -1);
    }

    for (byte led = 0; led < MAX_LED; led++)
    {
        state = GET_LEDSTATE(led);
        switch (state)
        {
        case LEDSTATE_OFF:
        default:
            tlc->setPWM(led, 0);
            break;
        case LEDSTATE_ON:
            tlc->setPWM(led, LED_PWM_MAX);
            break;
        case LEDSTATE_DIM:
            tlc->setPWM(led, LED_PWM_DIM);
            break;
        case LEDSTATE_MED:
            tlc->setPWM(led, LED_PWM_MED);
            break;
        case LEDSTATE_BLINK_FAST:
            tlc->setPWM(led, blinkFastState * LED_PWM_MAX);
            break;
        case LEDSTATE_BLINK_SLOW:
            tlc->setPWM(led, blinkSlowState * LED_PWM_MAX);
            break;
        case LEDSTATE_PULSE_FAST:
            tlc->setPWM(led, pulseFastVal);
            break;
        case LEDSTATE_PULSE_SLOW:
            tlc->setPWM(led, pulseSlowVal);
            break;
        }
    }
    tlc->write();
}

void loop()
{
    char line[21];
    uint16_t mcp_input;
    byte button;

    readSerialData();

    // Directly connected buttons (1-8)
    for (button = 0; button < DIRECT_BUTTON_COUNT; button++)
    {
        primaryJoystick.setButton(button, !digitalRead(button2input[button]));
        if (button < 6)
        {
            SET_LEDSTATE(button, (!digitalRead(button2input[button])) ? LEDSTATE_ON : LEDSTATE_OFF);
        }
    }

// MCP23017 expander inputs
#if MCP23017_USE_INTERRUPTS
    // MCP Buttons (9-24)
    noInterrupts(); // Disable interupts to ensure this has a proper copy
    if (mcpIntF)
    {
        uint16_t localIntF = mcpIntF;
        uint16_t localIntCap = mcpIntCap;
        mcpIntF = 0;
        interrupts();
        for (byte button = 0; button < 16; button++)
        {
            if (localIntF & _BV(button))
            {
                uint8_t shiftVal = button > 7 ? 8 : 0;
                bool isPrimaryJoystick = mcpPinToButton(button) < MCP_PRIMARY_BUTTONS;
                Joystick_* joystick = isPrimaryJoystick ? &primaryJoystick : &secondaryJoystick;
                joystick->setButton(isPrimaryJoystick ? mcpPinToButton(button) + DIRECT_BUTTON_COUNT : mcpPinToButton(button) - 
                    MCP_PRIMARY_BUTTONS, !((localIntCap & _BV(button)) >> shiftVal));
            }
        }
    }
    else
    {
        interrupts();
    }
#else
    readAllMCPButtons();
#endif

    // Analog inputs
    // The default axis range is 0-1023, which matches the ADC
    for (byte input = 0; input < ANALOG_INPUT_COUNT; input++)
    {
        (primaryJoystick.*axisFuncs[input])(analogRead(input));
    }

   /*  EVERY_N_MILLISECONDS(100)
    {
        lcd.setCursor(0, 3);
        sprintf(line, "%4d %4d %4d %4d", analogRead(0), analogRead(1), analogRead(2), analogRead(3));
        lcd.print(line);
    } */

    primaryJoystick.sendState();
    secondaryJoystick.sendState();

    EVERY_N_MILLISECONDS(LED_UPDATE_INTERVAL)
    {
        updateLEDState();
    }

    // E Stop
    if (!eStopPressed && !digitalRead(E_STOP_PIN))
    {
        Keyboard.press(' ');
        eStopPressed = true;
        lcd.setCursor(0, 0);
        lcd.print("** Emergency Stop **");
        pixelSetAll(CRGB::Red);
    }
    
    else if (eStopPressed && digitalRead(E_STOP_PIN))
    {
        Keyboard.release(' ');
        lcd.setCursor(0, 0);
        lcd.print("                    ");
        eStopPressed = false;
        pixelSetAll(CRGB::Black);
    }
}

void readAllMCPButtons()
{
    // Invert because of pullup
    uint16_t values = ~mcp.readGPIOAB();
    for (byte mcpPin = 0; mcpPin < 16; mcpPin++)
    {
        uint8_t shiftVal = mcpPin > 7 ? 8 : 0;
        bool isPrimaryJoystick = mcpPinToButton(mcpPin) < MCP_PRIMARY_BUTTONS;
        Joystick_* joystick = isPrimaryJoystick ? &primaryJoystick : &secondaryJoystick;
        joystick->setButton(isPrimaryJoystick ? mcpPinToButton(mcpPin) + DIRECT_BUTTON_COUNT : mcpPinToButton(mcpPin) - 
            MCP_PRIMARY_BUTTONS, (_BV(mcpPin) & values) >> shiftVal);
    }
}

void enableMCPInterupt()
{
    attachInterrupt(digitalPinToInterrupt(MCP_INTERUPT_PIN), mcpIsr, FALLING);
}

void mcpIsr()
{
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

uint8_t mcpPinToButton(uint8_t pin)
{
    // The 9-16 and 17-24 labels on the PCB are backwards relative to the chip (A vs B) so this swaps them
    if (pin > 7)
    {
        return pin - 8;
    }
    else
    {
        // GPA is in backwards order
        return (7 - pin) + 8;
    }
}

uint16_t readTwoMCPRegisters(uint8_t reg)
{
    Wire.beginTransmission(MCP_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MCP_ADDR, 2);
    return Wire.read() | (Wire.read() << 8);
}