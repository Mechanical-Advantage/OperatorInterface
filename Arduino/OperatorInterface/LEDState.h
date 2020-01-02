// LEDState.h
//
// This header supports storing state information for LEDs.  The idea is that we
// have a certain number of LEDs, and each LED can be in one of a small number
// of states.  We want to store all the LED states in the smallest possible
// amount of memory.  This header uses a number of macros to define the
// operating properties (including number of LEDs, bits per LED, and available
// states), declares the minimum amount of storage needed (as an array of uint16
// data), and provides other macros for getting and setting the LED states.
//

#define LED_COUNT 24      // Total number of entries to store
#define BITS_PER_LED 3    // How many bits we need to store each LED's state
#define MASK_ONE_LED 0x7  // Mask for the number of bits specified as BITS_PER_LED.
                          // 0x means hexadecimal (makes clear that this is a mask)
                          // and 7 is binary 111, representing the set of 3 bits

#define LEDS_PER_LOCATION ((int)(16/BITS_PER_LED))  // Using 16-bit unsigned int
#define STATE_LOCATIONS ((int)(((float)LED_COUNT/LEDS_PER_LOCATION)+0.5))  // How many 16-bit ints we need

// These are the possible states for each LED
#define LEDSTATE_OFF           0x0   // Completely off
#define LEDSTATE_BLINK_SLOW    0x1   // Slowly blinking
#define LEDSTATE_BLINK_FAST    0x2   // Rapidly blinking
#define LEDSTATE_PULSE_SLOW    0x3   // Slowly pulsing
#define LEDSTATE_PULSE_FAST    0x4   // Rapidly pulsing
#define LEDSTATE_DIM           0x5   // On dim
#define LEDSTATE_MED           0x6   // On medium
#define LEDSTATE_ON            0x7   // On bright

// This array stores the actual LED state.  Each array entry holds LEDS_PER_LOCATION states.
// By shifting and masking, we can access each individual location.

//   X  XXX XXX XXX XXX XXX 
//   ^  ^^^ ^^^ ^^^ ^^^ ^^^
//   |   |   |   |   |   |--- shift 0  First LED
//   |   |   |   |   |------- shift 3  Second LED
//   |   |   |   |----------- shift 6  Third LED
//   |   |   |--------------- shift 9  Fourth LED
//   |   |------------------- shift 12 Fifth LED
//   |----------------------- Leftover bit, not used

// The 'location' is the array entry.  When all packed together, our
// data is organized like this (where LEDs are numbered 000 to 023)

//   X 004 003 002 001 000  <-- array[0]
//   X 009 008 007 006 005  <-- array[1]
//   X 014 013 012 011 010  <-- array[2]
//   X 019 018 017 016 015  <-- array[3]
//   X XXX 023 022 021 020  <-- array[4]

// The actual storage array
extern uint16_t ledState[STATE_LOCATIONS];
// Must have exactly one declaration of the storage (in one file)
#define DECLARE_LEDSTATE_STORAGE uint16_t ledState[STATE_LOCATIONS] 
//#define DECLARE_LEDSTATE_STORAGE_EXTERN extern uint16_t ledState[STATE_LOCATIONS] 

// Which array element (location) holds this LED's value
#define LED_LOC_IDX(led) ((int)(led/LEDS_PER_LOCATION))

// The necessary shift amount to access this LED's value 
#define LED_SHIFT(led) ((led % LEDS_PER_LOCATION) * BITS_PER_LED)

// Macro to do the shifting in order to access this LED's value
// << is the bitwise left-shift operator
// So for example the mask for LED 1 (with a shift of 3 is) 00000000 00111000
#define LED_MASK(led) (MASK_ONE_LED << LED_SHIFT(led))

// Macro to set a specified LED to a specific state
// Mask out the old value using the inverted bitmask, then set the new value
#define SET_LEDSTATE(led,state) (ledState[LED_LOC_IDX(led)] = (ledState[LED_LOC_IDX(led)] & ~LED_MASK(led)) | ((state & MASK_ONE_LED) << LED_SHIFT(led) ) )

// Macro to get a specified LED's value
#define GET_LEDSTATE(led) ((ledState[LED_LOC_IDX(led)] >> LED_SHIFT(led)) & MASK_ONE_LED)

// Clear all of the array at once
#define SET_ALL_LEDS_OFF  (memset(ledState, 0, sizeof(ledState)))

// Set all LEDs to on (assumes ON is all-1's)
#define SET_ALL_LEDS_ON   (memset(ledState, 0xff, sizeof(ledState)))
