#include <Arduino.h>

#define OISERIAL_MAX_MSG_SIZE 26  // Start, size, command, up to 22 payload, checksum
#define OISERIAL_MIN_MSG_SIZE 4
#define OISERIAL_MAX_CMD_VAL 5 // TBD: update
#define OISERIAL_START_BYTE 0xc3
#define OISERIAL_ACK 0xaa
#define OISERIAL_NACK 0xff
typedef enum
{
  OISERIAL_CMD_NOOP = 0,
  OISERIAL_CMD_LED_SET,
  OISERIAL_CMD_LCD_MSG, //Command for LCD message; x,y coordinates and messages bytes
  OISERIAL_CMD_PIXEL_SET,
  OISERIAL_CMD_ALLPIXEL_SET,
  OISERIAL_CMD_GET_BOARDID,
  OISERIAL_CMD_GET_LCDSIZE, 
} cmds;

#define OISERIAL_LCD_X_MASK 0xf8
#define OISERIAL_LCD_Y_MASK 0x07
#define OISERIAL_LCD_X_RIGHT_SHIFT 3
#define MAX_PAYLOAD_SIZE 22 
#define OISERIAL_START_IDX 0
#define OISERIAL_SIZE_IDX 1
#define OISERIAL_CMD_IDX 2

extern void readSerialData();