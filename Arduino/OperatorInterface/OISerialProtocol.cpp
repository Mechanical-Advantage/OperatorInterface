#include "OISerialProtocol.h"
#include "LEDState.h" 
#include "src/lib/LiquidCrystal_I2C/LiquidCrystal_I2C.h" 
#include "src/lib/FastLED/FastLED.h"

#define BYTESREMAINING_NOT_SET_YET 0xff;  // Must be nonzero - large flag value 
#define TIMEOUT 5000 //time in milliseconds

byte oiMsgBuf[OISERIAL_MAX_MSG_SIZE];
uint8_t oiMsgBufIdx = 0;
uint8_t oiMsgBytesRemaining = BYTESREMAINING_NOT_SET_YET;
//DECLARE_LEDSTATE_STORAGE_EXTERN;
extern LiquidCrystal_I2C lcd;
extern void pixelSetOne(byte pixel_id, byte pixel_r, byte pixel_g, byte pixel_b);
extern void pixelSetAll(CRGB);

unsigned long lastKeepAlive;
void sendNack()
{
  Serial.write(OISERIAL_NACK);
  oiMsgBufIdx = 0;
  oiMsgBytesRemaining = BYTESREMAINING_NOT_SET_YET;
}

void sendAck()
{
  Serial.write(OISERIAL_ACK);
  oiMsgBufIdx = 0;
  oiMsgBytesRemaining = BYTESREMAINING_NOT_SET_YET;
}

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the terms of the GNU GPL 3.0 license
// source: https://www.leonardomiliani.com/en/2013/un-semplice-crc8-per-arduino/
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
 
bool isSerialTimedOut() 
{
  if(millis() - lastKeepAlive > TIMEOUT) {
    return true;
  } 
  else {
    return false;
  } 
}
void processCmd()
{
  byte cmd = oiMsgBuf[OISERIAL_CMD_IDX];
  byte lcd_x;
  byte lcd_y;
  byte pixel_id;
  byte pixel_r;
  byte pixel_g;
  byte pixel_b;
  CRGB color;
  //lcd.setCursor(0, 3);
  //lcd.print("CMD:");  

  if (cmd == OISERIAL_CMD_NOOP) {
    lastKeepAlive = millis(); 
  }
  else if (cmd == OISERIAL_CMD_LED_SET)
  {
    // This is an LED control command.  
    // The next 2 bytes indicate which LED and what value to set.
    SET_LEDSTATE(oiMsgBuf[OISERIAL_CMD_IDX + 1], oiMsgBuf[OISERIAL_CMD_IDX + 2]);
    //lcd.print("LED set");
  }
  else if (cmd == OISERIAL_CMD_LCD_MSG) 
  {
    //lcd.print("LCD set");
    lcd_y = oiMsgBuf[OISERIAL_CMD_IDX + 1] & OISERIAL_LCD_Y_MASK;
    lcd_x = (oiMsgBuf[OISERIAL_CMD_IDX + 1] & OISERIAL_LCD_X_MASK) >> OISERIAL_LCD_X_RIGHT_SHIFT; 


    // Insert a null in place of the checksum to denote end of string
    //oiMsgBuf[oiMsgBuf[OISERIAL_SIZE_IDX] - 1] = 0;
    lcd.setCursor(lcd_x, lcd_y);
    for (uint16_t charidx = OISERIAL_CMD_IDX + 2; charidx < oiMsgBuf[OISERIAL_SIZE_IDX] - 1; charidx++)
    {
      lcd.print(static_cast<char>(oiMsgBuf[charidx]));
    }
    

    /* char lcdbuf[21];
    strcpy(lcdbuf, oiMsgBuf[OISERIAL_CMD_IDX + 2]);
    sprintf(lcdbuf, "%d chars", strlen(lcdbuf));
    lcd.print(lcdbuf); */
    //lcd.print(String(oiMsgBuf[OISERIAL_CMD_IDX + 2]));
    
  } 
  else if (cmd == OISERIAL_CMD_PIXEL_SET) {
    pixel_id = oiMsgBuf[OISERIAL_CMD_IDX + 1]; 
    pixel_r = oiMsgBuf[OISERIAL_CMD_IDX + 2]; 
    pixel_g = oiMsgBuf[OISERIAL_CMD_IDX + 3]; 
    pixel_b = oiMsgBuf[OISERIAL_CMD_IDX + 4];
    pixelSetOne(pixel_id, pixel_r, pixel_g, pixel_b);
  }

  else if (cmd == OISERIAL_CMD_ALLPIXEL_SET) { 
    color.r = oiMsgBuf[OISERIAL_CMD_IDX + 1]; 
    color.g = oiMsgBuf[OISERIAL_CMD_IDX + 2]; 
    color.b = oiMsgBuf[OISERIAL_CMD_IDX + 3];
    pixelSetAll(color);
  } 

  else
  {
    char buf[20];
    lcd.setCursor(0, 3);
    lcd.print("BadCmd:");
    lcd.print(itoa(cmd, buf, 10));
  }
}

void readSerialData() {
  if (!Serial.available())
  {
    return; // No data to process
  }

  oiMsgBuf[oiMsgBufIdx] = Serial.read();
  char buf[20];

  //lcd.setCursor(16, 3);
  //lcd.print(itoa(oiMsgBuf[oiMsgBufIdx], buf, 10));

  switch (oiMsgBufIdx)
  {
  case OISERIAL_START_IDX:
    // Check for receiving an invalid start byte
    if (oiMsgBuf[oiMsgBufIdx] != OISERIAL_START_BYTE)
      {
        //lcd.setCursor(0, 1);
        //lcd.print("ST");
        sendNack();
        return;
      }
    break;

  case OISERIAL_SIZE_IDX:
    if (oiMsgBuf[oiMsgBufIdx] < OISERIAL_MIN_MSG_SIZE ||
        oiMsgBuf[oiMsgBufIdx] > OISERIAL_MAX_MSG_SIZE)
    {
      //lcd.setCursor(0, 1);
      //lcd.print("SZ");
      sendNack();
      return;
    }
    oiMsgBytesRemaining = oiMsgBuf[oiMsgBufIdx] - 2; // Subtract off the start and size bytes
    break;

  case OISERIAL_CMD_IDX:
    if (oiMsgBuf[oiMsgBufIdx] > OISERIAL_MAX_CMD_VAL)
    {
      //lcd.setCursor(0, 1);
      //lcd.print("CMD");
      sendNack();
      return;
    }
    oiMsgBytesRemaining--;
    break;

  default:
    oiMsgBytesRemaining--;
    break;
  }

  /* lcd.setCursor(0, 2);
  lcd.print("    ");
  lcd.setCursor(0, 2);
  lcd.print(itoa(oiMsgBytesRemaining, buf, 10)); */

  if (oiMsgBytesRemaining == 0)
  {
    // At end of message.  First verify checksum byte (at end) matches calculated checksum
    // Note: Subtract 1 from message size so as to not include the checksum byte in the CRC8 calculation 
    if (oiMsgBuf[oiMsgBuf[OISERIAL_SIZE_IDX]-1] != CRC8(oiMsgBuf, (oiMsgBuf[OISERIAL_SIZE_IDX] - 1)))
    {
      lcd.setCursor(0, 1);
      lcd.print("CKSUM BAD");
      sendNack();
      return;
    }
    //lcd.setCursor(0, 0);
    //lcd.print("CKSUM:");
    //byte checksum = CRC8(oiMsgBuf, (oiMsgBuf[OISERIAL_SIZE_IDX] - 1));
    //lcd.print(itoa(checksum, buf, 10));
    processCmd();
    sendAck();
  }
  else
  {
    oiMsgBufIdx++;
  }
}
