#!/usr/bin/env python3
from time import sleep
from networktables import NetworkTables
from serial import Serial, SerialException
from enum import Enum
import re

"""
This file provides the interface between NetworkTables 
(running on the RoboRio) and the Arduino Leonardo-based 
Operator Interface board.  It supports control of:
- LEDS (to one of 8 specified states)
- Text LCD fields (each with a location, length, and string value)
- Neopixel indicators

The expected NetworkTables structure is as follows:
OperatorInterface/
  LEDs   --> Array of numeric values, one per LED, value of 0-7 each
  LCD/
    <field>/    --> Arbitrary field identifier
      X         --> X location of this field (0 is left)
      Y         --> Y location a/k/a line of this field (0 is top)
      Length    --> Length of this field (will be truncated)
      String    --> Text to display  
    <field 2>/  --> As many fields as you can fit on your LCD,
                    each with the attributes listed above  
"""

NTSERVER = "127.0.0.1"
SERIAL_PORT = "COM7"  # COM7 in Windows
SERIAL_BAUD = 115200

class OISerialCommand(Enum):
  NOOP = 0
  LED_SET = 1
  LCD_MSG = 2
  PIXEL_SET = 3
  ALLPIXEL_SET = 4

class OISerialResponse(Enum):
  ACK = 0xaa
  NACK = 0xff
  ERR = 0xee


class OILEDState(Enum):
  LEDSTATE_OFF = 0x0   # Completely off
  LEDSTATE_BLINK_SLOW = 0x1   # Slowly blinking
  LEDSTATE_BLINK_FAST = 0x2   # Rapidly blinking
  LEDSTATE_PULSE_SLOW = 0x3   # Slowly pulsing
  LEDSTATE_PULSE_FAST = 0x4   # Rapidly pulsing
  LEDSTATE_DIM = 0x5   # On dim
  LEDSTATE_MED = 0x6   # On medium
  LEDSTATE_ON = 0x7   # On bright

OISERIAL_STARTBYTE = 0xc3
OISERIAL_MSG_BASE_LEN = 4   # min msg overhead: start + size + cmd + checksum
OISERIAL_LCD_X_MASK = 0xf8
OISERIAL_LCD_Y_MASK = 0x07
OISERIAL_LCD_X_SHIFT = 3

last_led_value = []
last_lcd_value = {}
arduino_connected = False
arduino = None

def arduinoWrite(serial_data):
  global arduino_connected
  try:
    arduino.write(bytes(serial_data))
  except SerialException:
    if arduino_connected:
      arduino_connected = False
      print("Lost connection to Arduino")
  except NameError:
    # Arduino has not been connected to
    pass

# Command methods for each operation that is supported
def setLED(id, led_state):
  """Set one LED to the specified state"""
  payload = []
  payload.append(id)
  payload.append(led_state)
  arduinoWrite(assembleMessage(OISerialCommand.LED_SET, payload))
  get_response()


def setMessage(x, y, msg):
  """Set an LCD text string at the specified x,y location"""
  payload = []
  payload.append(int(x) << OISERIAL_LCD_X_SHIFT | (int(y) & OISERIAL_LCD_Y_MASK))
  payload += [bytes(character, encoding='ascii')[0] for character in msg]
  #print ("Payload'{}' " .format(payload[1:]))
  arduinoWrite(assembleMessage(OISerialCommand.LCD_MSG, payload))
  get_response()


def setPixel(id, r, g, b):
  """Set a single neopixel to the RGB value specified"""
  payload = [id, r, g, b]
  arduinoWrite(assembleMessage(OISerialCommand.PIXEL_SET, payload))
  get_response()


def setAllPixel(r, g, b):
  """Set all neopixels to the RGB value specified"""
  payload = [r, g, b]
  arduinoWrite(assembleMessage(OISerialCommand.ALLPIXEL_SET, payload))
  get_response()


def keepAlive():
  payload = []
  arduinoWrite(assembleMessage(OISerialCommand.NOOP, payload))
  get_response()


# common code
def get_response():
  """
  Read response (ack/nack) and any data returned.

  Returns: tuple of response value (ACK/NACK or None) and list holding extra data
  """
  global arduino_connected
  global arduino
  resp_data = []
  resp_status = None
  resp_val = OISerialResponse.NACK.value # Fail by default

  # ACK or NACK is the first (often only) byte
  try:
    for tries in range(10, 0, -1):
      if arduino.in_waiting:
        resp = arduino.read(1)
        resp_val = int.from_bytes(resp, 'little')  # No real endianness as it's one byte!
        break

      elif tries:
        # No data, wait and retry
        #print("No response, retrying")
        sleep(0.02)
  except SerialException:
    arduino_connected = False
    resp_val = OISerialResponse.ERR.value

  if resp_val == OISerialResponse.ACK.value:
    resp_status = OISerialResponse.ACK
    #print("Got ACK")
  elif resp_val == OISerialResponse.NACK.value:
    resp_status = OISerialResponse.NACK
    print("Got NACK")
  elif resp_val == OISerialResponse.ERR.value:
    resp_status = OISerialResponse.ERR
    print("Serial Error")
  else:
    print("Got unknown status: ", resp, "with int value", resp_val)

  # Read any response data
  if resp_status != OISerialResponse.ERR:
    while arduino.in_waiting:
      resp_data.append(arduino.read(1))

  return (resp_status, resp_data)


def assembleMessage(cmd, payload):
  """
  Assemble a serial message to send to the Arduino.

  This function builds a complete command message including a 
  calculated checksum.

  Args:
     cmd: an OISerialCommand
     payload: bytes to send with the command.

  Returns:
      List of bytes representing the command.
  """
  serial_data = []
  if type(cmd) != OISerialCommand:
    print(f"ERROR: invalid command '{cmd}'")
    return serial_data

  serial_data.append(OISERIAL_STARTBYTE)
  serial_data.append(len(payload) + OISERIAL_MSG_BASE_LEN)
  serial_data.append(cmd.value)
  for byte in payload:
    serial_data.append(byte)
  crc8 = calcCRC8(serial_data)
  serial_data.append(crc8)
  #print("Built message:", bytes(serial_data))
  return serial_data

def calcCRC8(msg):
  crc = 0x00
  msglen = len(msg)
  msgidx = 0
  while (msglen > 0):
    extract = msg[msgidx]
    for tempI in range (8, 0, -1):
      sum = (crc ^ extract) & 0x01
      crc >>= 1
      if sum:
        crc ^= 0x8C
      extract >>= 1
    msglen -= 1
    msgidx += 1
  #print("Calculated checksum: ",crc)
  return crc

def connect_to_arduino():
  global arduino
  global arduino_connected
  print("Connecting to Arduino")
  # serial port setup
  try:
    arduino = Serial(SERIAL_PORT, SERIAL_BAUD)
    arduino.reset_input_buffer()
    # opening connection causes Arduino to reset and not receive data for a few seconds
    sleep(2)
    arduino_connected = True
    print("Connected to Arduino")
    refresh_status()

  except SerialException:
    print("Got serial exception - check USB connection")


def refresh_status():
  """
  Force a refresh of all of the output status.
  This is needed after the Arduino reconnects.
  """
  global table
  global lcd_table
  # Send a keepAlive so the Arduino knows we're connected and will clear 
  # any "not connected" message
  keepAlive()
  # Invoke the handlers that actually carry out the updates.  Set the "force" kwarg 
  # so that they will always resend data even if it hasn't changed from previous values.
  try:
    update_led_values(table, "LEDs", table.getNumberArray("LEDs", 0), False, force=True)
    for subtable in lcd_table.getSubTables():
      update_lcd_values_entry(NetworkTables.getTable("OperatorInterface/LCD/" + subtable),
                              "", "", False, force=True)
  except NameError:
    # Before NT init
    pass


def update_led_values(table, key, value, isNew, force=False):
    """
    Update LEDs to correct state according to current values.
    """
    global last_led_value
    if len(last_led_value) < len(value):
        last_led_value = [0]*len(value)
    for i in range(0, len(value)):
      if last_led_value[i] != value[i] or force:
        last_led_value[i] = int(value[i])
        setLED(i, int(value[i]))


def update_lcd_values_entry(source, key, value, isNew, force=False):
    """Update an LCD text field to current value.

    This function compares the field's attributes (location and length) 
    and if anything has changed, it first blanks the old field, then 
    redraws with new content.  If the string has changed it displays the
    new string, blanking to the end of the field.

    Displayed strings are truncated to the current length value.

    It is up to the user to specify lengths that actually fit on the screen.
    No attempt is made to detect screen overruns or overlapping fields.
    """
    global last_lcd_value
    subtable_id = re.search("OperatorInterface/LCD/(.*)/", str(source))
    if subtable_id:
      field_id = subtable_id.group(1)
    else:
      return

    newdata = {}
    # If "force" is in effect, we pretend there's a property change to force
    # a redraw to occur.
    field_propchange = force
    for attr in ['X', 'Y', 'Length']:
      # Networktables provides floating point, cast these to integers
      newdata[attr] = int(source.getValue(attr, 0))
      # If any of these attributes changed, we need to clear the old string
      if last_lcd_value[field_id][attr] != newdata[attr]:
        field_propchange = True

    if field_propchange:
      # Overwrite the 'old' field with a string of spaces
      clear_str = ' ' * last_lcd_value[field_id]['Length']
      setMessage(last_lcd_value[field_id]['X'], last_lcd_value[field_id]['Y'], clear_str)

    # Set the new string if it has changed, or if properties changed
    newdata['String'] = source.getValue('String', '')[0:newdata['Length']]
    if newdata['String'] != last_lcd_value[field_id]['String'] or field_propchange:
      setMessage(newdata['X'], newdata['Y'], newdata['String'].ljust(newdata['Length']))

    # Store current so we can detect future changes
    last_lcd_value[field_id] = newdata


def setup_networktables_lcd(source, key, mysubtable, boolarg):
  """
  Register an entry listener for each entry in the LCD subtable.

  The LCD subtables each represent one text field, with attributes for the 
  X/Y position, length, and the string value.  Any time there is a change,
  this function re-registers the listeners and re-initializes the "last seen"
  data so that a full update will occur.
  """
  for subtable in lcd_table.getSubTables():
    NetworkTables.getTable("OperatorInterface/LCD/" + subtable).addEntryListener(update_lcd_values_entry, immediateNotify=True)
    last_lcd_value.update({subtable: {"X":0, "Y":0, "Length":0, "String":""}})

#
# MAIN
#

# First connect to the Arduino
connect_to_arduino()

# NetworkTables setup
NetworkTables.initialize(server=NTSERVER)
#NetworkTables.enableVerboseLogging()
table = NetworkTables.getTable("OperatorInterface")
lcd_table = NetworkTables.getTable("OperatorInterface/LCD")
print("Connecting to NetworkTables...")
while NetworkTables.getRemoteAddress() is None:
    sleep(1)
print("Connected to NetworkTables")
# Listener to handle changes to the LED entries
table.addEntryListener(update_led_values, immediateNotify=True, key="LEDs")
# Listener for changes (including additions) to LCD entries
lcd_table.addSubTableListener(setup_networktables_lcd, localNotify=True)

lastStateConnected = True

# This loop runs continuously, sending keepalives to maintain an active 
# connection to the Arduino and reconnecting to NetworkTables and/or
# the Arduino as required.
while True:
  sleep(1)
  if not lastStateConnected and NetworkTables.getRemoteAddress() is not None:
    lastStateConnected = True
    print("Re-connected to NetworkTables")
  elif lastStateConnected and NetworkTables.getRemoteAddress() is None:
    lastStateConnected = False
    print("Lost connection to NetworkTables")

  if not arduino_connected:
    connect_to_arduino();
  else:
    keepAlive()
