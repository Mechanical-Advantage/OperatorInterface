#!/usr/bin/env python3
from time import sleep

from networktables import NetworkTables
from serial import Serial, SerialException
from enum import Enum

NTSERVER = "127.0.0.1"
SERIAL_PORT = "COM3"  # COM7 in Windows
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
arduino_connected = False

#list of commands 
def setLED(id, led_state):
  payload = []
  payload.append(id)
  payload.append(led_state)
  arduino.write(assembleMessage(OISerialCommand.LED_SET, payload))
  get_response()

def setMessage(x, y, msg):
  payload = []
  payload.append(x << OISERIAL_LCD_X_SHIFT | (y & OISERIAL_LCD_Y_MASK))
  payload += [bytes(character, encoding='ascii')[0] for character in msg] 
  print ("Payload'{}' " .format(payload[1:]))
  arduino.write(assembleMessage(OISerialCommand.LCD_MSG, payload))
  get_response()

def setPixel(id, r, g, b):
  payload = [id, r, g, b]
  arduino.write(assembleMessage(OISerialCommand.PIXEL_SET, payload)) 
  get_response() 

def setAllPixel(r, g, b):
  payload = [r, g, b]  
  arduino.write(assembleMessage(OISerialCommand.ALLPIXEL_SET, payload))
  get_response() 

def keepAlive():   
  payload = []
  arduino.write(assembleMessage(OISerialCommand.NOOP, payload))
  get_response()

# common code
def get_response():
  """
  Read response (ack/nack) and any data returned.

  Returns: tuple of response value (ACK/NACK or None) and list holding extra data
  """
  resp_data = []
  resp_status = None
  resp_val = OISerialResponse.NACK.value # Fail by default

  # ACK or NACK is the first (often only) byte
  for tries in range(10, 0, -1):
    if arduino.in_waiting:
      resp = arduino.read(1)
      resp_val = int.from_bytes(resp, 'little')  # No real endianness as it's one byte!
      break

    elif tries:
      # No data, wait and retry
      print("No response, retrying")
      sleep(0.02)

  if resp_val == OISerialResponse.ACK.value:
    resp_status = OISerialResponse.ACK
    print("Got ACK")
  elif resp_val == OISerialResponse.NACK.value:
    resp_status = OISerialResponse.NACK
    print("Got NACK")
  else:
    print("Got unknown status: ", resp, "with int value", resp_val)

  # Read any response data
  while arduino.in_waiting:
    resp_data.append(arduino.read(1))

  return (resp_status, resp_data)

def assembleMessage(cmd, payload):
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
  print("Built message:", bytes(serial_data))
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
  print("Calculated checksum: ",crc)
  return crc

def connect_to_arduino():
    print("Connecting to Arduino")
    # serial port setup
    try:
        global arduino
        arduino = Serial(SERIAL_PORT, SERIAL_BAUD)
        arduino.reset_input_buffer()
        # opening connection causes Arduino to reset and not recive data for a few seconds
        sleep(2)
        global arduino_connected
        arduino_connected = True
        print("Connected to Arduino")
        global last_value
        last_value = [] # Force resend of all LEDs
        global table
        try:
            update_values(table, "OI LEDs", table.getBooleanArray("OI LEDs"), False)
        except NameError:
            # Before NT init
            pass
    except SerialException:
        print("Got serial exception")
        raise
        
def update_led_values(table, key, value, isNew):
    global last_led_value 
    global arduino_connected
    diff = [] # Will have true if the bit changed
    serial_data = []
    if len(last_led_value) < len(value): 
        last_led_value = [0]*len(value) 
    for i in range(0, len(value)):  
      if last_led_value[i] != value[i]: 
        last_led_value[i] = int(value[i])
        setLED(i, int(value[i]))  

def update_lcd_values_entry(source, key, value, isNew): 
    global last_x_value  
    global last_y_value 
    global last_length_value 
    global last_string_value
    global arduino_connected 
    diff = [] 
    serial_data = [] 
    print("Update LCD value", "source", source, "key", key, "value", value)   
    subtable_entries = {  
      "X": "", 
      "Y": "", 
      "Length": "",  
      "String": "",
    } 
    for entry in subtable_entries: 
      if last_lcd_value[entry] != value[entry] 
      last_lcd_value[entry] = value[entry]
    
        

    """ try:
        arduino.write(bytes(serial_data))
    except SerialException:
        if arduino_connected:
            arduino_connected = False
            print("Lost connection to Arduino")
    except NameError:
        # Arduino has not been connected to
        pass """

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
for subtable in lcd_table.getSubTables():  
  print(subtable)
  NetworkTables.getTable("OperatorInterface/LCD/" + subtable).addEntryListener(update_lcd_values_entry, immediateNotify=True) 

#table.addTableListener(update_values, immediateNotify=True, key="OI LEDs")
table.addEntryListener(update_led_values, immediateNotify=True, key="LEDs")
#lcd_table.addEntryListener(update_lcd_values_entry, immediateNotify=True)

lastStateConnected = True
while True:
  sleep(2)
  if not lastStateConnected and NetworkTables.getRemoteAddress() is not None:
      lastStateConnected = True
      print("Re-connected to NetworkTables")
  elif lastStateConnected and NetworkTables.getRemoteAddress() is None:
        lastStateConnected = False
        print("Lost connection to NetworkTables")

  if not arduino_connected:
      connect_to_arduino();

  keepAlive(); 