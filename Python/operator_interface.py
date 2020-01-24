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

last_value = []
arduino_connected = False

#list of commands 
def setLED(id, led_state):
  payload = []
  payload.append(id)
  payload.append(led_state.value)
  arduino.write(assembleMessage(OISerialCommand.LED_SET, payload))
  get_response()

def setMessage(x, y, msg):
  payload = []
  payload.append(x << OISERIAL_LCD_X_SHIFT & (y & OISERIAL_LCD_Y_MASK))
  payload += [bytes(character, encoding='ascii')[0] for character in msg] 
  print ("Payload'{}' " .format(payload[1:]))
  arduino.write(assembleMessage(OISerialCommand.LCD_MSG, payload))
  get_response()

def setPixel(id, r, g, b):
  pass
  
def setAllPixel(r, g, b):
  pass
  
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
        
def update_values(table, key, value, isNew):
    global last_value
    global arduino_connected
    diff = [] # Will have true if the bit changed
    serial_data = []
    if len(last_value) < len(value):
        last_value = [False]*len(value)
    for old, new in zip(last_value, value):
        diff.append(False if old == new else True)
    byte_diff = [diff[i:i + 8-SET_BITS] for i in range(0, len(diff), 8-SET_BITS)]
    send_bits = [value[i:i + 8-SET_BITS] for i in range(0, len(value), 8-SET_BITS)]
    for index, item_diff, bits in zip(range(len(send_bits)), byte_diff, send_bits):
        if True in item_diff:
            # Ensure bits is of the proper length, must be list
            bits = list(bits)
            bits.extend([False]*((8-SET_BITS)-len(bits)))
            byte_index = index<<8-SET_BITS
            data_string = ''.join(['1' if x else '0' for x in bits])
            serial_data.append(int(data_string, base=2)+byte_index)
            print(index, data_string, sep=", ")
    last_value = value
    try:
        arduino.write(bytes(serial_data))
    except SerialException:
        if arduino_connected:
            arduino_connected = False
            print("Lost connection to Arduino")
    except NameError:
        # Arduino has not been connected to
        pass

connect_to_arduino()

# NetworkTables setup
NetworkTables.initialize(server=NTSERVER)
#NetworkTables.enableVerboseLogging()
table = NetworkTables.getTable("LEDs")
print("Connecting to NetworkTables...")
while NetworkTables.getRemoteAddress() is None:
    sleep(1)
print("Connected to NetworkTables")
#table.addTableListener(update_values, immediateNotify=True, key="OI LEDs")

lastStateConnected = True
while True:
    sleep(1)
    """ if not lastStateConnected and NetworkTables.getRemoteAddress() is not None:
        lastStateConnected = True
        print("Re-connected to NetworkTables")
    elif lastStateConnected and NetworkTables.getRemoteAddress() is None:
        lastStateConnected = False
        print("Lost connection to NetworkTables") """

    if not arduino_connected:
        connect_to_arduino();

    for i in range(0, 1):
      setLED(i, OILEDState.LEDSTATE_MED)
    setMessage(2, 0, "aaa"); 
    #setMessage(0, 1, "bbbbbbbbbbbbbbbbbb22"); 
    #setMessage(0, 2, "cccccccccccccccccc33");
    #setMessage(0, 3, "dddddddddddddddddd44");
