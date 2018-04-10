import pygatt
import time
import sys
import serial
import select

BLE_ADDRESS = '58:7a:62:4f:60:44'
UUID = '0000FFE1-0000-1000-8000-00805F9B34FB'

ser = serial.Serial('/dev/ttyUSB3', 115200)

def handle_data(handle, value):
	if(value[:1] == '='):
		print("Data: %s" % value)
	ser.write(value)


def main():
	adapter = pygatt.GATTToolBackend()
	device = None
	is_connected = False	
	while not is_connected:
		try:
			adapter.start()
			device = adapter.connect(BLE_ADDRESS)
			device.subscribe(UUID, callback=handle_data)
			is_connected = True
		except pygatt.exceptions.NotConnectedError as e:
			print("Connection error. Reconnecting...")
	
	print("Connected to {0}".format(BLE_ADDRESS))
	while 1:
		try:
			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				line = sys.stdin.readline()
				device.char_write(UUID, bytearray(line))
		except KeyboardInterrupt:
			ser.close()
			device.disconnect()
			adapter.stop()
			sys.exit()

if __name__ == '__main__':
	main()
