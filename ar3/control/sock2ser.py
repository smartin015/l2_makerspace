import socket
import serial
import threading

ser = serial.Serial("/dev/ttyACM0", 115200)
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 20000))

def port_echo(ser):
  while True:
    print(ser.readline().decode())

threading.Thread(target=port_echo, args=(ser,), daemon=True).start()
while True:
  gcode = sock.recv(512)
  ser.write(gcode + bytes("\n", "utf-8"))
