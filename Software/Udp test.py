import socket
import time

UDP_IP = "10.0.0.88"
UDP_PORT = 1234
MESSAGE = b"Hello, World!"


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


for i in range(10000):
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_PORT)
    print("message: %s" % MESSAGE)
    time.sleep(1)
    data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
    print("received message from: %s" % str(addr))
    print("received message: %s" % data)
    time.sleep(0.5)
