import socket 

BOARD_IP = "192.168.1.50"
BOARD_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)
payload = b"hello stm32 udp echo"
sock.sendto(payload, (BOARD_IP, BOARD_PORT))
data, addr = sock.recvfrom(2048)
print("received:", data)
print("from:", addr)
assert data == payload
print("UDP echo test passed")