import socket

PORT = 4444
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("", PORT))

print(f"Listening for ESP32 logs on UDP port {PORT}...")
while True:
    data, addr = sock.recvfrom(256)
    print(data.decode("utf-8", errors="replace"), end="", flush=True)
