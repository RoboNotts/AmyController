import socket

def send_map_start_cmd(address: str, port: int):
    with socket.create_connection((address, port)) as sock:
        sock.send("#MAPO")
        sock.send("#NAVS")
