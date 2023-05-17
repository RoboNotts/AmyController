import asyncio
import socket

from starlette.responses import PlainTextResponse


async def amy_cmd_proxy(request):
    address = "10.42.0.1"
    port = 12306
    with socket.create_connection((address, port)) as sock:
        sock.send(request.query_params.get("cmd").encode("ASCII"))
        return PlainTextResponse(sock.recv(4096).decode("ASCII"))
