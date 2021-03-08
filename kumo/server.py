import asyncio
import websockets

from .session import Session

WebSocket = websockets.WebSocketServerProtocol


class Server:

  def __init__(self, host: str = 'localhost', port: int = 8080):
    self.host = host
    self.port = port

  async def listen(self, websocket: WebSocket, path: str):
    session = Session(websocket)
    await session.listen()

  def run(self):
    print('Starting server on %s with port %d...' % (self.host, self.port))
    websocket = websockets.serve(self.listen, self.host, self.port)

    asyncio.get_event_loop().run_until_complete(websocket)
    asyncio.get_event_loop().run_forever()
