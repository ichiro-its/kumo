import asyncio
import json
from typing import Dict, List
import websockets

from .handlers import CreateNodeHandler, CreateSubscriptionHandler, MessageHandler, NodeHandler, SubscriptionHandler

WebSocket = websockets.WebSocketServerProtocol
CreateSubHandler = CreateSubscriptionHandler
SubHandler = SubscriptionHandler


class Session:

    def __init__(self, websocket: WebSocket):
        self.websocket = websocket

        self.node_handler = NodeHandler()
        self.sub_handler = SubHandler(self.node_handler)

        self.message_handlers: Dict[str, MessageHandler] = {}
        for message_handler in [
            CreateNodeHandler(self.node_handler),
            CreateSubHandler(self.sub_handler),
            self.sub_handler.sub_callback_handler]:

            self.message_handlers[message_handler.type] = message_handler

    async def listen(self):
        print('Session started!')

        while True:
            try:
                self.node_handler.spin()

                for message_handler in self.message_handlers.values():
                    await message_handler.send(lambda message: self.websocket.send(message))

                try:
                    while True:
                        await self.handle_message()

                except asyncio.TimeoutError:
                    pass

            except websockets.ConnectionClosed:
                print('Session closed!')
                return

            except Exception as e:
                print('Something happened!', e)

    async def handle_message(self) -> None:
        message_string = await asyncio.wait_for(self.websocket.recv(), 0.01)

        try:
            message = json.loads(message_string)

            message_type = str(message['type'])
            message_id = str(message['id'])

            if message_type in self.message_handlers:
                message_handler = self.message_handlers[message_type]
                message_handler.handle(message_id, message)

        except Exception as e:
            print('Failed to parse request!', e)
