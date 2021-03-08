import json
from typing import List, Callable


class MessageHandler():

    def __init__(self, type: str):

        self.type: str = type
        self.id_counter: int = 0

        self.messages: List[str] = []

    async def send(self, callback: Callable) -> None:
        while len(self.messages) > 0:
            await callback(self.messages.pop())

    def request(self, content: dict) -> None:
        content['type'] = self.type

        content['id'] = str(self.id_counter)
        self.id_counter += 1

        self.messages.append(json.dumps(content))

    def response(self, id: str, content: dict) -> None:
        content['type'] = self.type
        content['id'] = id

        self.messages.append(json.dumps(content))

    def handle(self,  id: str, content: dict) -> None:
        pass
