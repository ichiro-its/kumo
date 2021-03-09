from typing import Dict, List

from kumo.message import Message, MessageType


class BaseHandler:

    id_counter: int = 0

    def __init__(self):
        self.id = str(BaseHandler.id_counter)
        BaseHandler.id_counter += 1

        self.parent = None
        self.childs: Dict[str, BaseHandler] = {}

        self.messages: List[Message] = []

    def destroy(self) -> None:
        if self.parent is not None:
            parent = self.parent
            self.parent = None

            parent.detach(self.id)

        for child in self.childs.values():
            child: BaseHandler = child
            if child is not None:
                child.parent = None
                child.destroy()

    def attach(self, child) -> None:
        child.parent = self
        self.childs[child.id] = child

    def detach(self, id: str) -> None:
        child: BaseHandler = self.childs.get(id, None)
        if child is not None:
            if child.parent is not None:
                child.parent = None
                child.destroy()

    def send(self, message: Message) -> None:
        self.messages.append(message)

    def send_respond(self, request: Message, content: dict) -> None:
        self.send(Message(request.type, content, request.id))

    def send_request(self, type: MessageType, content: dict) -> None:
        self.send(Message(type, content))

    def send_error_respond(self, request: Message, error: Exception) -> None:
        self.send(Message(request.type, {'error': str(error)}, request.id))

    async def process(self) -> None:
        for child in self.childs.values():
            child: BaseHandler = child
            if child is not None:
                await child.process()

                self.messages = self.messages + child.messages
                child.messages.clear()

    async def handle_message(self, message: Message) -> None:
        for child in self.childs.values():
            child: BaseHandler = child
            if child is not None:
                await child.handle_message(message)
