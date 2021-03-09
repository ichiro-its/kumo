# Copyright (c) 2021 Ichiro ITS
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from typing import Dict, List

from kumo.message import Message, MessageType


class BaseHandler:

    id_counter: int = 0

    def __init__(self):
        self.id = str(BaseHandler.id_counter)
        BaseHandler.id_counter += 1

        self.active = True

        self.parent = None
        self.childs: Dict[str, BaseHandler] = {}

        self.messages: List[Message] = []

    def destroy(self) -> bool:
        if self.active is True:
            self.active = False

            for child in self.childs.values():
                child: BaseHandler = child
                child.destroy()

            return True

        return False

    def attach(self, child) -> None:
        child.parent = self
        self.childs[child.id] = child

        if self.active is False:
            child.destroy()

    def detach(self, id: str) -> None:
        child: BaseHandler = self.childs.get(id, None)
        del self.childs[id]

        if child is not None:
            if child.active is True:
                child.destroy()

            child.parent = None

    def send(self, message: Message) -> None:
        self.messages.append(message)

    def send_respond(self, request: Message, content: dict) -> None:
        self.send(Message(request.type, content, request.id))

    def send_request(self, type: MessageType, content: dict) -> None:
        self.send(Message(type, content))

    def send_error_respond(self, request: Message, error: Exception) -> None:
        self.send(Message(request.type, {'error': str(error)}, request.id))

    async def process(self) -> None:
        inactives = []
        for id, child in self.childs.items():
            child: BaseHandler = child

            self.messages = self.messages + child.messages
            child.messages.clear()

            if child.active:
                await child.process()
            else:
                inactives.append(id)

        for id in inactives:
            self.detach(id)

    async def handle_message(self, message: Message) -> None:
        for child in self.childs.values():
            child: BaseHandler = child
            if child.active:
                await child.handle_message(message)
