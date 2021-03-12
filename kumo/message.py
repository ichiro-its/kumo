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

from enum import Enum
import json


class MessageType(Enum):
    CREATE_NODE = 'CREATE_NODE'
    DESTROY_NODE = 'DESTROY_NODE'
    CREATE_PUBLISHER = 'CREATE_PUBLISHER'
    DESTROY_PUBLISHER = 'DESTROY_PUBLISHER'
    PUBLISHER_MESSAGE = 'PUBLISHER_MESSAGE'
    CREATE_SUBSCRIPTION = 'CREATE_SUBSCRIPTION'
    DESTROY_SUBSCRIPTION = 'DESTROY_SUBSCRIPTION'
    SUBSCRIPTION_MESSAGE = 'SUBSCRIPTION_MESSAGE'


class Message:

    id_counter: int = 0

    def __init__(self, type: MessageType, content: dict = {}, id: str = None):
        self.type = type
        self.content = content

        if id is None:
            self.id = str(Message.id_counter)
            Message.id_counter += 1
        else:
            self.id = str(id)

    def toString(self) -> str:
        message = {'type': self.type.value, 'id': self.id, 'content': self.content}
        return json.dumps(message)


def parse_message(message: str) -> Message:
    message_json: dict = json.loads(message)
    return Message(MessageType(message_json.get('type')),
                   message_json.get('content', {}),
                   message_json.get('id', None))
