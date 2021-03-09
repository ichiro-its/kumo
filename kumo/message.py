from enum import Enum
import json
from rclpy.logging import get_logger


class MessageType(Enum):
    CREATE_NODE = 'CREATE_NODE'
    CREATE_SUBSCRIPTION = 'CREATE_SUBSCRIPTION'
    DESTROY_SUBSCRIPTION = 'DESTROY_SUBSCRIPTION'
    SUBSCRIPTION_MESSAGE = 'SUBSCRIPTION_MESSAGE'


class Message:

    id_counter: int = 0

    def __init__(self, type: MessageType, content: dict = {}, id: str = None):
        self.type = type
        self.content = content

        if id == None:
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
