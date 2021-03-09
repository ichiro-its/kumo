from rclpy.logging import get_logger

from kumo.handlers.base_handler import BaseHandler
from kumo.handlers.node_handler import NodeHandler
from kumo.message import Message, MessageType


class ContextHandler(BaseHandler):

    def __init__(self):
        super().__init__()

        self.logger = get_logger('context_handler')

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.CREATE_NODE:
            try:
                return self.handle_create_node(message)

            except Exception as e:
                self.logger.error('Failed to create a Node! %s' % str(e))
                self.send_error_respond(message, e)

        await super().handle_message(message)

    def handle_create_node(self, message: Message) -> None:
        handler = NodeHandler(message.content.get('node_name'))
        self.attach(handler)

        self.logger.info('Node %s created!' % handler.id)
        self.send_respond(message, {'node_id': handler.id})
