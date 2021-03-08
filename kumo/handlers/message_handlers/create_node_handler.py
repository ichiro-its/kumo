from ..message_handler import MessageHandler
from ..node_handler import NodeHandler


class CreateNodeHandler(MessageHandler):

    def __init__(self, node_handler: NodeHandler):
        super().__init__('CREATE_NODE')

        self.node_handler: NodeHandler = node_handler

    def handle(self, id: str, request: dict) -> None:
        super().handle(id, request)

        try:
            node_name: str = str(request['node_name'])
            node_id, _ = self.node_handler.create(node_name)

            self.response(id, { 'node_id': node_id })

        except Exception as e:
            print('Failed to handle create node!', e)
            self.response(id, { 'error': str(e) })
