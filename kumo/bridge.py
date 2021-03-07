import asyncio
import json
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import websockets
from websockets import WebSocketServerProtocol

class Bridge:

    def __init__(self, websocket: WebSocketServerProtocol):
        self.websocket: WebSocketServerProtocol = websocket

        self.nodes: dict = {}
        self.nodes_it: int = 0

        self.subscriptions: dict = {}
        self.subscriptions_it = 0

        self.messages: list = []

    def __del__(self):
        for key in self.nodes:
            self.nodes[key].destroy_node()


    def send(self, message):
        self.messages.append(json.dumps(message))


    async def listen(self):
        while True:
            for key in self.nodes:
                rclpy.spin_once(self.nodes[key], timeout_sec=0.01)

            while len(self.messages) > 0:
                await self.websocket.send(self.messages.pop())

            try:
                request_json = await asyncio.wait_for(self.websocket.recv(), 0.01)
                try:
                    request = json.loads(request_json)
                    self.handle_request(request)
                except Exception as e:
                    print('Failed to parse request!', e)
            except asyncio.TimeoutError:
                pass
            except websockets.ConnectionClosed:
                print('Closed!')
                return
            except Exception as e:
                print('Something happened...', e)


    def handle_request(self, request: dict):
        if request['type'] == 'CREATE_NODE':
            try:
                node_id = self.create_node(request['nodeName'])

                self.send({
                    'type': request['type'],
                    'id': request['id'],
                    'nodeId': node_id})

            except Exception as e:
                self.send({
                    'type': request['type'],
                    'id': request['id'],
                    'error': str(e)})

        if request['type'] == 'CREATE_SUBSCRIPTION':
            try:
                if request['nodeId'] in self.nodes:
                    subscription_id = self.create_subscription(
                        self.nodes[request['nodeId']],
                        request['topicName'],
                        get_message(request['messageType']))

                    self.send({
                        'type': request['type'],
                        'id': request['id'],
                        'subscriptionId': subscription_id})

                else:
                    self.send({
                        'type': request['type'],
                        'id': request['id'],
                        'error': 'Invalid node!'})

            except Exception as e:
                self.send({
                    'type': request['type'],
                    'id': request['id'],
                    'error': str(e)})


    def create_node(self, node_name: str) -> int:
        node_id = self.nodes_it
        self.nodes_it += 1

        self.nodes[node_id] = Node(node_name)
        return node_id


    def create_subscription(self, node: Node, topic_name: str, message_type) -> int:
        subscription_id = self.subscriptions_it
        self.subscriptions_it += 1

        node.create_subscription(
            message_type, topic_name,
            lambda message: self.subscription_callback(subscription_id, message),
            10)

        return subscription_id

    def subscription_callback(self, subscription_id, message):
        fields = message.get_fields_and_field_types()

        message_dict = {}
        for field in fields:
            if hasattr(message, field):
                message_dict[field] = getattr(message, field)

        self.send({
            'type': 'SUBSCRIPTION_CALLBACK',
            'subscriptionId': subscription_id,
            'message': json.dumps(message_dict)})


async def on_connect(websocket: WebSocketServerProtocol, path: str):
    print('Connected!')

    bridge = Bridge(websocket)
    await bridge.listen()

def main(args=None):
    rclpy.init(args=args)

    server = websockets.serve(on_connect, 'localhost', 8080)

    print('Started WebSocket bridge on port 8080')
    asyncio.get_event_loop().run_until_complete(server)
    asyncio.get_event_loop().run_forever()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
