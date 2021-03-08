from rclpy.subscription import Subscription
from rosidl_runtime_py.utilities import get_message
from typing import Dict

from .message_handlers.subscription_callback_handler import SubscriptionCallbackHandler
from .node_handler import NodeHandler

SubCallbackHandler = SubscriptionCallbackHandler


class SubscriptionHandler():

    def __init__(self, node_handler: NodeHandler):
        self.node_handler = node_handler
        self.sub_callback_handler = SubCallbackHandler()

        self.subscriptions: Dict[str, Subscription] = {}
        self.counter: int = 0

    def get_by_id(self, id: str) -> Subscription:
        if id not in self.subscriptions:
            raise Exception('Cannot find Subscriptions with Id', id)

        return self.subscriptions[id]

    def create(self, node_id: str, topic_name: str, message_type: str) -> (str, Subscription):
        node = self.node_handler.get_by_id(node_id)

        sub_id = str(self.counter)
        self.counter += 1

        subscription = node.create_subscription(
            get_message(message_type),
            topic_name,
            lambda message: self.sub_callback_handler.callback(sub_id, message),
            10)

        self.subscriptions[sub_id] = subscription

        return sub_id, subscription
