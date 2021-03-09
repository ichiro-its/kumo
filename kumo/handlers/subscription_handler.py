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

    def destroy(self, subscription_id: str) -> str:
        subscription = self.get_by_id(subscription_id)

        subscription.destroy()

        return subscription_id
