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

import json
from rclpy.subscription import MsgType

from ..message_handler import MessageHandler


class SubscriptionCallbackHandler(MessageHandler):

    def __init__(self):
        super().__init__('SUBSCRIPTION_CALLBACK')

    def handle(self, id: str, content: dict) -> None:
        super().handle(id, content)

    def callback(self, subscription_id: str, message: MsgType):
        try:
            fields = message.get_fields_and_field_types()

            message_dict = {}
            for field in fields:
                if hasattr(message, field):
                    message_dict[field] = getattr(message, field)

            self.request({
                'subscription_id': subscription_id,
                'message': json.dumps(message_dict)})

        except Exception as e:
            print('Failed to handle subscription callback!', e)
