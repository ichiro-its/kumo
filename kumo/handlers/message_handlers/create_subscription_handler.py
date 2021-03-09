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

from ..message_handler import MessageHandler
from ..subscription_handler import SubscriptionHandler


class CreateSubscriptionHandler(MessageHandler):

    def __init__(self, subscription_handler: SubscriptionHandler):
        super().__init__('CREATE_SUBSCRIPTION')

        self.subscription_handler = subscription_handler

    def handle(self, id: str, content: dict) -> None:
        super().handle(id, content)

        try:
            subscription_id, _ = self.subscription_handler.create(
                str(content['node_id']),
                str(content['topic_name']),
                str(content['message_type']))

            self.response(id, {'subscription_id': subscription_id})

        except Exception as e:
            print('Failed to handle create subscription!', e)
            self.response(id, {'error': str(e)})
