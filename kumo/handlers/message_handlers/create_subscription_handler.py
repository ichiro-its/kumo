from ..message_handler import MessageHandler
from ..subscription_handler import SubscriptionHandler


class CreateSubscriptionHandler(MessageHandler):

    def __init__(self, subscription_handler: SubscriptionHandler):
        super().__init__('CREATE_SUBSCRIPTION')

        self.subscription_handler: SubscriptionHandler = subscription_handler

    def handle(self, id: str, content: dict) -> None:
        super().handle(id, content)

        try:
            subscription_id, _ = self.subscription_handler.create(
                str(content['node_id']),
                str(content['topic_name']),
                str(content['message_type']))

            self.response(id, { 'subscription_id': subscription_id })

        except Exception as e:
            print('Failed to handle create subscription!', e)
            self.response(id, { 'error': str(e) })
