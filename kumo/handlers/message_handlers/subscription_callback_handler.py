import json
from ..message_handler import MessageHandler
from rclpy.subscription import MsgType


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
