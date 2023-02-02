#!/usr/bin/env python3

import rclpy.node
from std_msgs.msg import Bool
import chime
chime.theme('mario')
import telebot
import os

class CoffeeFeedback(rclpy.node.Node):
    def __init__(self):
        super().__init__("coffee_feedback")
        self.audio_feed = self.declare_parameter('audio_feedback', True).value
        self.telegram_feed = self.declare_parameter('telegram_feedback', False).value
        if self.telegram_feed:
            token_f = os.path.join(os.path.dirname(os.path.realpath(__file__)), ".token")
            with open(token_f) as f:
                token = f.read().strip()
            self.bot = telebot.TeleBot(token)
        self.create_subscription(Bool, 'audio_label', self.has_received_data, 10)

    def has_received_data(self, msg):
        if msg.data:
            if self.audio_feed:
                chime.info()
            if self.telegram_feed:
                try:
                    self.bot.send_message(224534081, "coffee at IDSIA")
                except Exception as e:
                    pass
                    # self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = CoffeeFeedback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()