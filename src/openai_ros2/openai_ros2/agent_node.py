import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class GPT4Node(Node):

    def __init__(self):
        super().__init__('gpt4_node')
        self.subscription = self.create_subscription(
            String,
            'user_query',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'completion', 10)

        try:
            openai.OpenAI.api_key = os.environ['OPENAI_API_KEY']
            models = openai.Model.list() # Just to make sure your key is valid
            self.get_logger().info('API key successfully loaded')
        except Exception as e:
            self.get_logger().error('OPENAI_API_KEY is invalid.')

        try:
            self.chat_client = openai.OpenAI()
        except Exception as e:
            self.get_logger().error('Chat completion model not instantiated.')

    def listener_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)
        system_message, user_message = self.assemble_prompt(msg.data)
        try:
            response = self.chat_client.chat.completions.create(
              model="gpt-4o-mini",
              messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": user_message},
              ],
              max_tokens=150
            )
            completion = response.choices[0].message.content.strip()
            self.get_logger().info('Agent Response: "%s"' % completion)
            self.publisher_.publish(String(data=completion))
        except Exception as e:
            self.get_logger().error('Error in GPT-4 API call: %s' % str(e))

    def assemble_prompt(self, user_input):
        system_message = "You are a helpful assistant."
        user_message = user_input
        return system_message, user_message

def main(args=None):
    rclpy.init(args=args)
    gpt4_node = GPT4Node()
    rclpy.spin(gpt4_node)
    gpt4_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
