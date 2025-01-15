import os
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StreamlitUISubscriberNode(Node):

    def __init__(self):
        super().__init__('streamlit_subscriber_node')
        # Create subscriber
        self.subscriber_ = self.create_subscription(String, 'completion',
                              self.listener_callback, 10)
        # Initialize message
        self.msg = String()
        # Configure socket for data exchange with streamlit and connect
        self.socket_path = '/tmp/st_socket.socket'
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    def listener_callback(self, msg):
        self.get_logger().info('Agent response: "%s"' % msg.data)
        self.client.sendall(msg.data.encode())

    def connect_to_server(self):
        while True:
            try:
                self.client.connect(self.socket_path)
                self.client.send('subscriber'.encode())
                self.get_logger().info('UI subscriber client connected')
                break  # Successfully connected
            except ConnectionRefusedError:
                self.get_logger().info("UI subscriber client connection refused, retrying...")
                time.sleep(1)

def main():
    rclpy.init()
    ui_sub_node = StreamlitUISubscriberNode()
    ui_sub_node.connect_to_server()
    rclpy.spin(ui_sub_node)
    ui_sub_node.client.close()
    ui_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
