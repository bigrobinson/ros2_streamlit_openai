import os
import socket
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StreamlitUIPublisherNode(Node):

    def __init__(self):
        super().__init__('streamlit_publisher_node')
        # Create publisher for user query
        self.query_publisher_ = self.create_publisher(String, 'user_query', 10)
        # Create timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize message
        self.msg = String()
        # Configure socket for data exchange with streamlit and connect
        self.socket_path = '/tmp/st_socket.socket'
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    def timer_callback(self):
        # Receive data from server
        with self.client:
            while True:
                self.msg.data = self.client.recv(1024).decode()
                if not self.msg.data:
                    break
                self.query_publisher_.publish(self.msg)
                self.get_logger().info('User prompt: "%s"' % self.msg.data)

    def connect_to_server(self):
        while True:
            try:
                self.client.connect(self.socket_path)
                self.client.send('publisher'.encode())
                self.get_logger().info('UI publisher client connected')
                break  # Successfully connected
            except ConnectionRefusedError:
                self.get_logger().info('UI publisher client connection refused, retrying...')
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    ui_pub_node = StreamlitUIPublisherNode()
    ui_pub_node.connect_to_server()
    rclpy.spin(ui_pub_node)
    ui_pub_node.client.close()
    ui_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
