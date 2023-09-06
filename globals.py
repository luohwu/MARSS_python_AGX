import std_msgs.msg
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import rclpy
from ros_tcp_endpoint import TcpServer
class ExamplePublishers(Node):
    def __init__(self):
        super().__init__('centralPublisher')
        self.HoloLens= self.create_publisher(Float32MultiArray, 'HoloLens', 10)
        self.Femur = self.create_publisher(Float32MultiArray, 'Femur', 10)
        self.Saw= self.create_publisher(Float32MultiArray, 'Saw', 10)
        self.Pelvis= self.create_publisher(Float32MultiArray, 'Pelvis', 10)
        self.Arthrex= self.create_publisher(Float32MultiArray, 'Arthrex', 10)
        self.Clarius= self.create_publisher(Float32MultiArray, 'Clarius', 10)
        self.US_publisher = self.create_publisher(Image, 'US_images', 0)
        self.text_publisher=self.create_publisher(std_msgs.msg.String,'test2',10)


# centralPublishers = ExamplePublishers()
# tcp_server.setup_executor()
#
# tcp_server.destroy_nodes()
# rclpy.shutdown()