import os
import socket
from SpryTrackPackage.SpryTrackWrapper import SpryTrack
import rclpy
from rclpy.node import Node
from ros_tcp_endpoint import TcpServer
from std_msgs.msg import Float32MultiArray
from multiprocessing import Process,Manager
import threading
from Ultrasound import pycaster
from sensor_msgs.msg import Image

import sys
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        print("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

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



def startROSNode(centralPublishers):

    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # maybe a wait function here
    rclpy.spin(centralPublishers)

    centralPublishers.destroy_node()
    rclpy.shutdown()

def startTracking(spryTrack,centralPublishers):
    i=0
    while True:
        spryTrack.publish_last_frame(centralPublishers)
        i+=1
        if i%200==0:
            print(f"{i}: ")
            print(spryTrack.detected)
            print('-'*50)

def start_TCP_endpoint(IP,port):
    rclpy.init()
    tcp_server = TcpServer("UnityEndpoint",tcp_ip=IP,tcp_port=port)

    tcp_server.start()

    tcp_server.setup_executor()

    tcp_server.destroy_nodes()
    rclpy.shutdown()

if __name__=='__main__':

    geometry_lists = ['geometry580.ini', # geometry file of HL2
                      'geometry980.ini', # geometry file of Pevlis model
                      'geometry1280.ini', # geometry file of Pevlis model
                      ]
    geometry_lists=[os.path.join(os.getcwd(),"geometryFiles",geometryfile) for geometryfile in geometry_lists]
    spryTrack = SpryTrack(geometry_lists)
    rclpy.init()
    tcp_endpoint_thread=threading.Thread(target=start_TCP_endpoint,args=("0.0.0.0",10000,))
    tcp_endpoint_thread.start()

    centralPublishers = ExamplePublishers()
    ROS_thread=threading.Thread(target=startROSNode,args=(centralPublishers,))
    ROS_thread.start()

    spryTrack_thread=threading.Thread(target=startTracking,args=(spryTrack,centralPublishers,))
    spryTrack_thread.start()

    # clarius_thread=threading.Thread(target=pycaster.startStreaming(),args=())
    # clarius_thread.start()


