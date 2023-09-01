import os
import socket
from SpryTrackPackage.SpryTrackWrapper import SpryTrack
import rclpy
from rclpy.node import Node
from ros_tcp_endpoint import TcpServer
from std_msgs.msg import Float32MultiArray

import threading

class CentralPublishers(Node):
    def __init__(self):
        super().__init__('centralPublisher')
        self.HoloLens= self.create_publisher(Float32MultiArray, 'HoloLens', 10)
        self.Femur = self.create_publisher(Float32MultiArray, 'Femur', 10)
        self.Saw= self.create_publisher(Float32MultiArray, 'Saw', 10)
        self.Pelvis= self.create_publisher(Float32MultiArray, 'Pelvis', 10)
        self.Arthrex= self.create_publisher(Float32MultiArray, 'Arthrex', 10)



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
    tcp_server = TcpServer("UnityEndpoint",tcp_ip=IP,tcp_port=port)

    tcp_server.start()

    tcp_server.setup_executor()

    tcp_server.destroy_nodes()
    rclpy.shutdown()

if __name__=='__main__':
    geometry_lists = ['geometry580.ini', # geometry file of HL2
                      'geometry980.ini', # geometry file of Pevlis model
                      ]
    geometry_lists=[os.path.join(os.getcwd(),"geometryFiles",geometryfile) for geometryfile in geometry_lists]
    spryTrack = SpryTrack(geometry_lists)
    rclpy.init()
    tcp_endpoint_thread=threading.Thread(target=start_TCP_endpoint,args=("0.0.0.0",10000,))
    tcp_endpoint_thread.start()

    centralPublishers = CentralPublishers()
    ROS_thread=threading.Thread(target=startROSNode,args=(centralPublishers,))
    ROS_thread.start()

    spryTrack_thread=threading.Thread(target=startTracking,args=(spryTrack,centralPublishers,))
    spryTrack_thread.start()


