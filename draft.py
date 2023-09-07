import os
from SpryTrackPackage.SpryTrackWrapper import SpryTrack
import rclpy
from ros_tcp_endpoint import TcpServer
from multiprocessing import Process,managers
import threading
from Ultrasound import pycaster
import std_msgs.msg
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray







def startTracking():
    rclpy.init()
    spryTrackRosNode=SpryTrackRosNode()
    geometry_lists = ['geometry580.ini',  # geometry file of HL2
                      'geometry980.ini',  # geometry file of Pevlis model
                      'geometry1280.ini',  # geometry file of Pevlis model
                      ]
    geometry_lists = [os.path.join(os.getcwd(), "geometryFiles", geometryfile) for geometryfile in geometry_lists]
    spryTrack =SpryTrack(geometry_lists)
    i=0
    while True:
        msg=std_msgs.msg.String()
        msg.data="123!!!!!!!!!!!!!!!"
        spryTrackRosNode.text_publisher.publish(msg)
        spryTrack.publish_last_frame(spryTrackRosNode)
        i+=1
        if i%200==0:
            print(f"{i}: ")
            print(spryTrack.detected)
            print('-'*50)



class SpryTrackRosNode(Node):
    def __init__(self):

        super().__init__('SpryTrackNode')
        self.HoloLens= self.create_publisher(Float32MultiArray, 'HoloLens', 10)
        self.Femur = self.create_publisher(Float32MultiArray, 'Femur', 10)
        self.Saw= self.create_publisher(Float32MultiArray, 'Saw', 10)
        self.Pelvis= self.create_publisher(Float32MultiArray, 'Pelvis', 10)
        self.Arthrex= self.create_publisher(Float32MultiArray, 'Arthrex', 10)
        self.Clarius= self.create_publisher(Float32MultiArray, 'Clarius', 10)
        # self.US_publisher = self.create_publisher(Image, 'US_images', 0)
        self.text_publisher=self.create_publisher(std_msgs.msg.String,'test2',10)



def start_TCP_endpoint(IP,port):
    rclpy.init()
    tcp_server = TcpServer("UnityEndpoint",tcp_ip=IP,tcp_port=port)
    tcp_server.start()
    tcp_server.setup_executor()
    tcp_server.destroy_nodes()
    rclpy.shutdown()



if __name__=='__main__':

    tcp_endpoint_thread=Process(target=start_TCP_endpoint,args=("0.0.0.0",10000,))
    tcp_endpoint_thread.start()


    spryTrack_process=Process(target=startTracking,args=())
    spryTrack_process.start()

    clarius_thread=Process(target=pycaster.startClariusStreaming(),args=())
    clarius_thread.start()


