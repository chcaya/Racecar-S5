#!/usr/bin/env python

import rospy
import socket
import threading
from struct import *
from tf.transformations import euler_from_quaternion

format_RPOS = "fffxxxx" # 3 float32 et 4 octets vides
format_OBSF = "Ixxxxxxxxxxxx" # 1 uint32 et 12 octets vides
format_RBID = "Ixxxxxxxxxxxx" # 1 uint32 et 12 octets vides
format_PosBroad = "fffI" # 3 float32 et 1 uint32

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
	self.sub_odom = rospy.Subscriber("/racecar/odometry/filtered", Odometry, self.odom_cb)
	self.sub_laser = rospy.Subscriber("/racecar/scan", LaserScan, self.scan_cb)

        # Current robot state:
        self.id = 0xABCD
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
	self.rr_thread.start()

	self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	self.UDP_socket.bind(('10.0.10.31',self.pos_broadcast_port))
	self._timer = rospy.Timer(rospy.Duration(1), self.timer_cb)	

        print("ROSMonitor started.")

    def rr_loop(self):
        # Init your socket here :
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	self.rr_socket.settimeout(None)
	self.rr_socket.bind(('10.0.10.31',self.remote_request_port))
	while True:
		self.rr_socket.listen(1)
		(conn,addr)= self.rr_socket.accept()
		while True:
			data=conn.recv(16)
			if not data: break
			data=data.decode('ascii', 'replace')
			if data == "RPOS":
				sent=pack(format_RPOS,self.pos[0],self.pos[1],self.pos[2])
				test=unpack(format_RPOS,sent)
				conn.send(sent) 
			elif data == "OBSF":
				sent=pack(format_OBSF,self.obstacle)
				conn.send(sent) 
			elif data == "RBID":
				sent=pack(format_RBID,self.id)
				conn.send(sent) 
			else:
				conn.send("Unknown command")
		conn.close()
    def timer_cb(self,event):
	UDP_data=pack(format_PosBroad,self.pos[0],self.pos[1],self.pos[2],self.id)
	self.UDP_socket.sendto(UDP_data,("10.0.10.255",self.pos_broadcast_port))
    def odom_cb(self, msg):
	yaw=quaternion_to_yaw(msg.pose.pose.orientation)
	self.pos=(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)
    def scan_cb(self, msg):
	self.obstacle= False
	for value in msg.ranges:
		if (value < 1) and (value >= msg.range_min) and (value <= msg.range_max):
			self.obstacle = True
if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


