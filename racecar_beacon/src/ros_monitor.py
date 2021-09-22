#!/usr/bin/env python3

import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion
from struct import *
from racecar_beacon_constants import *


def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    # yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", REMOTE_CLIENT_PORT)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", VEHICLE_TRACKER_PORT)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        self.pb_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Socket UDP
        self.pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Mode broadcast
        self.pb_socket.settimeout(0.1)
        self.pb_timer = rospy.Timer(rospy.Duration(1), self.position_broadcast)

        print("ROSMonitor started.")

    def odom_cb(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def scan_cb(self, msg):
        self.obstacle = False

        if msg.range_min < 1:
            self.obstacle = True

    def position_broadcast(self, event):
        enc = pack(FORMAT_POSBROAD, self.pos[0], self.pos[1], self.pos[2], self.id)
        self.pb_socket.sendto(enc, ("255.255.255.255", self.pos_broadcast_port))

    def remote_request(self):
        (conn, addr) = self.rr_socket.accept() # returns new socket and addr. client

        while True:
            data = conn.recv(16) # receive data from client

            if not data:
                break # stop if client stopped

            cmd = data.decode("utf-8")

            rospy.logwarn(str(cmd))

            enc = pack("5s", b"ERROR")
            
            if cmd == "RPOS":
                enc = pack(FORMAT_RPOS, self.pos[0], self.pos[1], self.pos[2])
            elif cmd == "OBSF":
                enc = pack(FORMAT_OBSF, self.obstacle)
            elif cmd == "RBID":
                enc = pack(FORMAT_RBID, self.id)

            conn.send(enc)

        conn.close() # close the connection

    def rr_loop(self):
        # Init your socket here :
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket.settimeout(None)
        self.rr_socket.bind((HOST, self.remote_request_port))

        self.rr_socket.listen(1)

        while True:
            self.remote_request()

    def run(self):
        self.rr_thread.start()
        rospy.spin()


if __name__=="__main__":
    rospy.init_node("ros_monitor")

    ros_monitor_node = ROSMonitor()
    ros_monitor_node.run()
