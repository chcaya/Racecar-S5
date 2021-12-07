#!/usr/bin/env python

import rospy
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist, Pose
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Debris_Photography:
    def __init__(self):
        self.currentGoal=None
        self._cvbridge = CvBridge()
        self._image = None
        self.obstacles = []
        self.max_speed = rospy.get_param('~max_speed', 0.5)
        self.max_steering = rospy.get_param('~max_steering', 0.37)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.debris_pos = rospy.Subscriber('/racecar/object_coords', Pose, self.debris_callback, queue_size=1)
        self.cam_sub = rospy.Subscriber('/racecar/raspicam_node/image', Image, self.cam_cb, queue_size=1)

    def clamp(self, value, max_value):
        if abs(value) > max_value:
            if value < 0:
                max_value = -max_value   
            value = max_value

        return value

    def approachDebris(self,angle,distance,x,y):
        goal_distance = 1.5

        if abs(angle) < 0.05 and distance < 1.9:
            rospy.loginfo("New object")
            wait = 5 #seconds
            for i in range(0, 10*wait):
                self.cmd_vel_pub.publish(Twist())
                rospy.sleep(wait/100)

            cv_image = self._cvbridge.imgmsg_to_cv2(self._image, desired_encoding='passthrough')
            rospy.loginfo("Registered image:")
            photo_str = "photo_" + str(len(self.obstacles)) + ".png"
            rospy.loginfo(cv2.imwrite(photo_str, cv_image))
            self.obstacles.append((x,y))
        else:
            twist = Twist()
            twist.linear.x = self.clamp(distance-goal_distance, self.max_speed)
            twist.angular.z = self.clamp(angle, self.max_steering)
            rospy.loginfo("twist")
            rospy.loginfo(twist)
            self.cmd_vel_pub.publish(twist)

    def debris_callback(self, msg):
        # obj_map_x = msg.position.x
        # obj_map_y = msg.position.y
        # obj_dist = msg.position.z
        # obj_angle = msg.orientation.z

        if abs(msg.orientation.z) > 1:
            return
        
        for i in self.obstacles:
            if math.sqrt((i[0]-msg.position.x)**2+(i[1]-msg.position.y)**2) <= 1:
                rospy.loginfo("Object already detected")
                return

        self.approachDebris(msg.orientation.z,msg.position.z,msg.position.x,msg.position.y)

    def cam_cb(self, msg):
        self._image = msg



def main():
    rospy.init_node('Debris_Photography')
    debryPhotography = Debris_Photography()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

