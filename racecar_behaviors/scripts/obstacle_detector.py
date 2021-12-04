#!/usr/bin/env python

import rospy
import math 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult

class ObstacleDetector:
    def __init__(self):
        self.distance = rospy.get_param('~distance', 0.75)
        self.max_speed = rospy.get_param('~max_speed', 1)
        self._pose = [0, 0, 0]
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.mb_feedback_sub = rospy.Subscriber('/racecar/move_base/feedback', MoveBaseActionFeedback, self.mb_feedback_cb, queue_size=1)

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = int(len(msg.ranges)/2)
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        obstacle_detected = False
        stuck = False
        
        for i in range(l2-int(l2/8), l2+int(l2/8)) :
            if np.isfinite(ranges[i]) and ranges[i] > 0 and ranges[i] < self.distance:
                obstacle_detected = True

                for i in range(l2-int(l2/8), l2+int(l2/8)) :
                    if np.isfinite(msg.ranges[i]) and msg.ranges[i] > 0 and msg.ranges[i] < self.distance:
                        stuck = True
                        break
                break

        if obstacle_detected:
            rospy.loginfo("Obstacle detected!")

            twist = Twist()

            if not stuck:
                twist.linear.x = -self.max_speed
                twist.angular.z = self._pose[2]
                rospy.loginfo("Backing up!")

            else:
                rospy.loginfo("Stuck! Need help!")
            

            self.cmd_vel_pub.publish(twist)

    def mb_feedback_cb(self, msg):
        self._pose[0] = msg.feedback.base_position.pose.position.x
        self._pose[1] = msg.feedback.base_position.pose.position.y

        ori_list = [msg.feedback.base_position.pose.orientation.x, msg.feedback.base_position.pose.orientation.y,\
                    msg.feedback.base_position.pose.orientation.z, msg.feedback.base_position.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (ori_list)
        self._pose[2] = yaw

                

def main():
    rospy.init_node('obstacle_detector')
    obstacleDetector = ObstacleDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

