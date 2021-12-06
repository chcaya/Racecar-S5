#!/usr/bin/env python

import rospy
import time
import math 
import numpy as np
from geometry_msgs.msg import Twist, Pose
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Debris_Photography:
    def __init__(self):
        self.currentGoal=None
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.debris_pos = rospy.Subscriber('/racecar/object_coords', Pose, self.debris_callback, queue_size=1)
        self.obstacles = []

    def approachDebris(self,angle,distance,x,y):
        if ((angle < 0.0873 and angle >-0.0873) and ((distance > 1.9) and (distance < 2.1))):
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            time.sleep(5)
            self.obstacles.append((x,y))
            return
        elif (angle < 0.0873 and angle >-0.0873):
            twist = Twist()
            if distance>1:
                cmd_vel=(distance-2)*self.max_speed
                twist.linear.x = min(self.max_speed,cmd_vel)
            else:
                cmd_vel=(distance-2)*self.max_speed
                twist.linear.x = max(-self.max_speed,cmd_vel)    
        elif (distance > 1.9) and (distance < 2.1):
            twist = Twist()
            if angle >0:
                cmd_ang=((angle)/(0.8))*self.max_steering
                twist.angular.z = min(self.max_steering,cmd_ang)
            else:
                cmd_ang=((angle)/(0.8))*self.max_steering
                twist.angular.z = max(-self.max_steering,cmd_ang)
            twist.linear.x = 0.2*self.max_speed
        else:
            twist = Twist()
            if distance>2:
                cmd_vel=(distance-2)*self.max_speed
                twist.linear.x = min(self.max_speed,cmd_vel)
            else:
                cmd_vel=(distance-2)*self.max_speed
                twist.linear.x = max(-self.max_speed,cmd_vel)
            if angle >0:
                cmd_ang=((angle)/(0.8))*self.max_steering
                twist.angular.z = min(self.max_steering,cmd_ang)
            else:
                cmd_ang=((angle)/(0.8))*self.max_steering
                twist.angular.z = max(-self.max_steering,cmd_ang)
        self.cmd_vel_pub.publish(twist)
    def debris_callback(self, msg):
        # obj_map_x = msg.position.x
        # obj_map_y = msg.position.y
        # obj_dist = msg.position.z
        # obj_angle = msg.orientation.z
        
        for i in self.obstacles:
            if math.sqrt((i[0]-msg.position.x)**2+(i[1]-msg.position.y)**2)<=1:
                rospy.loginfo("Object already detected")
                return
            #rospy.loginfo(math.sqrt((i[0]-msg.linear.x)^2+(i[1]-msg.linear.y)^2))
        self.approachDebris(msg.orientation.z,msg.position.z,msg.position.x,msg.position.y)



def main():
    rospy.init_node('Debris_Photography')
    debryPhotography = Debris_Photography()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

