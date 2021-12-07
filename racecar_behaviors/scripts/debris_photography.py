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
from nav_msgs.srv import GetMap
from libbehaviors import *
from nav_msgs.msg import OccupancyGrid


class Debris_Photography:
    def __init__(self):
        self.currentGoal=None
        self._cvbridge = CvBridge()
        self._image = None
        self.objects = []
        self._origin = None
        self._reverseBrushfireMap = None
        self._map_loaded = False
        self.max_speed = rospy.get_param('~max_speed', 0.4)
        self.max_steering = rospy.get_param('~max_steering', 0.37)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.debris_pos = rospy.Subscriber('/racecar/object_coords', Pose, self.debris_callback, queue_size=1)
        self.cam_sub = rospy.Subscriber('/racecar/raspicam_node/image', Image, self.cam_cb, queue_size=1)
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=1)


        # prefix = "racecar"
        # rospy.wait_for_service(prefix + '/get_map')
        # try:
        #     get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        #     response = get_map()
        # except (rospy.ServiceException) as e:
        #     print("Service call failed: %s"%e)
        #     return
        
        # rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
        # grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
        
        # rospy.loginfo("Map origin:")
        # rospy.loginfo(response.map.info.origin)
        # self._reverseBrushfireMap = reverseBrushfire(brushfire(grid))

    def clamp(self, value, max_value):
        if abs(value) > max_value:
            if value < 0:
                max_value = -max_value   
            value = max_value

        return value

    def approachDebris(self,angle,distance,x,y):
        goal_distance = 1.5

        if abs(angle) < 0.15 and distance < 1.9:
            rospy.loginfo("New object")
            id = len(self.objects)
            obj_orig = [10*y + self._origin[0], 10*x + self._origin[1]]
            # obj_orig = [10*x, 10*y]
            # start = [37, 84]
            # end = [51, 211]
            add_object(id, x, y, self._image, self._origin, obj_orig, self._reverseBrushfireMap)
            # add_object(id, x, y, self._image, start, end, self._reverseBrushfireMap)
            self.objects.append((x,y))

            wait = 5 #seconds
            for i in range(0, 10*wait):
                rospy.loginfo("sleep")
                self.cmd_vel_pub.publish(Twist())
                rospy.sleep(wait/100)

            # cv_image = self._cvbridge.imgmsg_to_cv2(self._image, desired_encoding='passthrough')
            # rospy.loginfo("Registered image:")
            # photo_str = "photo_" + str(len(self.objects)) + ".png"
            # rospy.loginfo(cv2.imwrite(photo_str, cv_image))
        else:
            twist = Twist()
            twist.linear.x = self.clamp(distance-goal_distance, self.max_speed)
            twist.angular.z = self.clamp(angle, self.max_steering)
            speed = self.clamp(distance-goal_distance, self.max_speed)
            
            if abs(speed) < 0.2:
                    if speed < 0:
                        speed = -0.2
                    else:
                        speed = 0.2
                        
            twist.linear.x = speed
            if speed < 0:
                twist.angular.z = -self.clamp(angle, self.max_steering)
            else:
                twist.angular.z = self.clamp(angle, self.max_steering)
            
            #rospy.loginfo("twist")
            #rospy.loginfo(twist)
            self.cmd_vel_pub.publish(twist)

    def debris_callback(self, msg):
        if not self._map_loaded:
            return

        # obj_map_x = msg.position.x
        # obj_map_y = msg.position.y
        # obj_dist = msg.position.z
        # obj_angle = msg.orientation.z

        # rospy.loginfo("Angle:")
        # rospy.loginfo(msg.orientation.z)
        # rospy.loginfo("Distance:")
        # rospy.loginfo(msg.position.z)

        if abs(msg.orientation.z) > 1.0:
            return
        
        for i in self.objects:
            if math.sqrt((i[0]-msg.position.x)**2+(i[1]-msg.position.y)**2) <= 1:
                rospy.loginfo("Object already detected")
                return

        self.approachDebris(msg.orientation.z,msg.position.z,msg.position.x,msg.position.y)

    def cam_cb(self, msg):
        self._image = msg

    def map_callback(self, msg):
        if self._reverseBrushfireMap == None:
            rospy.loginfo("Got map=%dx%d resolution=%f", msg.info.height, msg.info.width, msg.info.resolution)    
            grid = np.reshape(msg.data, [msg.info.height, msg.info.width])

            # start = (172, 317)
            # end = (166, 211)
            # self._aStarMap = a_star(self._reverseBrushfireMap, start, end)

            # rospy.loginfo("Map origin:")
            # rospy.loginfo(msg.info.origin)
            self._origin = [-msg.info.origin.position.y*10, -msg.info.origin.position.x*10]
            self._reverseBrushfireMap = reverseBrushfire(brushfire(grid))
            self._map_loaded = True

            # map_debug(grid, self._reverseBrushfireMap ,self._reverseBrushfireMap, self._reverseBrushfireMap)



def main():
    rospy.init_node('Debris_Photography')
    debryPhotography = Debris_Photography()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

