#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from libbehaviors import *

class PathFollowing:
    def __init__(self):
        self._grid = np.empty(0)
        self._brushfireMap = np.empty(0)
        self._reverseBrushfireMap = np.empty(0)
        self._aStareMap = np.empty(0)
        self._pose = [0, 0, 0]
        self._current_goal = [0, 0, 0]
        self._id_goal = "id_goal"
        self._id_start = "id_start"
        self._id_obj = "id_obj"
        self._goal = [10, 0.5, math.pi, self._id_goal]
        self._detected_objects = []
        self._object_data = [0, 0, 0, 0]
        self._cvbridge = CvBridge()
        self._image = None

        self._tf_listener = tf.TransformListener()

        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/racecar/move_base/goal', MoveBaseActionGoal, queue_size=1, latch=True)


        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=1)
        self.mb_feedback_sub = rospy.Subscriber('/racecar/move_base/feedback', MoveBaseActionFeedback, self.mb_feedback_cb, queue_size=1)
        self.mb_result_sub = rospy.Subscriber('/racecar/move_base/result', MoveBaseActionResult, self.mb_result_cb, queue_size=1)
        self.obj_coords_sub = rospy.Subscriber('/racecar/object_coords', Pose, self.obj_coords_cb, queue_size=1)
        self.cam_sub = rospy.Subscriber('/racecar/raspicam_node/image', Image, self.cam_cb, queue_size=1)


        # self.get_map()

    def get_map(self):
        self._brushfireMap = brushfire(self._grid)
        self._reverseBrushfireMap = reverseBrushfire(self._brushfireMap)

        start = (0, 0)
        end = (1, 1)
        self._aStarMap = a_star(self._reverseBrushfireMap, start, end)

        map_debug(self._grid, self._brushfireMap, self._reverseBrushfireMap, self._aStarMap)

    def get_pose(self):
        pose = [0, 0, 0]
        while True:
            try:
                (trans, ori) = self._tf_listener.lookupTransform("/racecar/map", "/racecar/chassis", rospy.Time(0))

                pose[0] = trans[0]
                pose[1] = trans[1]

                ori_list = [ori[0], ori[1], ori[2], ori[3]]
                (roll, pitch, yaw) = euler_from_quaternion (ori_list)

                pose[2] = yaw

                return pose

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        #l2 = len(msg.ranges)/2;
        #ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        # l2 = int(len(msg.ranges)/2)
        # ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        # max_range = 5*[0]
        # i_max_range = None
        # for i in range(l2-int(l2/2), l2+int(l2/2)) :
        #     new_max_range = max_range.copy()
        #     new_max_range.pop(0)
        #     new_max_range.append(ranges[i])
        #     # print(i)
        #     # print("New max range:")
        #     # print(sum(new_max_range))
        #     # print("Max range:")
        #     # print(sum(max_range))
        #     if sum(new_max_range) > sum(max_range):
        #         max_range = new_max_range
        #         i_max_range = i-2
        #         # print("NEW MAX RANGE")

        # print(max_range)
        # print(float(2*math.pi*(i_max_range-180)/360))
        
        # twist = Twist()
        # twist.linear.x = self.max_speed
        # twist.angular.z = float(2*math.pi*(i_max_range-180)/360)

        # twist = Twist()
        # twist.linear.x = self.max_speed
        # twist.angular.z = 0
           
        # self.cmd_vel_pub.publish(twist)
        return
        
    def odom_callback(self, msg):
        # rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)
        return

    def map_callback(self, msg):
        # rospy.loginfo("Got map=%dx%d resolution=%f", msg.info.height, msg.info.width, msg.info.resolution)    
        # self._grid = np.reshape(msg.data, [msg.info.height, msg.info.width])

        # self._brushfireMap = brushfire(self._grid)
        # self._reverseBrushfireMap = reverseBrushfire(self._brushfireMap)

        # start = (172, 317)
        # end = (166, 211)
        # self._aStarMap = a_star(self._reverseBrushfireMap, start, end)
        return

        map_debug(self._grid, self._brushfireMap, self._reverseBrushfireMap, self._aStarMap)

    def send_goal(self, goal):
        self._current_goal = goal

        goal_msg = MoveBaseActionGoal()

        goal_msg.goal.target_pose.header.frame_id = "racecar/map"
        goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
        goal_msg.goal_id.id = goal[3]

        goal_msg.goal.target_pose.pose.position.x = goal[0]
        goal_msg.goal.target_pose.pose.position.y = goal[1]
        goal_msg.goal.target_pose.pose.position.z = 0

        ori_list = quaternion_from_euler(0, 0, goal[2])
        goal_msg.goal.target_pose.pose.orientation.x = ori_list[0]
        goal_msg.goal.target_pose.pose.orientation.y = ori_list[1]
        goal_msg.goal.target_pose.pose.orientation.z = ori_list[2]
        goal_msg.goal.target_pose.pose.orientation.w = ori_list[3]

        rospy.loginfo("Sent goal:")
        rospy.loginfo(goal)

        self.goal_pub.publish(goal_msg)

    def mb_feedback_cb(self, msg):
        self._pose[0] = msg.feedback.base_position.pose.position.x
        self._pose[1] = msg.feedback.base_position.pose.position.y

        ori_list = [msg.feedback.base_position.pose.orientation.x, msg.feedback.base_position.pose.orientation.y,\
                    msg.feedback.base_position.pose.orientation.z, msg.feedback.base_position.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (ori_list)
        self._pose[2] = yaw

        # rospy.loginfo("x:")
        # rospy.loginfo(x)
        # rospy.loginfo("y:")
        # rospy.loginfo(y)
        # rospy.loginfo("theta:")
        # rospy.loginfo(theta)
        # rospy.loginfo("Status:")
        # rospy.loginfo(msg.status.status)

    def mb_result_cb(self, msg):
        if msg.status.status == 3:
            rospy.loginfo("Goal reached id:")
            rospy.loginfo(msg.status.goal_id.id)
            # if self._current_goal == self._goal:
            if msg.status.goal_id.id == self._id_goal:
            # if abs(self._pose[0] - self._goal[0]) < 0.2 and abs(self._pose[1] - self._goal[1]) < 0.2 and\
            #         (abs(self._pose[2] - self._goal[2]) < 0.2 or abs(self._pose[2] - self._goal[2]) - 2*math.pi < 0.2):
                rospy.logerr("Final goal reached!")
                self._goal = [0, 0, 0, self._id_start]

            elif msg.status.goal_id.id == self._id_start:
            # elif abs(self._pose[0] - self._goal[0]) < 0.2 and abs(self._pose[1] - self._goal[1]) < 0.2 and\
            #     (abs(self._pose[2] - self._goal[2]) < 0.2 or abs(self._pose[2] - self._goal[2]) - 2*math.pi < 0.2):
                # rospy.sleep(1)
                rospy.signal_shutdown("Job done!")
                return

            elif msg.status.goal_id.id == self._id_obj:
            # else:
                rospy.loginfo("New object")
                # rospy.loginfo("Object dist:")
                # rospy.loginfo(self._object_data[2])
                # rospy.loginfo("Object angle:")
                # rospy.loginfo(self._object_data[3])
                rospy.loginfo("Adding object:")
                rospy.loginfo(self._object_data)
                # rospy.loginfo("Pose:")
                # rospy.loginfo(self._pose)
                self._detected_objects.append(self._object_data.copy())
                cv_image = self._cvbridge.imgmsg_to_cv2(self._image, desired_encoding='passthrough')
                rospy.loginfo("Registered image:")
                rospy.loginfo(cv2.imwrite("pic.png", cv_image))
                rospy.sleep(5)

            self.send_goal(self._goal)

    def obj_coords_cb(self, msg):
        obj_buffer = 1.5

        # obj_map_x = msg.position.x
        # obj_map_y = msg.position.y
        # obj_dist = msg.position.z
        # obj_angle = msg.orientation.z
        self._object_data[0] = msg.position.x
        self._object_data[1] = msg.position.y
        self._object_data[2] = msg.position.z
        self._object_data[3] = msg.orientation.z
        

        if self._object_data[2] > 5:
            rospy.loginfo("Object too far!")
            rospy.sleep(1)
            return

        for detected_obj in self._detected_objects:
            if abs(detected_obj[0] - self._object_data[0]) < 1 and abs(detected_obj[1] - self._object_data[1]) < 1:
                # rospy.loginfo("Detected objects list:")
                # rospy.loginfo(self._detected_objects)
                # rospy.loginfo("Detected object:")
                # rospy.loginfo(detected_obj)
                # rospy.loginfo("Current object:")
                # rospy.loginfo(self._object_data)
                rospy.loginfo("Object already detected!")
                rospy.sleep(1)
                return

        abs_angle = self._pose[2] + self._object_data[3]
        goal_x = (self._object_data[2]-obj_buffer)*math.cos(abs_angle) + self._pose[0]
        goal_y = (self._object_data[2]-obj_buffer)*math.sin(abs_angle) + self._pose[1]

        if not (abs(self._current_goal[0] - goal_x) < 0.1 and abs(self._current_goal[1] - goal_y) < 0.1 and\
             (abs(self._current_goal[2] - abs_angle) < 0.1 or abs(self._current_goal[2] - abs_angle) - 2*math.pi < 0.1)):
            self.send_goal([goal_x, goal_y, abs_angle, self._id_obj])

        # rospy.loginfo("Object dist:")
        # rospy.loginfo(self._object_data[2])
        # rospy.loginfo("Object angle:")
        # rospy.loginfo(self._object_data[3])

        # if self._object_data[2] < 1.5 and abs(self._object_data[3]) < 0.2:
        #     rospy.loginfo("New object")
        #     rospy.loginfo("Object dist:")
        #     rospy.loginfo(self._object_data[2])
        #     rospy.loginfo("Object angle:")
        #     rospy.loginfo(self._object_data[3])
        #     self._detected_objects.append(self._object_data)
        #     self.send_goal(self._pose)
        #     rospy.sleep(5.)
        #     self.send_goal(self._goal)

    def cam_cb(self, msg):
        self._image = msg

    def run(self):
        while self.goal_pub.get_num_connections() == 0:
            continue

        self.send_goal(self._goal)
        rospy.spin()


def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    pathFollowing.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

