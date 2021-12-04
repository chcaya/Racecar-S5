#!/usr/bin/env python

import rospy
import math
import sys
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    
    return (trans3, rot3)

def brushfire(occupancyGrid):
    clears = -1
    obstacles = 0
    unknowns = 0
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)

    mapOfWorld[occupancyGrid==0] = clears
    mapOfWorld[occupancyGrid==100] = obstacles # obstacles
    mapOfWorld[occupancyGrid==-1] = unknowns # unknowns
    
    # do brushfire algorithm here
    i_brushfire = 1

    while True:
        n_brushfire = 0

        for (row, col), cell in np.ndenumerate(mapOfWorld):
            if cell == i_brushfire-1:
                n_brushfire += 1

                if row-1 >= 0:
                    if mapOfWorld[row-1, col] == clears:
                        mapOfWorld[row-1, col] = i_brushfire

                if col+1 < mapOfWorld.shape[1]:
                    if mapOfWorld[row, col+1] == clears:
                        mapOfWorld[row, col+1] = i_brushfire

                if row+1 < mapOfWorld.shape[0]:
                    if mapOfWorld[row+1, col] == clears:
                        mapOfWorld[row+1, col] = i_brushfire

                if col-1 >= 0:
                    if mapOfWorld[row, col-1] == clears:
                        mapOfWorld[row, col-1] = i_brushfire

        if n_brushfire == 0:
            break
            
        i_brushfire += 1
 
    mapOfWorld[mapOfWorld == 0] = -1
    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld

def reverseBrushfire(brushfireMap):
    reverseBrushfireMap = brushfireMap.copy()
    max_value = np.amax(reverseBrushfireMap)
    reverseBrushfireMap += -max_value-1
    reverseBrushfireMap *= -1
    reverseBrushfireMap[brushfireMap == -1] = -1

    return reverseBrushfireMap

def a_star(reverseBrushfireMap, float_start, float_end):
    start = [int(i) for i in float_start]
    end = [int(i) for i in float_end]

    unknown = -1

    if reverseBrushfireMap[start[0], start[1]] == unknown:
        rospy.logerr("Starts in unknown terrain!")

    if reverseBrushfireMap[end[0], end[1]] == unknown:
        rospy.logerr("Ends in unknown terrain!")

    open_list = []
    closed_list = []

    weigthsMap = np.zeros((reverseBrushfireMap.shape[0], reverseBrushfireMap.shape[1], 5), dtype=int)

    x = 0
    y = 0
    for row in weigthsMap:
        for col in row:
            if reverseBrushfireMap[x, y] != unknown:
                weigthsMap[x, y, 1] = math.sqrt((end[0] - x)**2 + (end[1] - y)**2)
                weigthsMap[x, y, 2] = reverseBrushfireMap[x, y]
            else:
                weigthsMap[x, y, 0] = 1
            
            y += 1
        y = 0
        x += 1

    weigthsMap[start[0], start[1], 3] = 0
    weigthsMap[start[0], start[1], 4] = 0

    coords = start
    while coords != end:
        # rospy.loginfo(coords)
        coords = explore(open_list, closed_list, weigthsMap, coords, unknown)

    path = find_path(start, end, closed_list)
    # rospy.loginfo(path)

    aStarMap = reverseBrushfireMap.copy()

    path_value = 3*np.amax(aStarMap)
    for coords in path:
        aStarMap[coords[0], coords[1]] = path_value

    # for coords in open_list:
    #     aStarMap[coords[0], coords[1]] = coords[2]

    # for coords in closed_list:
    #     aStarMap[coords[0], coords[1]] = coords[2]



    # testMap = np.zeros((reverseBrushfireMap.shape[0], reverseBrushfireMap.shape[1]), dtype=int)
    # # testMap[start[0], start[1]] = 2
    # # testMap[end[0], end[1]] = 2
    # x = 0
    # y = 0
    # for row in weigthsMap:
    #     for col in row:
    #         testMap[x, y] = weigthsMap[x, y, 2]
    #         y += 1
    #     y = 0
    #     x += 1

    # rospy.loginfo(np.amax(testMap))

    # return testMap

    return aStarMap

def explore(open_list, closed_list, grid, coords, unknown):
    grid[coords[0], coords[1], 0] = 1
    closed_list.append([coords[0], coords[1], grid[coords[0], coords[1], 3]])

    prev_cost = grid[coords[0], coords[1], 3]

    check_neighbour(open_list, grid, [coords[0]-1, coords[1]], prev_cost, unknown)
    check_neighbour(open_list, grid, [coords[0], coords[1]+1], prev_cost, unknown)
    check_neighbour(open_list, grid, [coords[0]+1, coords[1]], prev_cost, unknown)
    check_neighbour(open_list, grid, [coords[0], coords[1]-1], prev_cost, unknown)

    open_list.sort(key=lambda cell: cell[2])
    # rospy.loginfo("open_list[0]:")
    # rospy.loginfo(open_list[0])
    # rospy.loginfo("open_list[-1]:")
    # rospy.loginfo(open_list[-1])
    coords_weight = open_list.pop(0)
    # rospy.loginfo(coords_weight)
    return [coords_weight[0], coords_weight[1]]

def check_neighbour(open_list, grid, coords, prev_cost, unknown):
    if coords[0] < grid.shape[0] and coords[0] >= 0 and coords[1] < grid.shape[1] and coords[1] >= 0:
        if coords[1] and grid[coords[0], coords[1], 0] == 0 and grid[coords[0], coords[1], 2] != unknown:
            grid[coords[0], coords[1], 3] = grid[coords[0], coords[1], 2] + prev_cost
            grid[coords[0], coords[1], 4] = grid[coords[0], coords[1], 1] + grid[coords[0], coords[1], 3]
            # rospy.loginfo("Neighbour:")
            # rospy.loginfo("Coords:")
            # rospy.loginfo(coords)
            # rospy.loginfo("Info:")
            # rospy.loginfo(grid[coords[0], coords[1]])
            
            should_add = True
            for open_cell in open_list:
                if open_cell[0] == coords[0] and open_cell[1] == coords[1]:
                    should_add = False
                    if open_cell[2] < grid[coords[0], coords[1], 4]:
                        open_cell = [coords[0], coords[1], grid[coords[0], coords[1], 4]]
                    break
            
            if should_add:
                open_list.append([coords[0], coords[1], grid[coords[0], coords[1], 4]])

            return should_add
        return False
    else:
        return False

def find_path(start, end, closed_list):
    path = [end]
    coords = end
    closed_list.reverse()
    while coords != start:
        if len(closed_list) == 0:
            rospy.logerr("Could not reach the end!")
            break

        pop_i = None
        min_neighbour = [0, 0, sys.maxsize]
        for i in range(len(closed_list)):
            if coords[0]-1 == closed_list[i][0] and coords[1] == closed_list[i][1] or\
                coords[0] == closed_list[i][0] and coords[1]+1 == closed_list[i][1] or\
                coords[0]+1 == closed_list[i][0] and coords[1] == closed_list[i][1] or\
                coords[0] == closed_list[i][0] and coords[1]-1 == closed_list[i][1]:

                if min_neighbour[2] > closed_list[i][2]:
                    min_neighbour = closed_list[i].copy()
                    pop_i = i

        closed_list.pop(pop_i)
        coords = [min_neighbour[0], min_neighbour[1]]
        path.append(coords)

    return path

def map_debug(grid, brushfireMap, reverseBrushfireMap, aStarMap):
    # Export brusfire map for visualization
    # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
    maximum = np.amax(brushfireMap)
    if maximum > 1:
        mask = brushfireMap == -1
        brushfireMap = brushfireMap.astype(float) / float(maximum) * 225.0 + 30.0
        brushfireMap[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        #cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfireMap, -1)))
        cv2.imwrite('brushfire.bmp', cv2.transpose(brushfireMap))
        rospy.loginfo("Exported brushfire.bmp")
    else:
        rospy.loginfo("brushfire failed! Is brusfire implemented?")

    maximum = np.amax(reverseBrushfireMap)
    if maximum > 1:
        mask = reverseBrushfireMap == -1
        reverseBrushfireMap = reverseBrushfireMap.astype(float) / float(maximum) * 225.0 + 30.0
        reverseBrushfireMap[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        #cv2.imwrite('reverseBrushfire.bmp', cv2.transpose(cv2.flip(reverseBrushfireMap, -1)))
        cv2.imwrite('reverseBrushfire.bmp', cv2.transpose(reverseBrushfireMap))
        rospy.loginfo("Exported reverseBrushfire.bmp")
    else:
        rospy.loginfo("reverseBrushfire failed! Is brusfire implemented?")

    maximum = np.amax(aStarMap)
    if maximum > 1:
        mask = aStarMap == -1
        aStarMap = aStarMap.astype(float) / float(maximum) * 225.0 + 30.0
        aStarMap[mask] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        #cv2.imwrite('aStar.bmp', cv2.transpose(cv2.flip(aStarMap, -1)))
        cv2.imwrite('aStar.bmp', cv2.transpose(aStarMap))
        rospy.loginfo("Exported aStar.bmp")
    else:
        rospy.loginfo("aStar failed! Is brusfire implemented?")
    
    # Example to show grid with same color than RVIZ
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0

    # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
    #cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1)))
    cv2.imwrite('map.bmp', cv2.transpose(grid))
    rospy.loginfo("Exported map.bmp")
