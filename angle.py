#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 6            # 1 <= samples_view <= samples
                        #Change the value of samples_view to a larger odd number, e.g. samples_view = 11. //2A
                        #his will allow the lidar to read 11 samples                                      //2A
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:   
            scan_filter.append(scan.ranges[0])

        else:
            #This will calculate the left_lidar_samples_ranges and right_lidar_samples_ranges based on the angle_increment and samples_view. //2A
            #Also, you need to update the samples_view to 6 to get 11 samples for -45 to 45 degrees. //2A
            left_lidar_samples_ranges = -(samples_view//2 - (math.degrees(scan.angle_increment) * samples_view))
            right_lidar_samples_ranges = samples_view//2 + (math.degrees(scan.angle_increment) * (samples_view - 1))
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True
        eft_obstacle = False
        right_obstacle = False

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
            
                # Check for left and right obstacles
                left_obstacle = any(distance < SAFE_STOP_DISTANCE for distance in lidar_distances[:5])
                right_obstacle = any(distance < SAFE_STOP_DISTANCE for distance in lidar_distances[-5:])

                # Take a turn to avoid obstacles
                if left_obstacle and not right_obstacle:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('Turn Right!')
                elif right_obstacle and not left_obstacle:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('Turn Left!')
                else:
                # Move backward if there's no other way to go
                    twist.linear.x = -0.1
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('Move Backward!')
            
        else:
            twist.linear.x = LINEAR_VEL
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            turtlebot_moving = True
            rospy.loginfo('Distance of the obstacle : %f', min_distance)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
