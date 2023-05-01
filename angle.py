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
right_distance = -1
left_distance = -1
front_distance = -1
back_distance = -1
collision_counter = 0

def turn_left(self, angle, velocity):
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = angle/2
        twist.angular.z = angle/2
        self._cmd_pub.publish(twist)
        rospy.loginfo('Turning to left')

def turn_right (self,angle, velocity):
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = -angle/2
        twist.angular.z = -angle/2
        self._cmd_pub.publish(twist)
        rospy.loginfo('Turning to right')

def turn_around(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 1.57
        self._cmd_pub.publish(twist)
        rospy.loginfo('Turning around')

def revers(self, velocity):
        twist = Twist()
        twist.linear.x = -velocity
        twist.angular.z = 0
        self._cmd_pub.publish(twist)
        rospy.loginfo('Going back')

class Obstacle():                                       

    twist = Twist()
    
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.obstacle()

    def get_scan(self):
        global right_distance
        global left_distance
        global front_distance
        global back_distance

        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in
                                    # turtlebot3_<model>.gazebo.xacro file
                                    # the default is 360.
        samples_view = 8            # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])


        else:
            front_lidar_samples_ranges = 0
            
            front_right_lidar_samples_ranges = 45
            right_lidar_samples_ranges = 90

            back_right_lidar_samples_ranges = 135
            back_lidar_samples_ranges = 180
            back_left_lidar_samples_ranges = 225

            left_lidar_samples_ranges = 270
            front_left_lidar_samples_ranges = 315
           
            def filter_zeros(lidar_samples):
                 return list(filter(lambda x: x != 0, lidar_samples))


            front_left_lidar_samples = tuple(filter_zeros(scan.ranges[front_left_lidar_samples_ranges:front_lidar_samples_ranges]))
            print("front left = \n",front_left_lidar_samples)
            front_right_lidar_samples =tuple(filter_zeros (scan.ranges[front_lidar_samples_ranges:front_right_lidar_samples_ranges]))
            print("front right = \n",front_right_lidar_samples)
            
            right_1_lidar_samples = tuple(filter_zeros(scan.ranges[front_right_lidar_samples_ranges:right_lidar_samples_ranges]))
            print("right_1_lidar = \n",right_1_lidar_samples)
            right_2_lidar_samples = tuple(filter_zeros(scan.ranges [right_lidar_samples_ranges:back_right_lidar_samples_ranges]))
            print("right_2_lidar = \n",right_2_lidar_samples)

            left_1_lidar_samples = tuple(filter_zeros(scan.ranges[back_left_lidar_samples_ranges:left_lidar_samples_ranges]))
            print("left_1_lidar = \n",left_1_lidar_samples)
            left_2_lidar_samples = tuple(filter_zeros(scan.ranges[left_lidar_samples_ranges:front_left_lidar_samples_ranges]))
            print("left_2_lidar = \n",left_2_lidar_samples)
            
            back_left_lidar_samples = tuple(filter_zeros(scan.ranges[back_lidar_samples_ranges:back_left_lidar_samples_ranges]))
            print("back left = \n",back_left_lidar_samples)
            back_right_lidar_samples = tuple(filter_zeros(scan.ranges[back_right_lidar_samples_ranges:back_lidar_samples_ranges]))
            print("back right = \n",back_right_lidar_samples)


            scan_filter.extend(front_left_lidar_samples + front_right_lidar_samples + left_1_lidar_samples+ left_2_lidar_samples + right_1_lidar_samples+ right_2_lidar_samples + back_left_lidar_samples + back_right_lidar_samples)
            print("Qeue reading")

            
        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0

        right_distance = min(right_1_lidar_samples + right_2_lidar_samples + (0.01,)) 
        left_distance = min(left_1_lidar_samples + left_2_lidar_samples + (0.01,))
        front_distance = min(front_left_lidar_samples + front_right_lidar_samples + (0.01,))
        back_distance = min(back_left_lidar_samples + back_right_lidar_samples + (0.01,))  
        print("lidar Data is being returned\n") 
        return scan_filter

    
    def obstacle(self):
        global right_distance
        global left_distance
        global front_distance
        global back_distance
        global collision_counter
        twist = Twist()
        turtlebot_moving = True


        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_STOP_DISTANCE:
                    turtlebot_moving
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
                    print("Turtlebot stoping, obstacle found \n")


        #Here I will make different if-statements for different cases

            if (front_distance < SAFE_STOP_DISTANCE or back_distance < SAFE_STOP_DISTANCE or left_distance < SAFE_STOP_DISTANCE or right_distance < SAFE_STOP_DISTANCE):
                 collision_counter += 1
                 print ("Collision counter = ",collision_counter)

            if ( front_distance > SAFE_STOP_DISTANCE ):
                twist.linear.x = LINEAR_VEL

                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)
                print("Turtlebot moving forward\n")

            if ( right_distance < SAFE_STOP_DISTANCE and front_distance < SAFE_STOP_DISTANCE and turtlebot_moving ):
                    turn_left(self,0.1,0)
                    print("Turning left \n")

            if ( left_distance < SAFE_STOP_DISTANCE and front_distance < SAFE_STOP_DISTANCE and turtlebot_moving ):
                    turn_right(self,0.1,0)
                    print("Turning right \n")   


            if (right_distance < SAFE_STOP_DISTANCE and front_distance < SAFE_STOP_DISTANCE and back_distance < SAFE_STOP_DISTANCE and turtlebot_moving):
                    turn_left(self,0.1,0)
                    print("Turning left \n")
                    
            if (left_distance < SAFE_STOP_DISTANCE and front_distance < SAFE_STOP_DISTANCE and back_distance < SAFE_STOP_DISTANCE and turtlebot_moving):
                    turn_right(self,0.1,0)
                    print("Turning right \n")   

            if (right_distance < SAFE_STOP_DISTANCE and front_distance < SAFE_STOP_DISTANCE and left_distance < SAFE_STOP_DISTANCE and turtlebot_moving):
                    revers(self,0.22)
                    turn_around(self)
                    print("Turning back \n")






def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if _name_ == '_main_':
    main()
