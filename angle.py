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
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Constants
LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2  #CHANGE IT TO 0.2
LIDAR_ERROR = 0.05
escape_distance = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

# Global variables
collision_counter = 0
speed_updates = 0.01
speed_accumulation = 0
turtlebot_moving = False

front_right_distance = 0.1
front_left_distance = 0.1
right_distance = 0.1
left_distance = 0.1
back_distance = 0.1 

class Obstacle():                                       

    global collision_counter 
    global speed_updates
    global speed_accumulation
    twist = Twist()

    def turn_left(self, angle, velocity):
        global turtlebot_moving
        turtlebot_moving = False
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = angle/2
        rospy.sleep(0.1)
        twist.angular.z = angle/2        # BACK 
        self._cmd_pub.publish(twist)
        rospy.loginfo('Turning to left')

    def turn_right (self,angle, velocity):
        global turtlebot_moving
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = -angle/2
        rospy.sleep(0.1)
        twist.angular.z = -angle/2
        self._cmd_pub.publish(twist)
        rospy.loginfo('Turning to right')
        turtlebot_moving = False


    def turn_around(self):
        global turtlebot_moving
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = math.pi/2
        self._cmd_pub.publish(twist)
        rospy.loginfo('Turning around')
        turtlebot_moving = False


    def reverse(self, velocity):
        global turtlebot_moving
        twist = Twist()
        twist.linear.x = -velocity
        twist.angular.z = 0
        self._cmd_pub.publish(twist)
        rospy.loginfo('Going back')
        turtlebot_moving = False


    def forward (self):
        twist = Twist()
        twist.linear.x = LINEAR_VEL
        twist.angular.z = 0.0
        self._cmd_pub.publish(twist)
        rospy.loginfo('Moving Forward!')
        print("Moving forward")

    
    def stop (self):
        global turtlebot_moving
        twist = Twist()
        turtlebot_moving = False
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._cmd_pub.publish(twist)
        rospy.loginfo('Obstacle found')
        print("Stopping")
    
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.obstacle()

    def get_scan(self):

        global front_right_distance
        global front_left_distance
        global right_distance
        global left_distance
        global back_distance

        try:
            scan = rospy.wait_for_message('scan', LaserScan, timeout=1)
        except rospy.exceptions.ROSException:
            rospy.logerr('LaserScan timed out')
            return []

        scan_filter = []

        def filter_zeros(lidar_samples):
              return list(filter(lambda x: x != 0, lidar_samples))
        
        samples = len(scan.ranges)  # The number of samples is defined in
                                    # turtlebot3_<model>.gazebo.xacro file
                                    # the default is 360.
        samples_view = 5            # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])


        else:
            front_angel = 0
            front2_angel = 360
            front_right_angel = 330
            front_left_angel = 30
            left_angel = 90
            right_angel = 270
  


            front_left_range = tuple(filter_zeros(scan.ranges[front_angel:front_left_angel]))
            if  front_left_range:
                front_left_distance = min(front_left_range)   
            else:
                front_left_distance = 0.01
                print ("No READINGS from front left")
                print("FRONT LEFT distance = \n",front_left_distance)

            front_right_range = tuple(filter_zeros(scan.ranges[front_right_angel:front2_angel]))
            if  front_right_range:
                front_right_distance = min(front_right_range)   
            else:
                front_right_distance = 0.01
                print ("No READINGS from front right")
                print("FRONT RIGHT distance = \n",front_right_distance)


            right_range = tuple(filter_zeros(scan.ranges[right_angel:front_right_angel]))
           
           
            if  right_range:

                right_distance = min(right_range)   


            else:
                right_distance = 0.01
                print ("No READINGS from right")
                print("RIGHT Distance = \n",right_distance)


            left_range = tuple(filter_zeros(scan.ranges[left_angel:front_left_angel]))
            if  left_range:
                left_distance = min(left_range)  
            else:
                left_distance = 0.01
                print ("No READINGS from left")
                print("LEFT Distance = \n",left_distance)

 
            back_range = tuple(filter_zeros(scan.ranges[left_angel:right_angel]))
            if  back_range:
                back_distance = min(back_range)   
            else:
                back_distance = 0.01
                print ("No READINGS from front left")
                print("BACK Distance = \n",back_distance)

            scan_filter.extend(front_left_range + front_right_range + left_range  + right_range  + back_range )
            print("Queue Reading")

            
        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 101


        return scan_filter

    
    def obstacle(self):
        turtlebot_moving = True
        global collision_counter 
        global speed_updates
        global speed_accumulation
        global front_right_distance
        global front_left_distance
        global right_distance
        global left_distance
        global back_distance 

        while not rospy.is_shutdown():

            speed_accumulation += LINEAR_VEL
            speed_updates += 1
            lidar_distances = self.get_scan()

            if not lidar_distances:
                min_distance = 0.01
            else: 
                    min_distance = min(lidar_distances)
        #If-statements for different cases

            print("FRONT LEFT distance = \n",front_left_distance)
            print("FRONT RIGHT distance = \n",front_right_distance)
            print("RIGHT Distance = \n",right_distance) 
            print("LEFT Distance = \n",left_distance)
            print("BACK Distance = \n",back_distance)



            if ( front_right_distance > SAFE_STOP_DISTANCE and front_left_distance > SAFE_STOP_DISTANCE ):
                #while  front_right_distance > SAFE_STOP_DISTANCE and front_left_distance > SAFE_STOP_DISTANCE:               
                    print("SaFe stop DISt",SAFE_STOP_DISTANCE)
                    print("1FRONT LEFT distance = \n",front_left_distance)
                    print("1FRONT RIGHT distance = \n",front_right_distance)
                    #turtlebot_moving = True
                    self.forward()
            elif ( front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE ): 
                if ( front_right_distance < escape_distance or front_left_distance < escape_distance ):
                    print("Obstacle infront(Too close to turn)")
                    self.reverse(LINEAR_VEL)
                    self.turn_around ()
                    print ("0Backing up")

                elif ( front_right_distance > escape_distance and front_left_distance > escape_distance ):
                    if (right_distance > SAFE_STOP_DISTANCE):
                       # while front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE: 

                            print("2FRONT LEFT distance = \n",front_left_distance)
                            print("2FRONT RIGHT distance = \n",front_right_distance)
                            print ("Obstacle found infront")
                            self.turn_right(1.3,0.0)
                    elif (left_distance > SAFE_STOP_DISTANCE):
                      #  while front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE: 
                            turtlebot_moving = True
                            print("3FRONT LEFT distance = \n",front_left_distance)
                            print("3FRONT RIGHT distance = \n",front_right_distance)
                            print ("Obstacle found infront")
                            self.turn_left(1.3,0.0)
                    elif (front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE and right_distance < SAFE_STOP_DISTANCE and left_distance < SAFE_STOP_DISTANCE):  
                        self.reverse(LINEAR_VEL)
                        self.turn_left(0.2,0.0)
                        print ("1All roads blocked. Backing up")
                        rospy.sleep(0.5)
                        self.turn_around()
            elif ( front_right_distance < SAFE_STOP_DISTANCE and front_left_distance > SAFE_STOP_DISTANCE ):  
                if (left_distance > SAFE_STOP_DISTANCE):
                        print 
                     #while front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE: 
                        self.turn_left(0.63,0.0)
                        print("4FRONT LEFT distance = \n",front_left_distance)
                        print("4FRONT RIGHT distance = \n",front_right_distance)
                        print ("Obstacle found Right infront")
                elif right_distance > SAFE_STOP_DISTANCE :
                        self.turn_right(0.63,0.0)
                        print ("Obstacle found Right infront")
                             
                elif   right_distance < SAFE_STOP_DISTANCE and left_distance < SAFE_STOP_DISTANCE:
                    self.reverse(LINEAR_VEL)
                    print ("Obstacle found Right infront and at left")
                    print ("2 Cant turn here. Backing up")
                    rospy.sleep(0.1)
                    self.turn_left(0.3,0.0)
            elif ( front_left_distance < SAFE_STOP_DISTANCE and front_right_distance > SAFE_STOP_DISTANCE ):  
                if (right_distance > SAFE_STOP_DISTANCE):
                     #while front_left_distance < SAFE_STOP_DISTANCE and front_right_distance < SAFE_STOP_DISTANCE: 
                        print("5FRONT LEFT distance = \n",front_left_distance)
                        print("5FRONT RIGHT distance = \n",front_right_distance)
                        self.turn_right(0.63,0.0)
                        print ("Obstacle found infront left ")
                elif left_distance > SAFE_STOP_DISTANCE :
                        self.turn_left(0.63,0.0)
                        print ("Obstacle found at front left")
                elif right_distance < SAFE_STOP_DISTANCE and left_distance < SAFE_STOP_DISTANCE:
                    print ("Obstacle found at front left and at right ")
                    self.reverse(LINEAR_VEL)
                    print ("3 Cant turn here. Backing up")
                    rospy.sleep(0.1)
                    self.turn_right(0.2,0.0)

        
                #Collision Counter
            if (front_right_distance < (escape_distance-0.02) and front_left_distance < (escape_distance-0.02)): 
                 collision_counter += 1
                 print ("Collision counter = ",collision_counter)



def main():

    global speed_accumulation
    global speed_updates
    global collision_counter
start_time = time.time()

while time.time() - start_time < 120:
            rospy.init_node('turtlebot3_obstacle')
            try:
                obstacle = Obstacle()
            except rospy.ROSInterruptException:
                pass

    if _name_ == '_main_':
         main()
Average_linear_speed=speed_accumulation/speed_updates
print("Average linear speed = ", Average_linear_speed)                           
print ("Final Collision counter = ",collision_counter)
