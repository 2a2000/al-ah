#!/usr/bin/python3
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
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import smbus
import time
bus = smbus.SMBus(1)
bus.write_byte_data(0x44, 0x01, 0x05)


def getColour():
    data=bus.read_i2c_block_data(0x44,0x09,6)
    red= data[3]*256 +data [2] 
    
    print("RED = (%d)" % (red))
    return int(red)

# Constants
LINEAR_VEL = 0.22
v1 = 0.17
v2 = 0.20
turn_distance = 0.5
STOP_DISTANCE = 0.2  
LIDAR_ERROR = 0.05
escape_distance = 0.1 
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

# Global variables
victims_counter = 0
red_flag = False
collision = False 
collision_counter = 0
speed_updates = 0.01
speed_accumulation = 0

front_right_distance = 0.1
front_left_distance = 0.1
right_distance = 0.1
left_distance = 0.1
back_distance = 0.1 
lidar_distances = []
start_time = time.time()


class Obstacle():                                       

    global collision_counter 
    global speed_updates
    global speed_accumulation
    twist = Twist()

    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.obstacle()


    def turn_left(self, ang_Vel, LiN_vel):
        self.twist.angular.z = ang_Vel      
        self.twist.linear.x = LiN_vel
        self._cmd_pub.publish(self.twist)
        rospy.loginfo('Turning to left')

    def turn_right (self,ang_Vel, LiN_vel):
        self.twist.angular.z = -ang_Vel
        self.twist.linear.x = LiN_vel
        self._cmd_pub.publish(self.twist)
        rospy.loginfo('Turning to right')


    def turn_around(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 2.8
        self._cmd_pub.publish(self.twist)
        rospy.loginfo('Turning around')


    def reverse(self, Lvelocity):
        self.twist.linear.x = -Lvelocity
        self.twist.angular.z = 0
        self._cmd_pub.publish(self.twist)
        rospy.loginfo('Moving back')


    def forward (self):
        self.twist.linear.x = LINEAR_VEL
        self.twist.angular.z = 0
        self._cmd_pub.publish(self.twist)
        rospy.loginfo('Moving Forward!')
    

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
        



        front_angel = 0
        front2_angel = 359
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


        left_range = tuple(filter_zeros(scan.ranges[front_left_angel:left_angel]))
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
        return scan_filter

    
    def obstacle(self):
        global turtlebot_moving
        turtlebot_moving = True
        global collision_counter 
        global speed_updates
        global speed_accumulation
        global front_right_distance
        global front_left_distance
        global right_distance
        global left_distance
        global back_distance
        global victims_counter 
        global red_flag
        global collision
        global lidar_distances
        global start_time 

        while (time.time()-start_time) < 120 :
            print("Control Loop")
            speed_updates += 1
            lidar_distances = self.get_scan()
            if lidar_distances :
                closest_object = min(lidar_distances)
            else :
                 print ("NO READING AT ALL")
                 closest_object = 0.10
                 
            # Case 1
            if ( front_right_distance > turn_distance and front_left_distance > turn_distance ):
                    print("FLEFT distance = \n",front_left_distance)
                    print("FRIGHT distance = \n",front_right_distance)
                    self.forward()
                    speed_accumulation += LINEAR_VEL 

            #Case 4
            elif ( front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE ): 
                if ( front_right_distance < escape_distance and front_left_distance < escape_distance ):
                    print("000 Obstacle infront(Too close to turn)")
                    self.reverse(0.6*LINEAR_VEL)
                    rospy.sleep(0.4)
                    self.turn_around ()
                    rospy.sleep(0.5)
                    self.turn_around ()
                    speed_accumulation += 0.6*LINEAR_VEL
                elif ( front_right_distance > escape_distance and front_left_distance > escape_distance ):
                    if (right_distance > SAFE_STOP_DISTANCE):
                            print("11 FLEFT distance = \n",front_left_distance)
                            print("11 FRIGHT distance = \n",front_right_distance)
                            print ("Obstacle infront")
                            self.turn_right(1.7,v1)
                            speed_accumulation += v1 
                    elif (left_distance > SAFE_STOP_DISTANCE):
                            print("22FLEFT distance = \n",front_left_distance)
                            print("22FRIGHT distance = \n",front_right_distance)
                            print ("Obstacle  infront")
                            self.turn_left(1.7,v1)
                            speed_accumulation += v1
                    elif (front_right_distance < SAFE_STOP_DISTANCE and front_left_distance < SAFE_STOP_DISTANCE and right_distance  < SAFE_STOP_DISTANCE  and left_distance < SAFE_STOP_DISTANCE):  
                        self.reverse(0.6*LINEAR_VEL)
                        rospy.sleep(0.4)
                        speed_accumulation += 0.6*LINEAR_VEL 
                        self.turn_around() 
                        rospy.sleep(0.5)
                        self.turn_around ()
                        print ("33 All roads blocked. Backing up")
 


            # Case 2
            elif ( front_right_distance < SAFE_STOP_DISTANCE and front_left_distance > SAFE_STOP_DISTANCE ):  
                if front_right_distance > escape_distance:
                        if (left_distance > SAFE_STOP_DISTANCE):
                                self.turn_left(1.7,v1) 
                                speed_accumulation += v1
                                print("44 FLEFT distance = \n",front_left_distance)
                                print("44 FRIGHT distance = \n",front_right_distance)
                                print ("Obstacle  Right infront")
                        elif right_distance > SAFE_STOP_DISTANCE : 
                                self.turn_right(1.7,v1)
                                speed_accumulation += v1 
                                print("55 FLEFT distance = \n",front_left_distance)
                                print("55 FRIGHT distance = \n",front_right_distance)                                
                                print ("Obstacle Right infront")  
                        elif right_distance < SAFE_STOP_DISTANCE  and left_distance < SAFE_STOP_DISTANCE :
                                self.reverse(0.6*LINEAR_VEL)
                                rospy.sleep(0.4)
                                speed_accumulation += LINEAR_VEL 
                                self.turn_around()
                                rospy.sleep(0.5)
                                self.turn_around()
                                print("66 FLEFT distance = \n",front_left_distance)
                                print("66 FRIGHT distance = \n",front_right_distance)
                                print ("Obstacle found at Front right and at front left")
                else:
                        print("77 FLEFT distance = \n",front_left_distance)
                        print("77 FRIGHT distance = \n",front_right_distance)
                        self.reverse(0.6*LINEAR_VEL)
                        rospy.sleep(0.4)
                        print ("Obstacle at Front right")
                        self.turn_left(1.7,v1) # turn left after moving back, because the obstacle is at front right
                        speed_accumulation += LINEAR_VEL*0.6 + v1 

            


            # Case 3
            elif ( front_left_distance < SAFE_STOP_DISTANCE and front_right_distance > SAFE_STOP_DISTANCE ):  
                    if front_left_distance > escape_distance: 
                        if (right_distance > SAFE_STOP_DISTANCE):
                                print("88 FLEFT distance = \n",front_left_distance)
                                print("88 FRIGHT distance = \n",front_right_distance)
                                self.turn_right(1.7,v1)
                                speed_accumulation += v1 
                                print ("Obstacle found infront left ")
                        elif left_distance > SAFE_STOP_DISTANCE :
                                print("99 FLEFT distance = \n",front_left_distance)
                                print("99 FRIGHT distance = \n",front_right_distance)
                                self.turn_left(1.7,v1)
                                speed_accumulation += v1 
                                print ("Obstacle at front left")
                        elif right_distance < SAFE_STOP_DISTANCE and left_distance < SAFE_STOP_DISTANCE :
                            print("101 FRONT LEFT distance = \n",front_left_distance)
                            print("101 FRONT RIGHT distance = \n",front_right_distance)
                            print ("Obstacle at front left and at right ")
                            self.reverse(0.6*LINEAR_VEL)
                            rospy.sleep(0.4)
                            self.turn_around() 
                            rospy.sleep(0.5)
                            self.turn_around ()
                            speed_accumulation += 0.6*LINEAR_VEL
                    else:
                        print("202 FRONT LEFT distance = \n",front_left_distance)
                        print("202 FRONT RIGHT distance = \n",front_right_distance)
                        print ("Obstacle at front left")
                        self.reverse(0.6*LINEAR_VEL)
                        rospy.sleep(0.4)
                        self.turn_around() 
                        rospy.sleep(0.5)
                        self.turn_around ()
                        speed_accumulation += 0.6*LINEAR_VEL  


                        #////////////////////////////Making small turns 50 cm away from obstacle ///////////////////////////////            
            elif ( front_right_distance < turn_distance and front_left_distance < turn_distance ): 
                    if (right_distance > SAFE_STOP_DISTANCE- 0.5):#-5 because we will not make big turn here so we dont need the same space as usual                    
                            self.turn_right(0.7,v2)                     
                            speed_accumulation += v2  
                    elif (left_distance > SAFE_STOP_DISTANCE- 0.5):                                           
                            self.turn_left(0.7,v2)               
                            speed_accumulation += v2 
            elif ( front_right_distance < turn_distance and front_left_distance > turn_distance ):  
                        if (left_distance > SAFE_STOP_DISTANCE- 0.5):
                                self.turn_left(0.7,v2) 
                                speed_accumulation += v2                       
                        elif right_distance > SAFE_STOP_DISTANCE- 0.5: 
                                self.turn_right(0.7,v2)                            
                                speed_accumulation += v2                          
            elif ( front_left_distance < turn_distance and front_right_distance > turn_distance ):  
                        if (right_distance > SAFE_STOP_DISTANCE - 0.5): 
                                self.turn_right(0.7,v2)                       
                                speed_accumulation += v1                           
                        elif left_distance > SAFE_STOP_DISTANCE - 0.5:                  
                                self.turn_left(0.7,v2)
                                speed_accumulation += v2 



        
                #Collision Counter with cooldown
            if collision == False:
                if closest_object <= (0.1) :
                    collision_counter += 1
                    collision = True
                    print ("Collision counter = ",collision_counter)
            else:
                 if closest_object > (0.1):
                      collision = False
                     
           
           # Victim counter with cooldown
            if red_flag == False:
                if (getColour() > 7750) and  (getColour() < 8050):
                    red_flag = True
                    victims_counter += 1 
                    print ("Victim Found")
            else:
                if (getColour() <= 7750) and (getColour() >= 8050) :
                    red_flag = False
                    print ("No red anymore")
                  



def main():

    global speed_accumulation
    global speed_updates
    global collision_counter
    global victims_counter 
rospy.init_node('turtlebot3_obstacle')
try:
    obstacle = Obstacle()
except rospy.ROSInterruptException:
    pass


if True:
    main()  
Average_linear_speed=speed_accumulation/speed_updates
print("Average linear speed = ", Average_linear_speed)                           
print ("Final Collision counter = ",collision_counter)
print ("Final Victims counter = ",victims_counter)