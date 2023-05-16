#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

def calculate_avg(arr):
    temp_arr = []
    for i in range (len(arr)):
        # print(f"arr['{i}']: '{arr[i]}' | ")
        if arr[i] < 10 and arr[i] > 0.2 and not math.isinf(arr[i]) and not math.isnan(arr[i]):
            temp_arr.append(arr[i])
    np_arr = np.array(np.array(temp_arr))
    # return np.sum(np_arr) / len(np_arr)
    return np_arr.min()

    # print(f"np.sum(arr): '{np.sum(arr)}'")
    # print(f"len(arr): '{len(arr)}'")
    # return np.sum(arr) / len(arr)

class Publisher(): 

        #callback function for the scanning. SOMETIMES min distance refuses to work
        #think i fixed it, but don't quote me on that.
    def scan_callback(self, scan_data):

        ps_arr = [0,0,0,0,0,0,0,0]
        self.ps = [0,0,0,0,0,0,0,0]
        self.pre_ps = [10,10,10,10,10,10,10,10]
        self.pre_top = 10
        # print(f"scan_data: '{len(scan_data.ranges)}'")

        # ps_arr[7] = scan_data.ranges[330:359]
        # ps_arr[6] = scan_data.ranges[293:330]
        # ps_arr[5] = scan_data.ranges[247:293]
        # ps_arr[4] = scan_data.ranges[180:247]

        ps_arr[0] = scan_data.ranges[-30:-9]
        ps_arr[1] = scan_data.ranges[-67:-30]
        ps_arr[2] = scan_data.ranges[-113:-67]
        ps_arr[3] = scan_data.ranges[-180:-113]

        top_arr = scan_data.ranges[-15:0] + scan_data.ranges[0:15]

        ps_arr[7] = scan_data.ranges[9:30]
        ps_arr[6] = scan_data.ranges[30:67]
        ps_arr[5] = scan_data.ranges[67:113]
        ps_arr[4] = scan_data.ranges[113:180]

        # print(f"scan_data.ranges[0]: '{scan_data.ranges[0]}'")
        # print(f"scan_data.ranges[15]: '{scan_data.ranges[15]}'")
        # print(f"scan_data.ranges[45]: '{scan_data.ranges[45]}'")
        # print(f"scan_data.ranges[90]: '{scan_data.ranges[90]}'")
        # print(f"scan_data.ranges[150]: '{scan_data.ranges[150]}'")
        # print(f"scan_data.ranges[210]: '{scan_data.ranges[210]}'")
        # print(f"scan_data.ranges[270]: '{scan_data.ranges[270]}'")
        # print(f"scan_data.ranges[315]: '{scan_data.ranges[315]}'")
        # print(f"scan_data.ranges[345]: '{scan_data.ranges[345]}'\n")

        for i in range(8):
            # print(f"\nps_arr['{i}']:")
            if len(ps_arr[i]) == 0:
                self.ps[i] = self.pre_ps[i]
            else:
                self.ps[i] = calculate_avg(ps_arr[i])
                self.pre_ps[i] = self.ps[i]

        # print(f"\ntop_arr:")
        if len(top_arr) == 0:
            self.top = self.pre_top
        else:
            self.top = calculate_avg(top_arr)
            self.pre_top = self.top

        self.scan_ready = True

    def go(self):

        self.vel_cmd.angular.z = 0.0
        self.vel_cmd.linear.x = 0.15

    def stop(self):

        self.vel_cmd.angular.z = 0.0
        self.vel_cmd.linear.x = 0.0

    def turn_left(self):

        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.3

    def turn_right(self):

        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = -0.3
    
    def delay(self, time):

        rospy.sleep(time)

    def turn_left_to_back(self):

        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.6   
        self.delay(1)

    def turn_right_to_back(self):

        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = -0.6
        self.delay(1)
    
    def check_distance(self):
        TOF_MAX = 0.6
        if self.top <= TOF_MAX:
            #stop or turn
            return 1
        #go
        return 0

    def check_proximity_turn(self):

        PROXIMITY_MAX = 0.5
        FLAT_TEST_DIFF = 0.5

        print(f"ps7: '{self.ps[7]}'")
        print(f"ps6: '{self.ps[6]}'")
        print(f"ps5: '{self.ps[5]}'")
        print(f"ps4: '{self.ps[4]}'")
        print(f"ps0: '{self.ps[0]}'")
        print(f"ps1: '{self.ps[1]}'")
        print(f"ps2: '{self.ps[2]}'")
        print(f"ps3: '{self.ps[3]}'")

        print(f"top: '{self.top}'")        

        left_sum = self.ps[5] + self.ps[6] + self.ps[7]
        right_sum = self.ps[0] + self.ps[1] + self.ps[2]

        print(f"left_sum: '{left_sum}'")
        print(f"right_sum: '{right_sum}'")

        tof_flag = self.check_distance()

        print(f"tof_flag: '{tof_flag}'")

        if tof_flag == 1 or self.ps[0] < PROXIMITY_MAX or self.ps[7] < PROXIMITY_MAX or self.ps[1] < PROXIMITY_MAX or self.ps[6] < PROXIMITY_MAX or self.ps[2] <  PROXIMITY_MAX or self.ps[5] < PROXIMITY_MAX:
            if tof_flag == 0 and self.ps[0] > 1.2 * PROXIMITY_MAX and self.ps[7] > 1.2 * PROXIMITY_MAX and self.ps[1] > 0.8 * PROXIMITY_MAX and self.ps[6] > 0.8 * PROXIMITY_MAX and self.ps[2] > 0.4 * PROXIMITY_MAX and self.ps[5] > 0.4 * PROXIMITY_MAX:
                return 0
            else:
                if (left_sum > right_sum) and ((left_sum - right_sum) < FLAT_TEST_DIFF):
                    # turn right to back
                    return 4
                elif (right_sum > left_sum) and ((right_sum - left_sum) < FLAT_TEST_DIFF):
                    # turn left to back
                    return 5
                elif left_sum > right_sum:
                    # turn left
                    return 1
                else:
                    # turn right
                    return 2
        else:
            # go straight
            return 0

    def __init__(self): 
        self.scan_ready = False
        self.node_name = "simple_publisher" 
        topic_name = "cmd_vel" 

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 

        self.subscan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.on_shutdown(self.shutdownhook)

        self.vel_cmd = Twist()

    
        self.rate = rospy.Rate(10) 
        self.vel_cmd = Twist()

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):

        DangerDistance = 0.5
        ObstacleDistance = 0.5
        TurnSpeed = 0

        flag = 3

        while not self.ctrl_c:

            if self.scan_ready:

                flag = self.check_proximity_turn()
                print(f"(main_loop)flag: '{flag}'")

                if flag == 0:
                    self.go()
                elif flag == 1:
                    self.turn_left()
                elif flag == 2:
                    self.turn_right()
                elif flag == 3:
                    self.stop()
                elif flag == 4:
                    self.turn_right_to_back()
                elif flag == 5:
                    self.turn_left_to_back()
                else:
                    self.stop()

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Publisher() 
    publisher_instance.__init__()
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
