#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class Publisher(): 

        #callback function for the scanning. SOMETIMES min distance refuses to work
    #think i fixed it, but don't quote me on that.
    def scan_callback(self, scan_data):
        
        
        #this sets the range for the obstacle avoidance on the left and right
        #these need to be larger so it can keep moving whilst avoiding objects
        left_arc = scan_data.ranges[0:41]
        right_arc = scan_data.ranges[-40:]
        

        #this sets the arc for stopping and turning
        #this is minimal, but enough so that a head on collision is avoided
        #basically; collision avoidance.
        frontleft = scan_data.ranges[0:21]
        frontright = scan_data.ranges[-20:]
        
        #the values are combined to make front_arc
        front_arc = np.array(frontleft+ frontright)
       
        #min_distance is the min distance of just the FRONT ARCs
        #if this is too small then the system has to stop and turn.
        self.min_distance = front_arc.min()
        

        #then we set up the left and right arcs for obstacle avoidance.
        left = np.array(left_arc)
        right = np.array(right_arc)
        
        self.min_left = left.min()
        self.min_right = right.min()
        self.max_left = left.max()
        self.max_right = right.max()
        self.scan_ready = True
             
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

        while not self.ctrl_c:

            if self.scan_ready:
                # print(f" self.min_distance '{self.min_distance}'")
                # print(f" self.min_left '{self.min_left}'")
                # print(f" self.min_right '{self.min_right}'")

                if self.min_distance < DangerDistance or self.min_left < ObstacleDistance or self.min_right < ObstacleDistance:
                    # print(f" self.min_distance '{self.min_distance}'")
                    # print(f" self.min_left '{self.min_left}'")
                    # print(f" self.min_right '{self.min_right}'")
                    self.vel_cmd.linear.x = 0
                    if abs(self.min_left - self.min_right) < 0.03:
                        print(f"stuck '{self.min_left}' ii '{self.min_right}'")
                        print(f"stuck '{self.min_left - self.min_right}'")
                        print(f" self.max_left '{self.max_left}'")
                        print(f" self.max_right '{self.max_right}'")
                        if self.max_left > self.max_right:
                            TurnSpeed = 1.8
                        else:
                            TurnSpeed = -1.8
                    elif self.min_left < self.min_right:
                        TurnSpeed = -0.2
                    elif self.min_right < self.min_left:
                        TurnSpeed = 0.2
                else :    
                    self.vel_cmd.linear.x = 0.3 # m/s
                    TurnSpeed = 0
            
            self.vel_cmd.angular.z = TurnSpeed # rad/s
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Publisher() 
    publisher_instance.__init__()
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass
