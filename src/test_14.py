#!/usr/bin/env python3
# check out more on : https://github.com/MHOONIM/gr5_real_robot_1

import rospy 
from geometry_msgs.msg import Twist # import the Twist message for publishing velocity commands:
from nav_msgs.msg import Odometry # import the Odometry message for subscribing to the odom topic:
from tf.transformations import euler_from_quaternion # import the function to convert orientation from quaternions to angles:
from sensor_msgs.msg import LaserScan # import the LaserScan
from math import sqrt, pow, pi # import some useful mathematical operations (and pi), which you may find useful:
import numpy as np
import roslaunch
import rospkg
from pathlib import Path
from time import sleep
import math

# ------------------------------------ LaserScan class start ----------------------------------------
class scan_subscriber():

    def __init__(self):

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_data):

        lidar_left = scan_data.ranges[360:0]
        lidar_right = scan_data.ranges[-360:]
        surrounding_arc = np.array(lidar_left[::-1] + lidar_right[::-1])

        # Filter out the upper bound and lower bound
        for i in range (len(surrounding_arc)):
            if surrounding_arc[i] > 1:
                surrounding_arc[i] = 10
            elif surrounding_arc[i] < 0.2:
                surrounding_arc[i] = 10
        
        # North side -- 1
        self.front_arc_l = surrounding_arc[339:359]
        self.front_arc_r = surrounding_arc[0:20]
        self.front_arc = np.concatenate((self.front_arc_l, self.front_arc_r))
        self.min_front = self.front_arc.min() # <-- Minimum of front arc
        self.avg_front = np.sum(self.front_arc) / len(self.front_arc)

        # North East side -- 2
        self.neast_arc = surrounding_arc[30:60]
        self.min_neast = self.neast_arc.min()
        self.avg_neast = np.sum(self.neast_arc) / len(self.neast_arc)

        # # East side -- 3
        self.east_arc = surrounding_arc[70:110]
        self.min_east = self.east_arc.min()
        self.avg_east = np.sum(self.east_arc) / len(self.east_arc)

        # # South East side -- 4
        self.seast_arc = surrounding_arc[120:150]
        self.min_seast = self.seast_arc.min()
        self.avg_seast = np.sum(self.seast_arc) / len(self.seast_arc)

        # North west -- 5
        self.nwest_arc = surrounding_arc[300:325]
        self.min_nwest = self.nwest_arc.min()

        # West -- 6
        self.west_arc = surrounding_arc[260:280]
        self.min_west = self.west_arc.min()

        # South West -- 7
        self.swest_arc = surrounding_arc[215:235]
        self.min_swest = self.swest_arc.min()

        # Optional Extra: Angular angle of the object
        arc_angles = np.arange(-20, 21)
        self.object_angle = arc_angles[np.argmin(self.front_arc)]
# ------------------------------------ LaserScan class end ------------------------------------------------

# ------------------------------------ Odometry class start -----------------------------------------------
class get_odom():

    def __init__(self):

        self.sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.initial = True


    def odom_callback(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.initial:
            # don't initialise again:
            self.initial = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
# ------------------------------------ Odometry class end -----------------------------------------------

# ------------------------------------ Explore class start ----------------------------------------------
class searching_test():

    def __init__(self):

        # define node name
        node_name = "searching_test"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        
        # create objects that subscribe to the topics
        self.data_scan = scan_subscriber() # <---- create data_scan object that subscribe to the LaserScan topic
        self.tb3_odom = get_odom() # <---- create tb3_odom object that subscribe to the Odometry topic

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

        self.path_map() # <---- Call the path_map function

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        # initialise the exploration parameters
        self.locate = False # initialise the destination locating flag
        self.dest_flag = 1 # initialise the destination flag
        self.get_target(self.dest_flag) # get the first destination coordinates
        self.prev_beta = self.tb3_odom.theta_z
        self.beta = self.prev_beta

        print(f'the bot is reaching to the destination {self.dest_flag}')
        print(f'coordinate_x : {self.target_x}')
        print(f'coordinate_y : {self.target_y}')

        # ------- get the first map (to make sure we've got a map) -----
        self.ros_l.launch(roslaunch.core.Node(
                    package="map_server",
                    node_type="map_saver",
                    args=f"-f {self.map_file}"))
        self.time = rospy.get_time()

        # Initialise PID parameters
        self.kp = 0.5
        self.kd = 0.1
        self.prev_error = 0
        self.error = 0

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True  

    # --------- Create path for saving the map ---------------
    def path_map(self):
        
        pkg_path = rospkg.RosPack().get_path('gr5_real_robot_1')
        map_path = Path(pkg_path).joinpath("maps")
        map_path.mkdir(exist_ok=True)
        self.map_file = map_path.joinpath('gr5_map_test')

        self.ros_l = roslaunch.scriptapi.ROSLaunch()
        self.ros_l.start() 
    # --------------------------------------------------------

    # ---------- Get the next coordinates to explore ---------
    def get_target(self, destination):
        if destination == 0:
            self.target_x = 0
            self.target_y = 0
        elif destination == 1:
            self.target_x = -1.3
            self.target_y = 0
        elif destination == 2:
            self.target_x = -1.3
            self.target_y = -1.3
        elif destination == 3:
            self.target_x = 0
            self.target_y = -1.3
        elif destination == 4:
            self.target_x = 1.3
            self.target_y = -1.3
        elif destination == 5:
            self.target_x = 1.3
            self.target_y = 0
        elif destination == 6:
            self.target_x = 1.3
            self.target_y = 1.3
        elif destination == 7:
            self.target_x = 0
            self.target_y = 1.3
        elif destination == 8:
            self.target_x = -1.3
            self.target_y = 1.3
    # --------------------------------------------------------

    def main_loop(self):

        while not self.ctrl_c:

            # Get the distance data from the LaserScan
            min_front_dis = self.data_scan.min_front
            min_neast_dis = self.data_scan.min_neast
            min_east_dis = self.data_scan.min_east
            min_seast_dis = self.data_scan.min_seast
            min_nwest_dis = self.data_scan.min_nwest
            min_west_dis = self.data_scan.min_west
            min_swest_dis = self.data_scan.min_swest
            
            # print(f'The minimum front distance : {min_front_dis}')
            # print(f'The average front distance : {avg_front_dis}')
            # print(f'tb3_location_x : {self.tb3_odom.x}')

            # Compute for the angle diff
            beta = self.tb3_odom.theta_z
            if beta < 0:
                beta = pi + (pi + beta)

            dis_y = self.target_y - self.tb3_odom.y
            dis_x = self.target_x - self.tb3_odom.x
            alpha = math.atan((dis_y) / (dis_x))
        
            if dis_x > 0 and dis_y > 0:
                # if the target is in the quadrant 1
                alpha = alpha * 1
            elif dis_x <= 0 and dis_y > 0:
                # if the target is in the quadrant 2
                alpha = pi - alpha
            elif dis_x <= 0 and dis_y <= 0:
                # if the target is in the quadrant 3
                alpha = (2 * pi) - alpha
            else:
                # if the target is in the quadrant 4
                alpha = alpha + (2 * pi)
           
            a_diff = beta - alpha

            # print(f'aplha = {alpha}')
            # print(f'beta = {beta}')
            # print(f'gamma = {a_diff}')
            # print(f'target_x : {self.target_x}')
            # print(f'target_y : {self.target_y}')
            # print(f'dis_x : {dis_x}')
            # print(f'dis_y : {dis_y}')

            # Locate the destination by making the angle diff close to 0.
            if self.locate == False: 
                self.error = a_diff
                angular_speed = (self.kp * self.error) + (self.kd * (self.error - self.prev_error))
                self.prev_error = self.error

                if angular_speed > 1:
                    angular_speed = 1
                elif angular_speed < -1:
                    angular_speed = -1
                
                if a_diff > 0.15:
                    self.vel.linear.x = 0
                    self.vel.angular.z = angular_speed * -1
                elif a_diff < -0.15:
                    self.vel.linear.x = 0
                    self.vel.angular.z = angular_speed * -1
                else:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    self.locate = True

            else:
                if (self.tb3_odom.x > self.target_x + 0.2 or self.tb3_odom.x < self.target_x - 0.2) or (self.tb3_odom.y > self.target_y + 0.2 or self.tb3_odom.y < self.target_y - 0.2):
                    
                    #Calculate for the speed
                    self.error = math.sqrt((self.target_x - self.tb3_odom.x)**2 + (self.target_y - self.tb3_odom.y)**2)
                    linear_speed = self.error * self.kp + ((self.error - self.prev_error) * self.kd)
                    if linear_speed > 0.26:
                        linear_speed = 0.26
                    # print(f'linear_speed = {linear_speed}')
                    self.prev_error = self.error

                    if a_diff < 0.15 and a_diff > -0.15:
                        if min_front_dis > 0.5:
                            self.vel.linear.x = linear_speed
                            self.vel.angular.z = 0
                        else:
                            self.vel.linear.x = 0
                            self.vel.angular.z = 0.3
                    else:
                        if min_front_dis < 0.5:
                            # There's a wall up ahead --> Sharp turn left
                            self.vel.linear.x = 0.0
                            self.vel.angular.z = 0.3

                            # Too close to the bottom --> Right turn <------- THIS CONDITION IS NOT GOOD IN REAL ROBOT
                            # if min_seast_dis < 0.5:
                            #     self.vel.linear.x = 0.1
                            #     self.vel.angular.z = -0.3
                        else:
                            # No wall ahead --> Moving forward
                            self.vel.linear.x = linear_speed
                            self.vel.angular.z = 0

                            # The wall not detected --> Moving straight to find the wall
                            if min_east_dis > 0.5:
                                self.vel.linear.x = linear_speed
                                self.vel.angular.z = 0
                            else:
                                # the wall is detected --> Turn right to approach the wall
                                self.vel.linear.x = 0.1
                                self.vel.angular.z = -0.3
                            
                            # Too close to the wall --> Turn left to get away from the wall
                            if min_neast_dis < 0.5:
                                self.vel.linear.x = 0.1
                                self.vel.angular.z = 0.2

                else:
                    # the bot reachs the destination --> stop, update dest_flag, get a new destination and clear locating flag
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0

                    print(f'The destination {self.dest_flag} have been reached.')

                    self.dest_flag = self.dest_flag + 1 # update destination flag

                    if self.dest_flag > 8:
                        self.dest_flag = 1

                    self.get_target(self.dest_flag) # get a new destination
                    self.locate = False # clear locating flag

                    print(f'The next destination is {self.dest_flag}')
                    print(f'coordinate_x : {self.target_x}')
                    print(f'coordinate_y : {self.target_y}')

                    # print(f'rostime = {rospy.get_time()}')
                    # Update the map after navigation in every 10 seconds
                    self.ros_l.launch(roslaunch.core.Node(
                                            package="map_server",
                                            node_type="map_saver",
                                            args=f"-f {self.map_file}")) 
                    self.time = rospy.get_time()   

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)

            # maintain the loop rate @ 10 hz
            self.rate.sleep()    
# ------------------------------------ Explore class end ----------------------------------------------


# ---------------------------------- Main program -----------------------------------------------------
if __name__ == "__main__":
    node = searching_test()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
