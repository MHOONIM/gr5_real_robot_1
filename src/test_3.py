#!/usr/bin/env python3

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

# ------------------------------------ LaserScan class start ----------------------------------------
class scan_subscriber():

    def __init__(self):
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_data):
        # North side -- 1
        f_left_arc = scan_data.ranges[21:0]
        f_right_arc = scan_data.ranges[-20:]
        front_arc = np.array(f_left_arc[::-1] + f_right_arc[::-1])
        self.min_front = front_arc.min() # <-- Minimum of front arc

        # North East side -- 2
        neast_arc_1 = scan_data.ranges[-35:-45]
        neast_arc_2 = scan_data.ranges[-55:-45]
        neast_arc = np.array(neast_arc_1[::-1] + neast_arc_2[::-1])
        self.min_neast = neast_arc.min()

        # East side -- 3
        east_arc_1 = scan_data.ranges[-80:-90]
        east_arc_2 = scan_data.ranges[-100:-90]
        east_arc = np.array(east_arc_1[::-1] + east_arc_2[::-1])
        self.min_east = east_arc.min()

        # South East side -- 4
        seast_arc_1 = scan_data.ranges[-115:-135]
        seast_arc_2 = scan_data.ranges[-145:-135]
        seast_arc = np.array(seast_arc_1[::-1] + seast_arc_2[::-1])
        self.min_seast = seast_arc.min()

        # Optional Extra: Angular angle of the object
        # arc_angles = np.arange(-20, 21)
        # self.object_angle = arc_angles[np.argmin(front_arc)]
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

        # ------- get the first map (to make sure we've got a map) -----
        self.ros_l.launch(roslaunch.core.Node(
                    package="map_server",
                    node_type="map_saver",
                    args=f"-f {self.map_file}"))
        self.time = rospy.get_time()

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True  

    # --------- Create path for saving the map ----------
    def path_map(self):
        
        pkg_path = rospkg.RosPack().get_path('gr5_real_robot_1')
        map_path = Path(pkg_path).joinpath("maps")
        map_path.mkdir(exist_ok=True)
        self.map_file = map_path.joinpath('gr5_map_test')

        self.ros_l = roslaunch.scriptapi.ROSLaunch()
        self.ros_l.start() 
    # ---------------------------------------------------

    def main_loop(self):

        while not self.ctrl_c:

            # Get the distance data from the LaserScan
            min_front_dis = self.data_scan.min_front
            min_east_dis = self.data_scan.min_east
            min_neast_dis = self.data_scan.min_neast
            min_seast_dis = self.data_scan.min_seast

            # print(f'The min front distance : {min_front_dis}')
            print(f'tb3_location_x : {self.tb3_odom.x}')

            if min_front_dis < 0.5:
                # There's a wall up ahead --> Sharp turn left
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.3

                # Too close to the bottom --> Right turn
                if min_seast_dis < 0.3:
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = -0.3
            else:
                # No wall ahead --> Moving forward
                self.vel.linear.x = 0.2
                self.vel.angular.z = 0

                # The wall not detected --> Moving straight to find the wall
                if min_east_dis > 0.5:
                    self.vel.linear.x = 0.2
                    self.vel.angular.z = 0
                else:
                    # the wall is detected --> Turn right to approach the wall
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = -0.4
                
                # Too close to the wall --> Turn left to get away from the wall
                if min_neast_dis < 0.5:
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0.2

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)

            # maintain the loop rate @ 10 hz
            self.rate.sleep()   

            # print(f'rostime = {rospy.get_time()}')
            # Update the map after navigation in every 5 seconds
            if (rospy.get_time() - self.time) > 5:
                self.ros_l.launch(roslaunch.core.Node(
                        package="map_server",
                        node_type="map_saver",
                        args=f"-f {self.map_file}")) 
                self.time = rospy.get_time()    
# ------------------------------------ Explore class end ----------------------------------------------


# ---------------------------------- Main program -----------------------------------------------------
if __name__ == "__main__":
    node = searching_test()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
