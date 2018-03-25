import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2, pi, pow, sqrt, sin, cos
from std_msgs.msg import Empty
from time import time
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import *
from random import randint
# A class to create object instance of the attractive force toward the goal
class Attractive:
        def __init__(self,Xg,Yg,Xc,Yc,Rg,Sg):
            self.Xg = Xg                     # x co-ord of the goal
            self.Yg = Yg                     # y co-ord of the goal
            self.Xc = Xc                     # x co-ord of the robot
            self.Yc = Yc                     # y co-ord of the robot
            self.Rg = Rg                     # radius of the goal
            self.Sg = Sg                     # radius of the attractive field

        # calculate the attractive force to the goal, returns delta
        def check_dist_goal(self):
            delta = Point()
            alpha = 350                  # a constant for the attractive field (100 for open spaces)

            #goal statement
            dg = sqrt(pow((self.Xg-self.Xc),2)+pow((self.Yg-self.Yc),2))
            thg = atan2(self.Yg-self.Yc,self.Xg-self.Xc)
            if dg < self.Rg:
                delta.x = delta.y = 0
            #if within search field
            if self.Rg <= dg <= self.Sg + self.Rg:
                delta.x = alpha*((dg-self.Rg))*cos(thg)
                delta.y = alpha*((dg-self.Rg))*sin(thg)
            #if outside search field
            if dg > self.Sg + self.Rg:
                delta.x = alpha*self.Sg*cos(thg)
                delta.y = alpha*self.Sg*sin(thg)
            return delta

# A class to create object instance of the repulsive force from obstacles
class Repulsive:
        def __init__(self,do,tho,Xc,Yc,Ro,So):
            self.do = do                   # distance to obstacle
            self.tho = tho                 # angle to obstacle
            self.Xc = Xc                   # x co-ord of the robot
            self.Yc = Yc                   # y co-ord of the robot
            self.Ro = Ro                   # radius of object (provides a safety margin)
            self.So = So                   # radius of the repulsive field

        # calculate the repulsive force from an obstacle, returns delta
        def check_dist_goal(self):
            delta = Point()
            beta = 500                    # a constant for the repulsive field
            #goal statement
            if self.do < self.Ro:
                delta.x = -1*np.sign(cos(self.tho))*10000    # repulsion becomes very large if too close to object
                delta.y = -1*np.sign(sin(self.tho))*10000    # repulsion becomes very large if too close to object
            #if within search field
            if self.Ro <= self.do <= self.So + self.Ro:
                delta.x = -beta*(self.So+self.Ro-self.do)*cos(self.tho)
                delta.y = -beta*(self.So+self.Ro-self.do)*sin(self.tho)
            #if outside search field
            if self.do > self.So + self.Ro:
                delta.x = delta.y = 0
            return delta


#Implementation
current_x = 0.0                      # current x co-ord of the robot (global)
current_y = 0.0                      # current y co-ord of the robot (global)
current_th = 0.0                     # current orientation of the robot (global)
goal = Point()                       # goal co-ordinates (global)
goal.x = 0                           # instanciate x co-ord of goal
goal.y = 0                           # instanciate y co-ord of goal
delta = Point()                      # delta (global)
delta.x = delta.y = 0                # instanciate delta x and y
resetted = True                      # Has the odometry been reset? Boolean (global)
dist = 10                            # instanciate dist (cannot be zero or linear velocity calculation does not function)
# Laser scan callback - steering method
def steering(data):
    global delta
    global goal
    global current_x
    global current_y
    global resetted
    global dist

    # Checks that the odometry hsas been reset
    if(resetted == False):
        return
    # Checks if a goal has been set
    if goal.x == goal.y == 0:
        return


    Fa = Attractive(goal.x,goal.y,current_x,current_y,0.5,20) # Attractive force  0.5 = radius of goal, 20 = radius of field
    #dealing with nan values
    laser = np.asarray(data.ranges)                           # Converts to a np array
    laser = np.nan_to_num(laser)                              # changes all nan values to 0
    laser = np.where(laser == 0, data.range_max + 10, laser)  # laser is temp array that converts 0 (nan) values to maximum range
    laser = np.where(laser > 30, data.range_max + 10, laser)  # nan is where the distance is outwith range

    # Assignments for the laser
    Fr = [0]*10                                               # array for segments
    temp = Point()                                            # temporary point variable
    dist = 10                                                 # dist set to 10 or linear velocity function doesn't work
    i = 0
    th = -0.5*pi                                              # adds 90 degrees to th to make vectors curve around obstacle
    delta.x = delta.y = 0                                     # reinitialize delta
    ranges = (len(laser))/10                                  # calculate steps for loop based on size of data. works with gazebo and turtlebot

    #arr inxeding may need flipped for a VM
    #repulsive loop in steps of 10:
    for i in range(10):
        arr = (min(laser[i*ranges:(i+1)*ranges-1]))              # the closest obstacle in this range
        th = (((i*18)+9)-90)*(pi/180)                            # the middle angle of the range
        #find lowest collision distance
        if(arr < dist):
            dist = arr
        #closest collision distance's x and y components calcuated using angle th and current pose angle
        temp.x = current_x + arr*cos(th+current_th)
        temp.y = current_y + arr*sin(th+current_th)
        tho = atan2(temp.y-current_y,temp.x-current_x)
        Fr = Repulsive(arr, tho,current_x,current_y,0.6,0.8)    # repulsive force. Constants adjusted with tests
        #sum all of the forces in variable delta
        delta.x = delta.x + Fr.check_dist_goal().x
        delta.y = delta.y + Fr.check_dist_goal().y
        #print "arr: %s at %s" %(arr,tho)
        #print "FRx: %s FRy: %s" %(Fr.check_dist_goal().x,Fr.check_dist_goal().y)

    delta.x = delta.x + Fa.check_dist_goal().x
    delta.y = delta.y + Fa.check_dist_goal().y

# odometry function to retrieve position of robot
def Odom(msg):
    global current_x
    global current_y
    global current_th

    # Checks that the odometry has been reste
    if(resetted == False):
        return

    current_x = msg.pose.pose.position.x                                        # Set global variable for x co-ord
    current_y = msg.pose.pose.position.y                                        # Set global variable for y co-ord
    roll = pitch = current_th = 0.0

    rot_q = msg.pose.pose.orientation

    #obtain the angle 'yaw' using a quaternion to euler converter
    (roll, pitch, current_th) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #convert the angle to degrees
#Rviz goal callback from move_base_simple/Goal sub - GoalPose
def GoalPose(data):
    global goal
    goal.x = current_x + data.pose.position.x
    goal.y = current_y + data.pose.position.y
    print "_____goalx: %s goaly: %s _____" %(goal.x,goal.y)

pub_g = rospy.Subscriber('/move_base_simple/goal', PoseStamped, GoalPose)

#set up nodes
rospy.init_node("speed_controller", anonymous = True)                           # Node
sub = rospy.Subscriber("/odom", Odometry, Odom)                                 # Odometry subscriber
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size =1)        # Publisher to move robot
speed = Twist()

# set up the odometry reset publisher
reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)  # Publisher to reset the Odometry
scan_sub = rospy.Subscriber('/scan', LaserScan, steering)                       # Subscriber to get info from the laser

# reset odometry values (these messages take a few iterations to get through)
timer = time()
#the longer the timer set the more accurate the odometry initializes
while time() - timer <1.5:                                                      # 1.5 second delay.  This seems to improve odometry accuracy on reset
    reset_odom.publish(Empty())
resetted = True
r = rospy.Rate(10)

# Main method
while not rospy.is_shutdown():

# use tan to find the angle needed to turn towards the goal
    angle_to_goal = atan2(delta.y,delta.x) #tanx = O/A

# find the difference between the angle of the bot and angle needed to turn
    angle =  angle_to_goal - current_th

# convert the angle to a usable number
    if angle < (-pi):
        angle = angle + (2 * pi)
    if angle >= pi:
        angle = angle - (2 * pi)
#robot does not move if goal has not been set
    if (goal.x == goal.y == 0):
        speed.angular.z = 0
        speed.linear.x = 0
#robot will stop and turn to reorient itself once goal is reached
    elif (0.4 >= abs(goal.x-current_x) and 0.4 >= abs(goal.y-current_y)): # goal conditions
        speed.angular.z = 0
        speed.linear.x = 0
        #orient robot to origional pose
        angle = 0 - current_th
        #convert the angle to a usable number
        if angle < (-pi):
            angle = angle + (2 * pi)
        if angle >= pi:
            angle = angle - (2 * pi)
        speed.angular.z = 0.5*angle
        print ("x: %s y: %s th: %s angle: %s" % (current_x, current_y, current_th, angle))
    else:
        turnspeed = 0.4*angle
        speed.linear.x = max(0.05, 0.2 - (0.05/(dist)))       # dynamically control linear velocity using distance to objects
        speed.angular.z = turnspeed                     # dynamically control angular velocity
        print "x:%s,y%s" %(abs(goal.x-current_x),abs(goal.y-current_y))
    pub.publish(speed)
    r.sleep()
rospy.spin()
