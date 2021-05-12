import rospy
import miro2 as miro2

from miro2.utils import wheel_speed2cmd_vel
from miro2.interface import vision

from std_msgs.msg import UInt16
from sensor_msgs.msg import Range, Imu, CompressedImage, JointState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import os
import math
import time
import random
import cv2
import numpy as np

import miro_constants as con
from apriltag_perception import AprilTagPerception

class Client:

    # Collision variables
    touch_body = False
    touch_head = False
    object_close = False
    resolve_collision = False
    resolve_collision_iterations = 300
    
    # Loop state
    state = "home"

    # Scoring variables
    score = 0
    last_score_change = time.time()  # Used to determine when last score update was
    SCORE_INCREASE = 10 # Determines how quickly score increases per second 

    # Hunger variables
    hunger_value = 100
    last_hunger_change = time.time() # Used to determine when last hunger value update was
    HUNGER_THRESHOLD = 95 # Determines the point where the robot will look for 
    HUNGER_DECREASE = 1 # Determines how quickly the hunger increases per second

    start_time = None # Used for time elapsed

    # Random walk variables
    WALK_STRAIGHT_LIMITS = (0, 5) # Range of time MiRO going straight in random walk
    WALK_TURN_LIMITS = (0,3) # Range of time MiRo turns in random walk
    walk_turn_direction = "R"
    
    # Random walk timing variables
    walk_straight_time = None
    walk_turn_time = None
    walk_start_turn_time = None 
    walk_start_straight_time = None
    walk_state = None

    # April tag variables
    APPROACH_DISTANCE = 0.4 # Distance to approach tag in meters
    tag_size = 0.2
    tag_distance = 0.8
    tag_family = 'tag36h11'
    tags_r = None
    tags_l = None
    FOOD_TAG_ID = 4
    HOME_TAG_ID = 3


    # General variables
    BROADCAST_RATE = 50
    MAX_WHEEL_SPEED = 0.25
    TICK = 0.01    
    ROBOT_NAME = '/' + os.getenv('MIRO_ROBOT_NAME')

    # Collision variables
    resolve_collision_counter = resolve_collision_iterations


    # Spiral Walk Variables	
    walk_type = "spiral" # Change walk type here spiral or random
    spiral_call = 0      # Number to times spiral function is called, radius will increase every time function is called
    iterations = 0       # Number of times main loop is iterated
    left_velocity = MAX_WHEEL_SPEED * 0.8     # left wheel velocity
    right_velocity = MAX_WHEEL_SPEED * 0.75   # right wheel velocity

    
    def touch_body_callback(self, data):
        # Touch body callback
        # Sets touch_body if miro's touch body sensor returns true
        if data.data > 0:
            self.touch_body = True


    def touch_head_callback(self, data):
        # Touch head callback
        # Sets touch_head if miro's touch head sensor returns true
        if data.data > 0:
            self.touch_head = True


    def sonar_callback(self, data):
        # Sonar callback
        # Sets object_close if object is within 5cm of sonar
        if data.range < 0.05:
            self.object_close = True


    def process_frame(self, frame):
        # Image input processing
        # Code taken from https://github.com/MiRo-projects/basic_functions/miro_ros_interface.py
        frame_array = np.frombuffer(frame.data, np.uint8)
        image_array = cv2.imdecode(frame_array, cv2.IMREAD_UNCHANGED)
        image_undistorted = cv2.undistort(image_array, con.MTX, con.DIST, None)
        return image_undistorted


    def callback_caml(self, frame):
        # Callback for left camera
        self.caml_undistorted = self.process_frame(frame)


    def callback_camr(self, frame):
        # Callback for right camera
        self.camr_undistorted = self.process_frame(frame)


    def detect_collision(self):
        # Detects collision
        # Uses the touch_body, touch_head and object_close to see if miro has had a collision
        if self.touch_body or self.touch_head or self.object_close:
            return True
        return False


    def reset_collision_markers(self):
        # Resets collision markers
        self.touch_body = False
        self.touch_head = False
        self.object_close = False
        self.resolve_collision_counter = self.resolve_collision_iterations


    def move_miro(self, vel_left, vel_right):
        # Moves miro based on:
        # vel_left - Left wheel velocity -1.0 - 1.0 m/s
        # vel_right - Right wheel velocity -1.0 - 1.0 m/s
        velocity_msg = TwistStamped()
        (dr, dtheta) = wheel_speed2cmd_vel([vel_left, vel_right])

        velocity_msg.twist.linear.x = dr
        velocity_msg.twist.angular.z = dtheta

        self.velocity.publish(velocity_msg)
    

    def collision_resolve(self):
        # Moves backwards whilst the collision isn't resolved
        if self.resolve_collision_counter > 0:
            self.move_miro(-0.1, -0.1)
            self.resolve_collision_counter -= 1
        
        # When the collision is resolved update loop state
        else:
            self.move_miro(0,0)
            self.resolve_collision_counter = 0
            self.set_walk_parameters # Chooses a new direction to walk if stuck in corner
            self.resolve_collision = False


    def check_alive(self):
        # Checks is miro has died from starvation
        if self.hunger_value >= 0:
            return True
        return False


    def decrease_hunger(self):
        # Decreases hunger value per second
        if (time.time()- self.last_hunger_change) >= 1:
            self.hunger_value -= self.HUNGER_DECREASE
            self.last_hunger_change = time.time()


    def increase_score(self):
        # Increases score if in nesting state
        if (time.time() - self.last_score_change) >= 1:
            self.score += self.SCORE_INCREASE
            self.last_score_change = time.time()


    def death(self):
        # Upon death output relevant stats
        print "Animal has died"
        print "Survival time - " + str(time.time() - self.start_time)
        print "Score - " + str(self.score)


    def check_food_search(self):
        # Checks if it needs to search for food
        if self.hunger_value <= self.HUNGER_THRESHOLD:
            self.state = "search_food"
            print "Searching for food"
            self.set_walk_parameters()


    def set_walk_parameters(self):
        # Sets initial timings and direction, starts walk with a turn 
        self.walk_straight_time = random.randint(self.WALK_STRAIGHT_LIMITS[0], self.WALK_STRAIGHT_LIMITS[1])
        self.walk_turn_time = random.randint(self.WALK_TURN_LIMITS[0], self.WALK_TURN_LIMITS[1])
        self.walk_turn_direction = random.choice(["R", "L"])
        self.walk_start_turn_time = time.time()
        self.walk_state = "turn"


    def random_walk(self):
        # Random walk code
        # 1. Turns left or right for pre-determined random time
        # 2. Moves straight for pre-determined random time
        # 3. Repeat
        if self.walk_state == "turn":
            if self.walk_turn_direction == "right":
                self.move_miro(-self.MAX_WHEEL_SPEED, self.MAX_WHEEL_SPEED)

            else:
                self.move_miro(self.MAX_WHEEL_SPEED, -self.MAX_WHEEL_SPEED)
            
            if (time.time() - self.walk_start_turn_time) >= self.walk_turn_time:
                self.walk_state = "straight"
                self.walk_start_straight_time = time.time()


        else:
            self.move_miro(self.MAX_WHEEL_SPEED, self.MAX_WHEEL_SPEED)

            if (time.time() - self.walk_start_straight_time) >= self.walk_straight_time:
                self.walk_state = "straight"
                self.walk_start_turn_time = time.time()

    def spiral_walk(self, iterations):
            # Spiral Walk code
            # 1. Moves in spiral motion
            # 2. Increase spiral radius on each function call to increase search space with time
            # 3. Repeat

            radius_increase = 0        # Increase of spiral radius per function call

            self.spiral_call += 1
            radius_increase =  (float(self.spiral_call) / float(iterations))
            self.left_velocity =  self.left_velocity * (1 - radius_increase)
            self.right_velocity = self.right_velocity * (1 + radius_increase)

            if self.left_velocity < self.MAX_WHEEL_SPEED and self.right_velocity < self.MAX_WHEEL_SPEED and  self.right_velocity > 0 and self.left_velocity > 0:
                self.move_miro(self.left_velocity, self.right_velocity)
            else:
               print ("Spiral Exceeding Speed Limit")
               self.move_miro(self.MAX_WHEEL_SPEED * 0.8 , self.MAX_WHEEL_SPEED * 0.7)

            # ------For Testing -------
            #print ("Radius Increase: " + str(radius_increase))
            #print ("Spiral Call Count: " + str(self.spiral_call))
            #print ("Iterations: " + str(iterations))
            #print ("Left Velocity: " + str(self.left_velocity))
            #print ("Right Velocity: " + str(self.right_velocity))
	        #print ("")		




    
    def tag_alignment(self, tag_id):
        # Checks in the tag with tag_id is in the field of view of left or right camera
        # Also outputs if the tag has been approached, within desired distance
        left_cam = False
        right_cam = False
        approached = False

        if self.caml_undistorted is not None:
            self.tags_l = self.apriltag_perception.detect_tags(self.caml_undistorted)
            if self.tags_l: 
                for tag in self.tags_l:
                    self.apriltag_perception.draw_box(self.caml_undistorted, tag, colour='green')
                    self.apriltag_perception.draw_center(self.caml_undistorted, tag, colour='red')
                    if tag.id == tag_id:
                        left_cam = True
                        if tag.distance <= self.APPROACH_DISTANCE:
                            approached = True 
            cv2.imshow('Left Cam', self.caml_undistorted)
        
        if self.camr_undistorted is not None:
            self.tags_r = self.apriltag_perception.detect_tags(self.camr_undistorted)

            if self.tags_r:
                for tag in self.tags_r:
                    self.apriltag_perception.draw_box(self.camr_undistorted, tag, colour='green')
                    self.apriltag_perception.draw_center(self.camr_undistorted, tag, colour='red')
                    if tag.id == tag_id:
                        right_cam = True
                        if tag.distance <= self.APPROACH_DISTANCE:
                            approached = True 
            cv2.imshow('Right Cam', self.camr_undistorted)

        return left_cam, right_cam, approached


        

    def __init__(self):
        rospy.init_node('object_localization', anonymous=True)
        rospy.sleep(2.0)
        print('Initializing')

        # Apriltag perception
        self.apriltag_perception = AprilTagPerception(size=self.tag_size, family=self.tag_family)

        # Sensor subscribers
        self.touch_body_sub = rospy.Subscriber(self.ROBOT_NAME + '/sensors/touch_body', UInt16, self.touch_body_callback, queue_size=1, tcp_nodelay=True)
        self.touch_head_sub = rospy.Subscriber(self.ROBOT_NAME + '/sensors/touch_head', UInt16, self.touch_head_callback, queue_size=1, tcp_nodelay=True)
        self.sonar = rospy.Subscriber(self.ROBOT_NAME + '/sensors/sonar', Range, self.sonar_callback, queue_size=1, tcp_nodelay=True)
        #self.body_vel = rospy.Subscriber(self.ROBOT_NAME + '/sensors/body_vel', TwistStamped, self.body_vel_callback, queue_size=1, tcp_nodelay=True)
        self.caml_sub = rospy.Subscriber(self.ROBOT_NAME + '/sensors/caml/compressed', CompressedImage, self.callback_caml)
        self.camr_sub = rospy.Subscriber(self.ROBOT_NAME + '/sensors/camr/compressed', CompressedImage, self.callback_camr)

        # Inits camera image variables
        self.caml_undistorted = None
        self.camr_undistorted = None

        # Control subscribers
        self.velocity = rospy.Publisher(self.ROBOT_NAME + '/control/cmd_vel', TwistStamped, queue_size=1, tcp_nodelay=True)
        self.kin_joints = rospy.Publisher(self.ROBOT_NAME + '/control/kinematic_joints', JointState, queue_size=0)
        rospy.sleep(2.0)


    def main_loop(self):
        # Main control loop
        print "Looping"
        print "At home"
        
        # Used for time elapsed
        self.start_time = time.time()

        # Loop runs whilst ros isn't shutdown and robot isn't dead
        while not rospy.core.is_shutdown() and self.check_alive():
            self.iterations += 1

            if self.detect_collision():
                # Detected collision
                print "Collision"
                self.reset_collision_markers()
                self.resolve_collision = True


            elif self.resolve_collision:
                # If resolving collision move backwards
                self.collision_resolve()


            elif self.state == "home":
                # Checks it doesn't need to search for food
                # Calls increase score, to increase score if enough time has elapsed since last increase
                self.check_food_search()
                self.increase_score()


            elif self.state == "search_food":
                # Conducts random walk
                # Checks food tag can be spotted
                # If tag spotted enter "food_found" state
                if self.walk_type == "spiral":                                       # check for walk mode
                    self.spiral_walk(self.iterations)
                else:
                    self.random_walk()
                tag_l, tag_r, approached = self.tag_alignment(self.FOOD_TAG_ID)

                if tag_l or tag_r:
                    self.state = "food_found"
                    print "Food Found, Approaching"



            elif self.state == "food_found":
                # Aligns tag so it's visible in both cameras and heads forwards
                # If tag is approached successfully then increase to full hunger
                # If tag is lost return to "search_food" state
                tag_l, tag_r, approached = self.tag_alignment(self.FOOD_TAG_ID)

                if approached:
                    self.hunger_value = 100
                    self.state = "return_home"
                    print "Searching for home"

                elif tag_l and not tag_r:
                    self.move_miro(-0.05, 0.05)

                elif tag_r and not tag_l:
                    self.move_miro(0.05, -0.05)

                elif tag_l and tag_r:
                    self.move_miro(self.MAX_WHEEL_SPEED, self.MAX_WHEEL_SPEED)

                else:
                    self.state = "search_food"
                    print "Searching for food"


            elif self.state == "return_home":
                # Conducts random walk
                # Checks home tag can be spotted
                # If tag spotted enter "home_found" state
                if self.walk_type == "spiral":                                       # check for walk mode
                    self.spiral_walk(self.iterations)
                else:
                    self.random_walk()
                tag_l, tag_r, approached = self.tag_alignment(self.HOME_TAG_ID)

                if tag_l or tag_r:
                    self.state = "home_found"
                    print "Home Found, Approaching"


                self.check_food_search()


            elif self.state == "home_found":
                # Aligns tag so it's visible in both cameras and heads forwards
                # If tag is approached successfully then enter "home" state
                # If tag is lost return to "search_home" state
                tag_l, tag_r, approached = self.tag_alignment(self.FOOD_TAG_ID)

                if approached:
                    self.state = "home"
                    print "At home"

                elif tag_l and not tag_r:
                    self.move_miro(-0.05, 0.05)

                elif tag_r and not tag_l:
                    self.move_miro(0.05, -0.05)

                elif tag_l and tag_r:
                    self.move_miro(self.MAX_WHEEL_SPEED, self.MAX_WHEEL_SPEED)

                else:
                    self.state = "search_home"

                self.check_food_search()


            self.decrease_hunger()
            rospy.sleep(self.TICK)
        self.death()

if __name__ == "__main__":
    client = Client()
    client.main_loop()
