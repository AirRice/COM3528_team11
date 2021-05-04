from bluetoother_scanner import BluetootherScanner

import rospy
import miro2 as miro2

from miro2.utils import wheel_speed2cmd_vel
from miro2.interface import vision

from std_msgs.msg import UInt16
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import os
import math

from bluetoother_scanner import BluetootherScanner

class Client:

    TOUCH_BODY = False
    TOUCH_HEAD = False
    OBJECT_CLOSE = False
    RESOLVE_COLLISION = False
    RESOLVE_COLLISION_ITERATIONS = 300
    BROADCAST_RATE = 50
    MAX_WHEEL_SPEED = 0.25
    STATE = "home"


    TICK = 0.01    
    ROBOT_NAME = '/' + os.getenv('MIRO_ROBOT_NAME')

    resolve_collision_counter = RESOLVE_COLLISION_ITERATIONS
    angle_to_origin = 0

    def touch_body_callback(self, data):
        # Touch body callback
        # Sets TOUCH_BODY if miro's touch body sensor returns true
        if data.data > 0:
            self.TOUCH_BODY = True

    def touch_head_callback(self, data):
        # Touch head callback
        # Sets TOUCH_HEAD if miro's touch head sensor returns true
        if data.data > 0:
            self.TOUCH_HEAD = True

    def sonar_callback(self, data):
        # Sonar callback
        # Sets OBJECT_CLOSE if object is within 5cm of sonar
        if data.range < 0.05:
            self.OBJECT_CLOSE = True

    def body_vel_callback(self, data):
        # Angle update
        # Updated angle_to_origin based 
        self.angle_to_origin += data.twist.angular.z * (180 / math.pi) / self.BROADCAST_RATE
        self.angle_to_origin %= 360

    def detect_collision(self):
        # Detects collision
        # Uses the TOUCH_BODY, TOUCH_HEAD and OBJECT_CLOSE to see if miro has had a collision
        if self.TOUCH_BODY or self.TOUCH_HEAD or self.OBJECT_CLOSE:
            return True
        return False

    def reset_collision_markers(self):
        # Resets collision markers
        self.TOUCH_BODY = False
        self.TOUCH_HEAD = False
        self.OBJECT_CLOSE = False
        self.resolve_collision_counter = self.RESOLVE_COLLISION_ITERATIONS

    def move_miro(self, vel_left, vel_right):
        # Moves miro based on:
        # vel_left - Left wheel velocity -1.0 - 1.0 m/s
        # vel_right - Right wheel velocity -1.0 - 1.0 m/s
        velocity_msg = TwistStamped()
        (dr, dtheta) = wheel_speed2cmd_vel([vel_left, vel_right])

        velocity_msg.twist.linear.x = dr
        velocity_msg.twist.angular.z = dtheta

        self.velocity.publish(velocity_msg)
    

    def resolve_collision(self):
        # Moves backwards whilst the collision isn't resolved
        if self.resolve_collision_counter > 0:
            self.move_miro(-0.1, -0.1)
            self.resolve_collision_counter -= 1
        
        # When the collision is resolved update loop state
        else:
            self.move_miro(0,0)
            self.resolve_collision_counter = 0
            self.RESOLVE_COLLISION = False


    def __init__(self):
        rospy.init_node('object_localization', anonymous=True)
        rospy.sleep(2.0)
        print('Initializing')

        # Sensor subscribers
        self.touch_body = rospy.Subscriber(self.ROBOT_NAME + '/sensors/touch_body', UInt16, self.touch_body_callback, queue_size=1, tcp_nodelay=True)
        self.touch_head = rospy.Subscriber(self.ROBOT_NAME + '/sensors/touch_head', UInt16, self.touch_head_callback, queue_size=1, tcp_nodelay=True)
        self.sonar = rospy.Subscriber(self.ROBOT_NAME + '/sensors/sonar', Range, self.sonar_callback, queue_size=1, tcp_nodelay=True)
        self.body_vel = rospy.Subscriber(self.ROBOT_NAME + '/sensors/body_vel', TwistStamped, self.body_vel_callback, queue_size=1, tcp_nodelay=True)

        # Control subscribers
        self.velocity = rospy.Publisher(self.ROBOT_NAME + '/control/cmd_vel', TwistStamped, queue_size=1, tcp_nodelay=True)


    def main_loop(self):
        print('Looping')
        while not rospy.core.is_shutdown():
            if self.detect_collision():
                self.reset_collision_markers()
                self.RESOLVE_COLLISION = True

            elif self.RESOLVE_COLLISION:
                self.resolve_collision()


            elif self.STATE == "home":
                # Code for when home
                pass


            elif self.STATE == "search_food":
                # Code for searching for food
                pass


            elif self.STATE == "food_found":
                # Code for food found
                pass

            elif self.STATE == "return_home":
                # Code for returning home
                pass

            else:
                pass

            rospy.sleep(self.TICK)

if __name__ == "__main__":
    client = Client()
    client.main_loop()

# Points to check:
# Correct angles
# Correct magnitudes