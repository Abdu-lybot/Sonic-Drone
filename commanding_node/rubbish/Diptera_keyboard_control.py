#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
import Diptera_arm2

import sys, select, termios, tty


class keyboard_control():
    msg = """
    Abdussalam.alajami@upf.edu
    Reading from the keyboard  and Publishing to attitude mavros msg!
    ---------------------------
    Moving around:
       q    w    e
       a    s    d
       z    x    .
    For Holonomic mode (strafing), hold down the shift key:
    ---------------------------
        U    I    O
        J    K    L
        M    <    >
        t : up (+z)
        b : down (-z)
        anything else : let diptera scripts send
        q/z : increase/decrease max speeds by 10%
        b/n : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        CTRL-C to quit
        """


    def __init__(self):
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.countx = 0
        self.county = 0
        self.countz = 0
        self.count_thrust = 0
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0
        self.acc_thrust = 0
        self.movement = str
        self.moveBindings = {
            'w': (0, 0.100, 0, 0),    #pith forward
            's': (0, -0.100, 0, 0),   #pitch backward
            'd': (0.100, 0, 0, 0 ),   #roll right
            'a': (-0.100, 0, 0, 0),   #roll left
            'e': (0, 0, 0, 0.1),    #assend +z
            'q': (0, 0, 0, -0.01),  #decend -z
            'z': (0, 0, 0.1, 0),    #yaw cw
            'x': (0, 0, -0.1, 0),   #yaw ccw
            #'O': (1, -1, 0, 0),
            #'I': (1, 0, 0, 0),
            #'J': (0, 1, 0, 0),
            #'L': (0, -1, 0, 0),
            #'U': (1, 1, 0, 0),
            #'<': (-1, 0, 0, 0),
            #'>': (-1, -1, 0, 0),
            #'M': (-1, 1, 0, 0),
            #'t': (0, 0, 1, 0),
            #'b': (0, 0, -1, 0),
        }

        self.speedBindings = {
            #'q': (1.1, 1.1),
            #'z': (.9, .9),
            #'i': (1.1, 1),
            #'x': (.9, 1),
            #'e': (1, 1.1),
            #'c': (1, .9),
        }

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        #self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def vels(self, speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def print_movement(self,movement ,speed):
        if movement == 'Forward':
            print("w is pressed = increasing PITCH by",speed)
        elif movement == 'Backward':
            print("s is pressed = increasing PITCH by",speed)
        elif movement == 'Right':
            print("d is pressed = increasing ROLL by",speed)
        elif movement == 'Left':
            print("a is pressed = increasing ROLL by",speed)
        elif movement == '+UP':
            print("e is pressed = increasing THROTTLE by",speed)
        elif movement == '+Down':
            print("q is pressed = decreasing THROTTLE by",speed)
        elif movement == 'yawCW':
            print("z is pressed = increasing YAW-CW by",speed)
        elif movement == 'yawCCW':
            print("x is pressed = increasing YAW-CCW by",speed)

kb = keyboard_control()
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    rospy.init_node('drone_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    #x = 0
    #y = 0
    #z = 0
    #th = 0
    status = 0

    try:
        print(kb.msg)
        print(kb.vels(speed,turn))
        while(1):
            key = kb.getKey()
            if key in kb.moveBindings.keys():
                x = kb.moveBindings[key][0]
                if x == 0.1:
                    kb.countx = kb.countx + 100
                    kb.acc_x = x + kb.countx
                    kb.movement = 'Right'
                    kb.print_movement(kb.movement,kb.acc_x)
                elif x == -0.1:
                    kb.countx = kb.countx - 100
                    kb.acc_x = x + kb.countx
                    kb.movement = 'Left'
                    kb.print_movement(kb.movement,kb.acc_x)
                y = kb.moveBindings[key][1]
                if y == 0.1:
                    kb.county = kb.county + 100
                    kb.acc_y = y + kb.county
                    kb.movement = 'Forward'
                    kb.print_movement(kb.movement, kb.acc_y)
                elif y == -0.1:
                    kb.county = kb.countx - 100
                    kb.acc_y = y + kb.county
                    kb.movement = 'Backward'
                    kb.print_movement(kb.movement, kb.acc_y)
                z = kb.moveBindings[key][2]
                if z == 0.1:
                    kb.countz = kb.countz + 0.1
                    kb.acc_z = z + kb.county
                    kb.movement = 'yawCW'
                    kb.print_movement(kb.movement,kb.acc_z)
                elif z == -0.1:
                    kb.countz = kb.countz - 0.1
                    kb.acc_z = z + kb.countz
                    kb.movement = 'yawCCW'
                    kb.print_movement(kb.movement,kb.acc_z)
                th = kb.moveBindings[key][3]
                if th == 0.1:
                    kb.count_thrust = kb.count_thrust + 0.1
                    kb.acc_thrust = th + kb.count_thrust
                    kb.movement = '+UP'
                    kb.print_movement(kb.movement,kb.acc_thrust)
                elif th == -0.01:
                    kb.count_thrust = kb.count_thrust - 0.01
                    kb.acc_thrust = th + kb.count_thrust
                    kb.movement = '+Down'
                    kb.print_movement(kb.movement,kb.acc_thrust)
            elif key in kb.speedBindings.keys():
                speed = speed * kb.speedBindings[key][0]
                turn = turn * kb.speedBindings[key][1]

                print(kb.vels(speed,turn))
                if (status == 14):
                    print(kb.msg)
                status = (status + 1) % 15
            else:
                #x = 0
                #y = 0
                #z = 0
                #th = 0
                if (key == '\x03'):
                    break
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = kb.imu.orientation
            target_raw_attitude.body_rate.x = kb.acc_x  # ROLL_RATE
            target_raw_attitude.body_rate.y = kb.acc_y  # PITCH_RATE
            target_raw_attitude.body_rate.z = kb.acc_z  # YAW_RATE
            target_raw_attitude.thrust = kb.acc_thrust
            attitude_target_pub.publish(target_raw_attitude)

            #twist = Twist()
            #twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
            #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            #pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        print("waiting for input from user to command and control")
        '''
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.orientation = kb.imu.orientation
        target_raw_attitude.body_rate.x = 0  # ROLL_RATE
        target_raw_attitude.body_rate.y = 0  # PITCH_RATE
        target_raw_attitude.body_rate.z = 0  # YAW_RATE
        target_raw_attitude.thrust = 
        attitude_target_pub.publish(target_raw_attitude)
        '''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
