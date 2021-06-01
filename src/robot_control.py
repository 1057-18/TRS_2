#!/usr/bin/env python

import numpy as np
import rospy
import std_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Moving around:
            w
       a    s    d
            x

  w/x : increase/decrease linear velocity
  a/d : increase/decrease angular velocity
  s : stop

  ^C to EXIT
"""

chs = """
Choose robot control regimen:
    m) manual
    a) automatic
    q) quit
"""

def get_key(settings):
    if os.name == 'nt':
        if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
        else:
            return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def make_simple_profile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

def check_linear_limit_velocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

    return vel

def check_angular_limit_velocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

    return vel

def rotation_matrix(theta):
    return  np.array([[np.math.cos(theta), np.math.sin(theta), 0], [-np.math.sin(theta), np.math.cos(theta), 0], [0, 0, 1]])

def translation_matrix(tx, ty):
    return np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]])

def run_teleop():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    os.system('cls' if os.name == 'nt' else 'clear')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0
    
    print(msg)
    while (1):
        key = get_key(settings)
        if key == 'w':
            target_linear_vel = check_linear_limit_velocity(target_linear_vel + LIN_VEL_STEP_SIZE)
            os.system('cls' if os.name == 'nt' else 'clear')
            print(msg)
            print(vels(target_linear_vel, target_angular_vel))
        elif key == 'x':
            target_linear_vel = check_linear_limit_velocity(target_linear_vel - LIN_VEL_STEP_SIZE)
            os.system('cls' if os.name == 'nt' else 'clear')
            print(msg)
            print(vels(target_linear_vel, target_angular_vel))
        elif key == 'a':
            target_angular_vel = check_angular_limit_velocity(target_angular_vel + ANG_VEL_STEP_SIZE)
            os.system('cls' if os.name == 'nt' else 'clear')
            print(msg)
            print(vels(target_linear_vel, target_angular_vel))
        elif key == 'd':
            target_angular_vel = check_angular_limit_velocity(target_angular_vel - ANG_VEL_STEP_SIZE)
            os.system('cls' if os.name == 'nt' else 'clear')
            print(msg)
            print(vels(target_linear_vel, target_angular_vel))
        elif key == ' ' or key == 's':
            target_linear_vel = 0.0
            control_linear_vel = 0.0
            target_angular_vel = 0.0
            control_angular_vel = 0.0
            os.system('cls' if os.name == 'nt' else 'clear')
            print(msg)
            print(vels(target_linear_vel, target_angular_vel))
        else:
            if (key == '\x03'):
                break

        twist = Twist()

        control_linear_vel = make_simple_profile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = control_linear_vel;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0
    
        control_angular_vel = make_simple_profile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = control_angular_vel

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def callback(odom, args):
    global_x = args[0]
    global_y = args[1]
    
    h1 = True
    sign = None

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    os.system('cls' if os.name == 'nt' else 'clear')
    
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
   
    orien = odom.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w]) 
    
    print(f'current position: x={np.round(odom.pose.pose.position.x, 4)}, y={np.round(odom.pose.pose.position.y, 4)}')
    print(f'current orientation: theta={theta}')

    R = rotation_matrix(theta)
    T = translation_matrix(-x, -y)
    W = np.dot(R, T)
    coordinates = np.dot(W, np.array([[global_x], [global_y], [1]]))
    
    local_x = coordinates[0][0]
    local_y = coordinates[1][0]

    rho = (local_x**2 + local_y**2)**1/2
    gamma = np.arctan(local_y/local_x)
    alpha = -theta + gamma
    beta = -theta - alpha

    k_rho = 3
    k_alpha = 4
    k_beta = -6

    if h1:
        if local_x >= 0:
            h1 = False
            sign = 1
        else:
            h1 = False
            sign = -1

    v = sign * k_rho * rho
    w = k_alpha * alpha + k_beta * beta

    twist = Twist()

    twist.linear.x = constrain(v, -MAX_LIN_VEL, MAX_LIN_VEL);
    twist.linear.y = 0.0;
    twist.linear.z = 0.0

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = constrain(w, -MAX_ANG_VEL, MAX_ANG_VEL)

    pub.publish(twist)

def run_controller():
    os.system('cls' if os.name == 'nt' else 'clear')
    target = input('Enter target coordinates: ').split(' ')
    x = float(target[0])
    y = float(target[1])
    sub = rospy.Subscriber('odom', Odometry, callback, (x, y))
    rospy.spin()

def choose_control_regimen():
    rospy.init_node('robot_controller')

    while(1):
        os.system('cls' if os.name == 'nt' else 'clear')
        print(chs + '\n')
        while(1):
            val = input('Enter desired option: ')
            if val in ['m', 'a', 'q']:
                break
            else:
                os.system('cls' if os.name == 'nt' else 'clear')
                print(chs)
                print('Wrong inpit. Please enter valid option. ---> ("m", "a", "q")')
        if val == 'm':
            run_teleop()
        if val == 'a':
            run_controller()
        else:
            os.system('cls' if os.name == 'nt' else 'clear')
            break

if __name__ == '__main__':
    try:
        choose_control_regimen()
    except rospy.ROSInterruptException:
        pass
