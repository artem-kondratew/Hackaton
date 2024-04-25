import sys
import threading
import json
import os

from robot_msgs.msg import OmegaAngles
from std_msgs.msg import UInt8
from  geometry_msgs.msg import Twist
import rclpy

import termios
import tty

config_file = os.path.dirname(__file__) + "/../config/config.json"
with open(config_file) as handle:
    config = json.loads(handle.read())

pub_cam_topic = config["pub_cam_topic"]
pub_grip_topic = config["pub_grip_topic"]
pub_task_topic = config["pub_task_topic"]

MAX_CAM_V_ANGLE = config["max_cam_v_angle"]
MIN_CAM_V_ANGLE = config["min_cam_v_angle"]
MAX_CAM_H_ANGLE = config["max_cam_h_angle"]
MIN_CAM_H_ANGLE = config["min_cam_h_angle"]

MAX_GRIP_V_ANGLE = config["max_grip_v_angle"]
MIN_GRIP_V_ANGLE = config["min_grip_v_angle"]
MAX_GRIP_H_ANGLE = config["max_grip_h_angle"]
MIN_GRIP_H_ANGLE = config["min_grip_h_angle"]


msg = """
This node takes keypresses from the keyboard and publishes them to /control/gripper_angles and /control/camera_angles
as robot_msgs/OmegaAngles.msg messages. It works best with a US keyboard layout.
---------------------------
Moving around:
        i     
   j    k    l
        ,     

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
---------------------------
Camera Control:
s/a : decrease/increase vertical angle by 5
d/f : decrease/increase horizontal angle by 5
---------------------------
Gripper Control:
r/t : decrease/increase vertical angle by 5
y/u : decrease/increase horizontal angle by 5
---------------------------
Task Control:
1-4 : choose required number of task
1 : line following
2 : break
3 : beep
4 : blinking
---------------------------
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    ',': (-1, 0, 0, 0),
    'k': (0, 0, 0, 0)
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9)
}

cameraBindings = {
    'a': (5, 0),
    's': (-5, 0),
    'f': (0, 5),
    'd': (0, -5)
}

gripperBindings = {
    't': (5, 0),
    'r': (-5, 0),
    'u': (0, 5),
    'y': (0, -5)
}

taskBindings = {"1", "2", "3", "4"}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())  # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def angles(name, angle_v, angle_h):
    return f'currently:\t{name} angles {angle_v} {angle_h}'

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)



def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('hmi_control')

    pub_cam = node.create_publisher(OmegaAngles, pub_cam_topic, 10)
    pub_grip = node.create_publisher(OmegaAngles, pub_grip_topic, 10)
    pub_task = node.create_publisher(UInt8, pub_task_topic, 10)
    pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    
    camera_horiz = 80
    camera_vert = 90
    gripper_horiz = 40
    gripper_vert = 5

    status = 0.0

    angles_msg = OmegaAngles()
    curr_task = UInt8()
    twist_msg = Twist()

    camera_msg = angles_msg
    gripper_msg = angles_msg
    twist = twist_msg

    try:
        print(msg)
        print(angles("camera", camera_vert, camera_horiz))
        print(angles("gripper", gripper_vert, gripper_horiz))
        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

                twist.linear.x = x * speed
                twist.linear.y = y * speed
                twist.linear.z = z * speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th * turn

                pub_cmd_vel.publish(twist)

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

                twist.linear.x = x * speed
                twist.linear.y = y * speed
                twist.linear.z = z * speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th * turn

                pub_cmd_vel.publish(twist)

            elif key in cameraBindings.keys():
                camera_vert += cameraBindings[key][0]
                camera_horiz += cameraBindings[key][1]

                if camera_vert < MIN_CAM_V_ANGLE:
                    camera_vert = MIN_CAM_V_ANGLE
                if camera_vert > MAX_CAM_V_ANGLE:
                    camera_vert = MAX_CAM_V_ANGLE

                if camera_horiz < MIN_CAM_H_ANGLE:
                    camera_horiz = MIN_CAM_H_ANGLE
                if camera_horiz > MAX_CAM_H_ANGLE:
                    camera_horiz = MAX_CAM_H_ANGLE
                
                camera_msg.vert_angle = camera_vert
                camera_msg.horiz_angle = camera_horiz

                print(angles("camera", camera_vert, camera_horiz))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

                pub_cam.publish(camera_msg)

            elif key in gripperBindings.keys():
                gripper_vert += gripperBindings[key][0]
                gripper_horiz += gripperBindings[key][1]

                if gripper_vert < MIN_GRIP_V_ANGLE:
                    gripper_vert = MIN_GRIP_V_ANGLE
                if gripper_vert > MAX_GRIP_V_ANGLE:
                    gripper_vert = MAX_GRIP_V_ANGLE

                if gripper_horiz < MIN_GRIP_H_ANGLE:
                    gripper_horiz = MIN_GRIP_H_ANGLE
                if gripper_horiz > MAX_GRIP_H_ANGLE:
                    gripper_horiz = MAX_GRIP_H_ANGLE

                gripper_msg.vert_angle = gripper_vert
                gripper_msg.horiz_angle = gripper_horiz

                print(angles("gripper", gripper_vert, gripper_horiz))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                
                pub_grip.publish(gripper_msg)

            elif key in taskBindings:
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

                curr_task.data = int(key)
                print(f'Current task number is {curr_task.data}')

                pub_task.publish(curr_task)

            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            

    except Exception as e:
        print(e)

    finally:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub_cmd_vel.publish(twist)

        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()