import sys
import threading

from robot_msgs.msg import OmegaAngles
from std_msgs.msg import UInt8
import rclpy

import termios
import tty



msg = """
This node takes keypresses from the keyboard and publishes them to /control/gripper_angles and /control/camera_angles
as robot_msgs/OmegaAngles.msg messages. It works best with a US keyboard layout.
---------------------------
Camera Control:
r/f : increase/decrease vertical angle by 5
e/q : increase/decrease horizontal angle by 5
---------------------------
Gripper Control:
w/s : increase/decrease vertical angle by 5
d/a : increase/decrease horizontal angle by 5
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

cameraBindings = {
    'r': (5, 0),
    'f': (-5, 0),
    'e': (0, 5),
    'q': (0, -5),
}

gripperBindings = {
    'w': (5, 0),
    's': (-5, 0),
    'd': (0, 5),
    'a': (0, -5),
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


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('hmi_control_node')

    pub_cam = node.create_publisher(OmegaAngles, '/control/camera_angles', 10)
    pub_grip = node.create_publisher(OmegaAngles, '/control/gripper_angles', 10)
    pub_task = node.create_publisher(UInt8, '/control/task_number', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    camera_horiz = 0  # TODO: MAKE CORRECT INITIAL VALUES
    camera_vert = 0
    gripper_horiz = 0
    gripper_vert = 0

    status = 0.0

    angles_msg = OmegaAngles()
    curr_task = UInt8()

    camera_msg = angles_msg
    gripper_msg = angles_msg


    try:
        print(msg)
        while True:
            key = getKey(settings)
            print(key)
            if key in cameraBindings.keys():
                camera_vert += cameraBindings[key][0]
                camera_horiz += cameraBindings[key][1]

                if camera_vert < 0:
                    camera_vert = 0
                if camera_vert > 255:
                    camera_vert = 255

                if camera_horiz < 0:
                    camera_horiz = 0
                if camera_horiz > 255:
                    camera_horiz = 255
                
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

                if gripper_vert < 0:
                    gripper_vert = 0
                if gripper_vert > 255:
                    gripper_vert = 255

                if gripper_horiz < 0:
                    gripper_horiz = 0
                if gripper_horiz > 255:
                    gripper_horiz = 255

                gripper_msg.vert_angle = gripper_vert
                gripper_msg.horiz_angle = gripper_horiz

                print(angles("gripper", gripper_vert, gripper_horiz))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                
                pub_grip.publish(gripper_msg)

            elif key in taskBindings:
                print(f'Current task number is {key}')
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

                curr_task.data = int(key)

                pub_task.publish(curr_task)

            else:
                if (key == '\x03'):
                    break

            

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()