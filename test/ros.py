import rclpy
import serial
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
import numpy as np
import math
from threading import Lock


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        # Replace 'COMX' with the actual port name
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.timer_ = self.create_timer(0.02, self.timer_callback)
        self.pwm = 0
        self.response = 0
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.mutex = Lock()

    def joy_callback(self, msg):
        value_to_send = int(msg.axes[0]*255)
        self.pwm = value_to_send

    def timer_callback(self):

        self.mutex.acquire()

        try:

            dataToSend = f"{self.pwm},{2323},{-1},\n".encode('utf-8')

            self.ser.write('m'.encode('utf-8'))
            self.ser.write(dataToSend)

            # response = self.ser.readline().strip()

            # self.get_logger().info(f"sent {dataToSend} rec {response}")
            # response=0

            self.ser.write('e'.encode('utf-8'))
            response = self.ser.readline().strip()  # read(3) will read 3 bytes
            print(f'Recived data {int(response)}')

        finally:
            self.mutex.release()

        motor_1 = TransformStamped()
        # motor_2 = TransformStamped()
        # motor_3 = TransformStamped()

        motor_1.header.stamp = self.get_clock().now().to_msg()
        motor_1.header.frame_id = 'world'
        motor_1.child_frame_id = 'motor_1'

        motor_1.transform.translation.x = 0.5
        motor_1.transform.translation.y = 0.0
        motor_1.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, int(response)/420*np.pi)
        motor_1.transform.rotation.x = q[0]
        motor_1.transform.rotation.y = q[1]
        motor_1.transform.rotation.z = q[2]
        motor_1.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(motor_1)


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
