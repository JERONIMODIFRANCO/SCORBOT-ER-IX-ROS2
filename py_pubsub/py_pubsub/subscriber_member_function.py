# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import serial


import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ida',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # ser = serial.Serial('/dev/ttyUSB0', 9600)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s", publishing via Serial' % msg.data)
        self.get_logger().info('Enviando comando via Serial "%s"' % msg.data)
        ser = serial.Serial('/dev/ttyACM0', 4800,8,"N",1,None,True)
        serial_msg = bytes(msg.data, 'utf-8')
        ser.write(b'%b\n' % serial_msg)
        self.data = str(ser.readline())
        msg = String()
        msg.data = self.data
        # msg.data = 'Hello World: %d' % 2
        self.get_logger().info('%s' % msg.data)
        ser.close()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
