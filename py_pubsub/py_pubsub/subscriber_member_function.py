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
import struct


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
        ser = serial.Serial('/dev/ttyACM0', 9600,8,"N",1,None,True)
        # ser.timeout = 1
        # serial_msg = bytes(msg.data, 'utf-8')
        serial_msg = msg.data.encode('utf-8')
        self.get_logger().info('%s' % msg.data)
        self.get_logger().info('%s' % str(serial_msg))
        # ser.write(b'%b\n' % serial_msg)
        # ser.write(b'%b\n' % bytearray.fromhex('R050803'))
        # self.data = ser.readline()
        # byte_data = self.data.decode('utf-8')
        # file = open('/home/gaston/ros2_ws/src/SCORBOT-ER-IX-ROS2/py_pubsub/Prueba.txt', 'a')
        # file.write('%s' % str(byte_data))
        # file.close()
        Encabezado = ser.read(3)
        Dato1 = ser.readline()
        Dato2 = ser.readline() 
        Dato3 = ser.readline()
        file = open('/home/gaston/ros2_ws/src/SCORBOT-ER-IX-ROS2/py_pubsub/Prueba.txt', 'a')
        if(msg.data == 'C'):
            # self.get_logger().info('%s-' % str(Dato1.strip()))
            # self.get_logger().info('%s-' % str(Dato2.strip()))
            # self.get_logger().info('%s-' % str(Dato3.strip()))
            string1_data = str(struct.unpack('f', Dato1.strip())[0])
            string2_data = str(struct.unpack('f', Dato2.strip())[0])
            string3_data = str(struct.unpack('f', Dato3.strip())[0])
            self.get_logger().info('Pepe')
        else:
            string1_data = str(int.from_bytes(Dato1.strip(), byteorder='big',signed=True)) #Ver de agregarle el signo
            string2_data = str(int.from_bytes(Dato2.strip(), byteorder='big',signed=True))
            string3_data = str(int.from_bytes(Dato3.strip(), byteorder='big',signed=True))
        
        file.write('%s' % str(Encabezado))
        file.write('%s/' % string1_data)
        file.write('%s/' % string2_data)
        file.write('%s\n' % string3_data)
        file.close()
        # msg.data = 'Hello World: %d' % 2
        # self.get_logger().info('%s/' % str(Dato1.strip()))
        # self.get_logger().info('%s/' % str(Dato2.strip()))
        # self.get_logger().info('%s\n' % str(Dato3.strip()))
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
