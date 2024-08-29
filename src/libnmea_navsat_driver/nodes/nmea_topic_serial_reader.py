# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial

from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message as rtcm_msgs_RTCM
import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver


gps_port = None

def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()

    nmea_pub = driver.create_publisher(Sentence, "nmea_sentence", 10)
#    rtcm_sub = driver.create_subscription(rtcm_msgs_RTCM, "rtcm", subscribe_rtcm,10)

    serial_port = driver.declare_parameter('port', '/dev/ttyUSB0').value
    serial_baud = driver.declare_parameter('baud', 4800).value

    # Get the frame_id
    frame_id = driver.get_frame_id()

    try:
        gps_port = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        try:
            while rclpy.ok():
                data = gps_port.readline().strip()

                sentence = Sentence()
                sentence.header.stamp = driver.get_clock().now().to_msg()
                sentence.header.frame_id = frame_id
                sentence.sentence = data.decode("ascii")
                nmea_pub.publish(sentence)
                driver.add_sentence(sentence.sentence,frame_id)

        except Exception as e:
            driver.get_logger().error("Ros error: {0}".format(e))
            gps_port.close()  # Close GPS serial port
    except serial.SerialException as ex:
        driver.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))


def subscribe_rtcm(rtcm):

    try:
        if gps_port is not None:
        # print(rtcm.data)
            try:
                gps_port.write(rtcm.message)
                print("write rtcm")
            except Exception as e:
                print("Ros error: {0}".format(e))
                gps_port.close()  # Close GPS serial port
    
    except serial.SerialException as ex:
        print("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
