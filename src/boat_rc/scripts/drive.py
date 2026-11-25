#!/usr/bin/env python3
"""RadioMaster RP3 CRSF channel receiver on Pi 4."""
import serial
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, UInt8


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 128 else 0)) & 255
    return crc


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def clear(self):
        self.buffer.clear()

    def feed(self, data):
        self.buffer.extend(data)
        events = []
        while len(self.buffer) >= 2:
            if self.buffer[0] not in (0xC8, 0xEC, 0xEE, 0x00) or not 2 <= self.buffer[1] <= 62:
                del self.buffer[0]
                continue
            size = self.buffer[1] + 2
            if len(self.buffer) < size:
                break
            body = bytes(self.buffer[2:size-1])
            if crc8(body) != self.buffer[size-1]:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            kind, payload = body[0], body[1:]
            # Fail closed on unsupported channel-frame variants.
            if kind == 0x16 and len(payload) == 22:
                packed = int.from_bytes(payload, 'little')
                events.append(('channels', [(packed >> (11*i)) & 2047 for i in range(16)]))
            elif kind == 0x14 and len(payload) >= 10:
                events.append(('lq', payload[2]))
        return events


class Drive(Node):
    def __init__(self):
        super().__init__('boat_rc')
        param = lambda key, default: self.declare_parameter(key, default).value
        self.parser = Parser()
        self.last_poll = time.monotonic()
        self.uart = serial.Serial(param('port', '/dev/ttyAMA1'), param('baud', 420000),
                                  timeout=0, exclusive=True)
        self.uart.reset_input_buffer()
        self.channel_pub = self.create_publisher(Int32MultiArray, '/rc_channels', 1)
        self.lq_pub = self.create_publisher(UInt8, '/rc_uplink_lq', 1)
        self.create_timer(0.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        now = time.monotonic()
        if now - self.last_poll >= 0.1:
            self.uart.reset_input_buffer()
            self.parser = Parser()
        self.last_poll = now
        for kind, value in self.parser.feed(self.uart.read(4096)):
            if kind == 'channels':
                self.channel_pub.publish(Int32MultiArray(data=value))
            else:
                self.lq_pub.publish(UInt8(data=value))


def main():
    rclpy.init()
    node = Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.uart.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
