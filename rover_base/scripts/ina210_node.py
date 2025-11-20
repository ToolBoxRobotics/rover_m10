#!/usr/bin/env python3
import rospy
import smbus2
from std_msgs.msg import Float32MultiArray

I2C_BUS = 1
MUX_ADDR = 0x70
INA_ADDR = 0x40
MUX_CH_INA = 1

class INA219Node:
    def __init__(self):
        rospy.init_node("ina219_node")
        self.bus = smbus2.SMBus(I2C_BUS)
        self.pub = rospy.Publisher("power", Float32MultiArray, queue_size=10)

        self.select_mux_channel(MUX_CH_INA)

        # Basic configuration (reset + default)
        self.bus.write_word_data(INA_ADDR, 0x00, 0x399F)  # example config
        # Calibration registers depend on shunt resistor; adjust.

    def select_mux_channel(self, ch):
        self.bus.write_byte(MUX_ADDR, 1 << ch)

    def read_word(self, reg):
        raw = self.bus.read_word_data(INA_ADDR, reg)
        # INA stores data big-endian, swap bytes if needed
        raw_swapped = ((raw & 0xFF) << 8) | (raw >> 8)
        return raw_swapped

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.select_mux_channel(MUX_CH_INA)

            bus_voltage_raw = (self.read_word(0x02) >> 3) * 4  # mV
            current_raw = self.read_word(0x04)                 # depends on calibration

            bus_voltage = bus_voltage_raw / 1000.0   # V
            current = current_raw * 0.001            # A (placeholder scale)

            msg = Float32MultiArray(data=[bus_voltage, current])
            self.pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    INA219Node().spin()
