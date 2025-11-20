#!/usr/bin/env python3
import rospy
import smbus2
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

I2C_BUS = 1
MUX_ADDR = 0x70
IMU_ADDR = 0x68
MUX_CH_IMU = 0  # channel 0

class MPU6050Node:
    def __init__(self):
        rospy.init_node("mpu6050_node")
        self.bus = smbus2.SMBus(I2C_BUS)
        self.pub = rospy.Publisher("imu/data_raw", Imu, queue_size=10)

        # Select IMU channel
        self.select_mux_channel(MUX_CH_IMU)

        # Wake up MPU6050
        self.bus.write_byte_data(IMU_ADDR, 0x6B, 0x00)  # PWR_MGMT_1

    def select_mux_channel(self, ch):
        self.bus.write_byte(MUX_ADDR, 1 << ch)

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(IMU_ADDR, reg)
        low = self.bus.read_byte_data(IMU_ADDR, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.select_mux_channel(MUX_CH_IMU)

            # Read accelerometer
            acc_x = self.read_word_2c(0x3B) / 16384.0
            acc_y = self.read_word_2c(0x3D) / 16384.0
            acc_z = self.read_word_2c(0x3F) / 16384.0

            # Read gyro
            gyro_x = self.read_word_2c(0x43) / 131.0 * math.pi / 180.0
            gyro_y = self.read_word_2c(0x45) / 131.0 * math.pi / 180.0
            gyro_z = self.read_word_2c(0x47) / 131.0 * math.pi / 180.0

            msg = Imu()
            msg.header = Header(stamp=rospy.Time.now(), frame_id="imu_link")
            msg.linear_acceleration.x = acc_x * 9.81
            msg.linear_acceleration.y = acc_y * 9.81
            msg.linear_acceleration.z = acc_z * 9.81
            msg.angular_velocity.x = gyro_x
            msg.angular_velocity.y = gyro_y
            msg.angular_velocity.z = gyro_z
            # orientation left zero or fused with a filter elsewhere

            self.pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    MPU6050Node().spin()
