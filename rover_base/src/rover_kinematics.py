#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class RoverKinematics:
    def __init__(self):
        rospy.init_node("rover_kinematics")

        # Parameters (meters)
        self.wheelbase = rospy.get_param("~wheelbase", 0.5)   # front–rear distance
        self.track    = rospy.get_param("~track", 0.4)        # left–right distance
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 35.0)
        self.max_speed = rospy.get_param("~max_speed", 1.0)   # m/s

        self.pub_speed = rospy.Publisher("wheel_speed_cmd", Float32MultiArray, queue_size=1)
        self.pub_steer = rospy.Publisher("steering_cmd", Float32MultiArray, queue_size=1)

        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def cmd_cb(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        # Normalize speed to [-1,1]
        speed_norm = self.clamp(vx / self.max_speed, -1.0, 1.0)

        # Steering angle (simple proportional relationship)
        # For small turns, approximate steering angle in radians
        steer_rad = 0.0
        if abs(wz) > 1e-3 and abs(vx) > 1e-3:
            radius = vx / wz
            steer_rad = self.clamp(
                self.track / (2.0 * radius),
                -self.max_steer_deg * 3.14159 / 180.0,
                self.max_steer_deg * 3.14159 / 180.0
            )
        steer_deg = steer_rad * 180.0 / 3.14159
        steer_deg = self.clamp(steer_deg, -self.max_steer_deg, self.max_steer_deg)

        # All 6 wheels same speed for now, sign determined by vx
        wheel_speeds = [speed_norm] * 6

        # Front left, front right, rear left, rear right
        # You can add inner/outer differential angles if desired
        steering_angles = [steer_deg, steer_deg, -steer_deg, -steer_deg]

        speed_msg = Float32MultiArray(data=wheel_speeds)
        steer_msg = Float32MultiArray(data=steering_angles)

        self.pub_speed.publish(speed_msg)
        self.pub_steer.publish(steer_msg)

if __name__ == "__main__":
    RoverKinematics()
    rospy.spin()
