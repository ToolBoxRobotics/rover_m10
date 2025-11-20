#!/usr/bin/env python3
import rospy
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmIK:
  def __init__(self):
    rospy.init_node("arm_ik_node")

    robot = URDF.from_parameter_server()
    base_link = rospy.get_param("~base_link", "arm_base_link")
    tip_link  = rospy.get_param("~tip_link",  "arm_tool_link")

    tree = kdl.Tree()
    kdl_tree = kdl.Tree()
    # Use kdl_parser_py if available:
    from kdl_parser_py.urdf import treeFromUrdfModel
    ok, kdl_tree = treeFromUrdfModel(robot)
    if not ok:
      rospy.logerr("Failed to build KDL tree")
      return

    self.chain = kdl_tree.getChain(base_link, tip_link)
    self.nr_joints = self.chain.getNrOfJoints()

    self.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]

    self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)

    self.joint_state_sub = rospy.Subscriber("arm/joint_states", JointState, self.js_cb)
    self.target_sub      = rospy.Subscriber("arm/target_pose", PoseStamped, self.target_cb)
    self.traj_pub        = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)

    self.current_js = None

  def js_cb(self, msg):
    self.current_js = msg

  def target_cb(self, pose_msg):
    if self.current_js is None:
      rospy.logwarn("No joint states yet")
      return

    # Seed from current joint positions
    q_init = kdl.JntArray(self.nr_joints)
    for i, name in enumerate(self.joint_names):
      try:
        idx = self.current_js.name.index(name)
        q_init[i] = self.current_js.position[idx]
      except ValueError:
        q_init[i] = 0.0

    # Convert pose
    p = pose_msg.pose.position
    o = pose_msg.pose.orientation
    frame = kdl.Frame(
      kdl.Rotation.Quaternion(o.x, o.y, o.z, o.w),
      kdl.Vector(p.x, p.y, p.z)
    )

    q_out = kdl.JntArray(self.nr_joints)
    ret = self.ik_solver.CartToJnt(q_init, frame, q_out)

    if ret >= 0:
      jt = JointTrajectory()
      jt.joint_names = self.joint_names
      pt = JointTrajectoryPoint()
      pt.positions = [q_out[i] for i in range(self.nr_joints)]
      pt.time_from_start = rospy.Duration(2.0)
      jt.points.append(pt)
      self.traj_pub.publish(jt)
    else:
      rospy.logwarn("IK failed with code %d", ret)

if __name__ == "__main__":
  ArmIK()
  rospy.spin()
