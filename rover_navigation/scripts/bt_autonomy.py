#!/usr/bin/env python3
import rospy
import py_trees
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class MoveToPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, pose):
        super(MoveToPose, self).__init__(name)
        self.pose = pose
        self.client = None
        self.sent = False

    def setup(self, timeout):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server(rospy.Duration(10.0))
        return True

    def initialise(self):
        self.sent = False

    def update(self):
        if not self.sent:
            goal = MoveBaseGoal()
            goal.target_pose = self.pose
            self.client.send_goal(goal)
            self.sent = True
            return py_trees.common.Status.RUNNING

        state = self.client.get_state()
        if state == 3:  # SUCCEEDED
            return py_trees.common.Status.SUCCESS
        elif state in [4,5,8]:  # ABORTED, REJECTED, CANCELED
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

def make_pose(x, y, yaw, frame="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = 0.0  # TODO: set from yaw if needed
    pose.pose.orientation.w = 1.0
    return pose

def create_tree():
    root = py_trees.composites.Sequence("Mission")

    wp1 = MoveToPose("GoToWP1", make_pose(1.0, 0.0, 0.0))
    wp2 = MoveToPose("GoToWP2", make_pose(2.0, 1.0, 0.0))

    root.add_children([wp1, wp2])
    return root

if __name__ == "__main__":
    rospy.init_node("bt_autonomy")
    root = create_tree()
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.setup(timeout=15)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        behaviour_tree.tick()
        rate.sleep()
