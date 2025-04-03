#! /usr/bin/env python
# coding: utf-8

import math
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class PIDController(object):
    def __init__(self, p_gain, i_gain, d_gain):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain
        self._error_1 = 0.0
        self._error_2 = 0.0
        self._output = 0.0

    def update(self, current, target):
        error = target - current
        delta_output = self._p_gain * (error - self._error_1)
        delta_output += self._i_gain * (error)
        delta_output += self._d_gain * (error - 2 * self._error_1 + self._error_2)
        self._output += delta_output
        self._error_2 = self._error_1
        self._error_1 = error
        return self._output

class GripperClient(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/sciurus17/controller1/right_hand_controller/gripper_cmd", GripperCommandAction)
        self._goal = GripperCommandGoal()
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            exit(1)

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal, feedback_cb=self.feedback)

    def feedback(self, msg):
        rospy.loginfo("Feedback received: %s" % msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

def joint_state_callback(msg):
    global joint_state
    joint_state = msg

def main():
    global joint_state
    rospy.init_node("control_effort_gripper")
    gc = GripperClient()
    pid_controller = PIDController(0.5, 0.0, 3.0)
    r = rospy.Rate(60)
    target_angle = math.radians(30)
    while not rospy.is_shutdown():
        if len(joint_state.position) < 8:
            continue
        gripper_angle = joint_state.position[7]
        target_effort = pid_controller.update(gripper_angle, target_angle)
        gc.command(target_angle, target_effort)
        r.sleep()

if __name__ == '__main__':
    joint_state = JointState()
    sub_joint_state = rospy.Subscriber(
        "/sciurus17/controller1/joint_states", JointState, joint_state_callback, queue_size=1)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
