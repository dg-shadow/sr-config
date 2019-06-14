#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RePubTrajectory(object):
    def __init__(self):
        self._bag_tf_sub = rospy.Subscriber("/rh_trajectory_controller/command_remapped", JointTrajectory, self._bag_traj_cb, tcp_nodelay=True)
        self._traj_pub = rospy.Publisher("/rh_trajectory_controller/command", JointTrajectory, queue_size=10)
        self._joints_to_move = ["rh_FFJ1", "rh_FFJ2"] #, "rh_FFJ3"]

    def _bag_traj_cb(self, data):
        new_traj = JointTrajectory()
        point = JointTrajectoryPoint()
        new_traj.header = data.header
        for index, joint_name in enumerate(data.joint_names):
            if joint_name in self._joints_to_move:
                new_traj.joint_names.append(joint_name)
                point.positions.append(data.points[0].positions[index])
        point.time_from_start.nsecs = 55000000
        new_traj.points = [point]
        new_traj.header.stamp = rospy.Time.now()
        self._traj_pub.publish(new_traj)


if __name__ == "__main__":
    rospy.init_node("republish_trajectory")
    pub_traj = RePubTrajectory()
    rospy.spin()
