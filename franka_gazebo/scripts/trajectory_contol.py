"""Trajectory control example script."""

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import time

if __name__ == "__main__":

    rospy.init_node("trajectory_publisher", anonymous=True)
    pub = rospy.Publisher(
        "/panda_gripper_trajectory_controller/command", JointTrajectory, queue_size=10
    )
    time.sleep(2)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        # Create message
        req = JointTrajectory()
        req.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [0.04, 0.04]
        point.effort = [0.0, 0.0]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start = rospy.Duration(secs=1)
        req.points.append(point)

        # Close
        pub.publish(req)
        time.sleep(2)

        # open
        req = JointTrajectory()
        req.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.effort = [0.0, 0.0]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start = rospy.Duration(secs=1)
        req.points.append(point)
        pub.publish(req)
        time.sleep(2)

        # print("done")

    # req.points.append()
    # pub.publish(hello_str)
