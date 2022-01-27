## OPEN

```bash
rostopic pub --once /panda_gripper_trajectory_controller/command trajectory_msgs/JointTrajectory  "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'panda_finger_joint1'
- 'panda_finger_joint2'
points:
- positions: [0.04, 0.04]
  velocities: [0,0]
  accelerations: [0,0]
  effort: [0,0]
  time_from_start: {secs: 1, nsecs: 0}"
```

## CLOSE

```bash
rostopic pub --once /panda_gripper_trajectory_controller/command trajectory_msgs/JointTrajectory  "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'panda_finger_joint1'
- 'panda_finger_joint2'
points:
- positions: [0.0, 0.0]
  velocities: [0,0]
  accelerations: [0,0]
  effort: [0,0]
  time_from_start: {secs: 1, nsecs: 0}"
```