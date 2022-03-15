# Simulation of the controllers of TU Delft in Gazebo

How to start the simulation of the cartesian impedance control


roslaunch franka_gazebo panda.launch x:=-0.5     world:=$(rospack find franka_gazebo)/world/test2     controller:=cartesian_impedance_example_controller     rviz:=true







See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.




## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
