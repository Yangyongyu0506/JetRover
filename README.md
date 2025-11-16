# Development Log
## 2025.11.16
From now on, I will start using lifecycle publishers for all topics in the ESP_agent node, including the battery state topic. This change aims to enhance the robustness and reliability of the communication between the node and other components in the ROS2 ecosystem. By utilizing lifecycle publishers, we can better manage the state of our topics, ensuring that they are only active when the node is in an appropriate state. This will help prevent issues related to message delivery during state transitions and improve overall system stability.

We need to backup some bash commands for conveniently debugging lifecycle nodes:
```bash
colcon build --symlink-install --packages-select hardware_interface_py
source install/setup.bash
ros2 run hardware_interface_py ESP_agent
ros2 lifecycle get /ESP_agent_node
ros2 lifecycle set /ESP_agent_node configure
ros2 lifecycle set /ESP_agent_node activate
```
Note: You can't create normal subs in on_configure.