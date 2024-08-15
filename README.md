# AMR_Order_Pickup
# Steps to run the AMR Order Optimizer Code:

1. Clone this repository, and build the workspace with:
   $colcon build

2. Source the setup files:
   $source /opt/ros/humble/setup.bash
   $source ~/{your_workspace}/install/setup.bash

3. Run the orderoptimizer node with the directory_path parameter, and the transformbroadcaster node:
   $ros2 run amr_pkg orderoptimizer --ros-args -p directory_path:=path/to/your/directory
   $ros2 run amr_pkg transformbroadcaster
   Eg: "/home/rahul/Downloads/amr_example_ROS/applicants_amr_example_1" , where it contains orders and configuration

5. Visualize in RVIZ:
   $ros2 run rviz2 rviz2 

6. Publish on topic '/nextOrder':
   $ros2 topic pub -1 /nextOrder interfaces/msg/Order "{order_id: {Order id}, description: {' your description '}"
   Eg: ros2 topic pub -1 /nextOrder interfaces/msg/Order "{order_id: 1300020, description: 'This is a sample order description'}"





