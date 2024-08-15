# AMR_Order_Pickup

# to set param
ros2 param set /order_optimizer_node directory_path /your/desired/path

# to run the Nodes
source ~/.bashrc
ros2 run amr_pkg orderoptimizer
ros2 run amr_pkg transformbroadcaster

with parameter:
ros2 run amr_pkg orderoptimizer --ros-args -p directory_path:=/home/sahil/Downloads/amr_example_ROS/applicants_amr_example_1


# to publish on the nextOrder Topic
ros2 topic pub -1 /nextOrder interfaces/msg/Order "{order_id: 1170558, description: 'This is a sample order description'}"

# to run RVIZ for visualization
ros2 run rviz2 rviz2
