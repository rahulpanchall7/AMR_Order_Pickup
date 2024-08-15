# AMR_Order_Pickup


# to run the Nodes
source ~/.bashrc
ros2 run amr_pkg orderoptimizer
ros2 run amr_pkg transformbroadcaster

# to publish on the nextOrder Topic
ros2 topic pub -1 /nextOrder interfaces/msg/Order "{order_id: 1170558, description: 'This is a sample order description'}"

# to run RVIZ for visualization
ros2 run rviz2 rviz2
