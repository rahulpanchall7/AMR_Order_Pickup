# AMR_Order_Pickup
# Steps to run the AMR Order Optimizer Code:

1. Clone this repository, and build the workspace with:
   $colcon build

2. Source the setup files:
   $source /opt/ros/humble/setup.bash
   $source ~/{your_workspace}/install/setup.bash

3. Run the ransformbroadcaster node, for static tftree transform:
   $ros2 run amr_pkg transformbroadcaster
   
5. Open RVIZ:
   $ros2 run rviz2 rviz2

6. Run the orderoptimizer node with the directory_path parameter:
   $ros2 run amr_pkg orderoptimizer --ros-args -p directory_path:=path/to/your/directory
   Eg: "/home/rahul/Downloads/amr_example_ROS/applicants_amr_example_1" , where it contains orders and configuration

7. Publish on topic '/currentPosition' only once at begining before the first order to initialise the AMR's positon, as later on it uses a global
   variable to update the position:
   $ros2 topic pub /currentPosition geometry_msgs/msg/PoseStamped "{
     header: {
       stamp: { sec: 0, nanosec: 0 },
       frame_id: 'map'
     },
     pose: {
       position: { x: {init_x}, y: {init_y}, z: 0.0 },
       orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
     }
   }"

9. Publish on topic '/nextOrder':
   $ros2 topic pub -1 /nextOrder interfaces/msg/Order "{order_id: {Order id}, description: {' your description '}"
   Eg: ros2 topic pub -1 /nextOrder interfaces/msg/Order "{order_id: 1300020, description: 'This is a sample order description'}"










