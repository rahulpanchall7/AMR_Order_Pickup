<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>AMR Order Pickup - Setup Guide</title>
</head>
<body>
  <h1>AMR_Order_Pickup</h1>
  <h2>Steps to run the AMR Order Optimizer Code:</h2>

  <ol>
    <li>Clone this repository, and build the workspace:
      <pre><code>colcon build</code></pre>
    </li>

    <li>Source the setup files:
      <pre><code>
source /opt/ros/humble/setup.bash
source ~/{your_workspace}/install/setup.bash
      </code></pre>
    </li>

    <li>Run the transformbroadcaster node for static TF tree transform:
      <pre><code>ros2 run amr_pkg transformbroadcaster</code></pre>
    </li>

    <li>Open RVIZ:
      <pre><code>ros2 run rviz2 rviz2</code></pre>
    </li>

    <li>Run the orderoptimizer node with the <code>directory_path</code> parameter:
      <pre><code>ros2 run amr_pkg orderoptimizer --ros-args -p directory_path:=path/to/your/directory</code></pre>
      <p>Example:</p>
      <pre><code>/home/rahul/Downloads/amr_example_ROS/applicants_amr_example_1</code></pre>
      <p>This directory should contain orders and configuration files.</p>
    </li>

    <li>Publish on topic <code>/currentPosition</code> once at the beginning to initialize the AMR's position:
      <pre><code>
ros2 topic pub -1 /currentPosition geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: 'map'
  },
  pose: {
    position: { x: 25.0, y: 300.0, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}"
      </code></pre>
    </li>
