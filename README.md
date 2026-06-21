[//]: # (Image References)

[image1]: ./assets/skid1.png "Starter package"
[image2]: ./assets/skid2.png "Adding a camera"
[image3]: ./assets/skid3.png "Adding a camera"
[image4]: ./assets/odom1.jpg "Adding a camera"
[image5]: ./assets/camera1.png "Adding a camera"
[image6]: ./assets/cam_rqr.png "rqt reconfigure"
[image7]: ./assets/compressed_rqt.png "Wide angle camera"
[image8]: ./assets/relay_node_compressed.png "Wide angle camera"
[image9]: ./assets/rqt_reconfigure.png "IMU"
[image10]: ./assets/rgbd1.png "TF Tree"
[image11]: ./assets/rgbd2.png "Odometry"
[image12]: ./assets/depth-image.png "Navsat"
[image13]: ./assets/lidar1.png "New York - Madrid"
[image14]: ./assets/visualize_lidar.png "RViz GPS"
[image15]: ./assets/lidar2.png "RViz GPS"
[image16]: ./assets/3d_lidar.png "Lidar"
[image17]: ./assets/3d_mapping.png "Lidar"
[image18]: ./assets/mapping_rgbd.png "Lidar"
[image19]: ./assets/ekf1.png "Lidar"
[image20]: ./assets/imu1.png "Lidar"
[image21]: ./assets/opencv1.png "RGBD Camera"
[image22]: ./assets/opencv2.png "RGBD Camera"
[image23]: ./assets/opencv3.png "RGBD Camera"
[image24]: ./assets/opencv4.png "Depth image"
[image25]: ./assets/yolo1.png "OpenCV"
[image26]: ./assets/yolo2.png "Red ball in Gazebo"

# Table of Contents
1. [Introduction](#introduction)  
1.1. [Download ROS package](#download-ros-package)  
1.2. [Test the starter package](#test-the-starter-package)
2. [Skid-Steer](#skid-steer)
3. [Friction](#friction)
4. [Odometry](#odometry)
5. [Sensors](#sensors)
6. [Camera](#camera)  
2.1. [Image transport](#image-transport)  
2.2. [rqt reconfigure](#rqt-reconfigure)
2.3. [RGBD camera](#rgbd-camera)  
7. [Lidar](#lidar)
3.1. [2D lidar](#2d-lidar)
3.1. [3D lidar](#3d-lidar) 
8. [IMU](#imu)  
4.1. [Sensor fusion with ekf](#sensor-fusion-with-ekf)
5. [Perception](#perception)
9. [OpenCV](#opencv)
10. [Yolov8](#yolov8)

# Introduction

# Skid-Steer

You're given with the caster wheel bot 

![alt text][image1]

So to convert it to skid steer 4-wheeled Diff-drive bot we need to give it 4 wheels, 2 on left and 2 on right. Currently we have 2 wheels attached but they are very near to centre as in caster wheel to give the base stability, but now we have four so we will keep them little away from centre.

So lets replace caster wheel with the front left wheel first

Delete this section in erc_bot.urdf to remove the caster wheel from robot model 

```xml
   <joint type="fixed" name="front_caster_wheel_joint">
    <origin xyz="0.11 0 -0.05" rpy="0 0 0"/>
    <child link="front_caster_wheel"/>
    <parent link="base_link"/>
   </joint>

   <link name='front_caster_wheel'>
     <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="0.002" ixy="0" ixz="0"
          iyy="0.002" iyz="0"
          izz="0.002"
      />
    </inertial>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <visual name='front_caster_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
        <material name="white"/>
    </visual>
  </link>
```

Lets add front left wheel first

Attach the Wheel to robot's base
```xml
  <joint type="continuous" name="front_left_wheel_joint">
    <origin xyz="0.15 0.15 0" rpy="0 0 0"/>
    <child link="front_left_wheel"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="100" velocity="10"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>
```

Define mass and inertial properties
```xml
  <link name='front_left_wheel'>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <inertia
          ixx="0.014" ixy="0" ixz="0"
          iyy="0.014" iyz="0"
          izz="0.025"
      />
    </inertial>
```

define collision model
```xml
    <collision>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/> 
      <geometry>
        <cylinder radius=".1" length=".05"/>
      </geometry>
    </collision>
```
define visual mode l
```xml
   <visual name='front_left_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder radius=".1" length=".05"/>
      </geometry>
       <material name="green"/>
    </visual>
  </link>
```
Just do the same for right wheel but dont forget to change the origin for right wheel(HINT:It should be opposite to front left wheel, so you need to change one coordinate of right wheel in origin tags)

Let's test the model in Rviz
```bash
ros2 launch erc_gazebo_sensors check_urdf.launch.py
```

![alt text][image2]

Here u can see in Rviz the rear wheels are closer to the base centre, so just mirror the front wheels for both rear wheels

Also add the joints inside Diff-drive plugin in erc_bot.gazebo such it looks:
```xml
<?xml version="1.0"?>
<robot>
  <gazebo>
    <plugin
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
        <!-- Topic for the command input -->
        <topic>/cmd_vel</topic>

        <!-- Wheel joints -->
        <left_joint>rear_left_wheel_joint</left_joint>
        <right_joint>rear_right_wheel_joint</right_joint>
        <left_joint>front_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>

        <!-- Wheel parameters -->
        <wheel_separation>0.3</wheel_separation>
        <wheel_radius>0.1</wheel_radius> 

        <!-- Control gains and limits (optional) -->
        <max_velocity>3.0</max_velocity> 
        <max_linear_acceleration>1</max_linear_acceleration>
        <min_linear_acceleration>-1</min_linear_acceleration>
        <max_angular_acceleration>2</max_angular_acceleration>
        <min_angular_acceleration>-2</min_angular_acceleration>
        <max_linear_velocity>0.5</max_linear_velocity>
        <min_linear_velocity>-0.5</min_linear_velocity>
        <max_angular_velocity>1</max_angular_velocity>
        <min_angular_velocity>-1</min_angular_velocity>
        
        <!-- Other parameters (optional) -->
        <odom_topic>odom</odom_topic> 
        <tf_topic>tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_footprint</child_frame_id>
        <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>

    <plugin
        filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
        <joint_name>rear_left_wheel_joint</joint_name>
        <joint_name>rear_right_wheel_joint</joint_name>
        <joint_name>front_left_wheel_joint</joint_name>
        <joint_name>front_right_wheel_joint</joint_name>

    </plugin>
  </gazebo>
  
</robot>
```

Then build your workspace and source it 

Then launch our robot to gazebo arena

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image3]

And move it using this teleop node

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can also test your bot motion directly from the gazebo teleop GUI --> goto the three dotted menu --> type teleop --> specify wheels speed and yaw then move around 

FRICTION

For all 4-wheels
```xml
  <gazebo reference="front_left_wheel">
    <mu1>1.5</mu1>
    <mu2>0.7</mu2>
    <kp>200000.0</kp>
    <kd>5000.0</kd>
    <minDepth>0.002</minDepth>
    <maxVel>0.3</maxVel>
    <fdir1>0 1 0</fdir1>
  </gazebo>
```
For base link
```xml
    <gazebo reference="base_link">
    <mu1>0.000002</mu1>
    <mu2>0.000002</mu2>
  </gazebo>
```
ODOMETRY

Lets view our robots trajectory

Check the trajcetory package inside the workspace

In terminal 1
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

In terminal 2
```bash
ros2 run trajectory_server trajectory_server
```

Now move around using the keyboard teleop node or Gazebo teleop GUI and visualize the robot's path in green coloured trajectory

![alt text][image4]

# SENSORS

# Camera

To add a camera - and every other sensors later - we have to change 2 files:
1) `erc_bot.urdf`: here define the position, orientation and other physical properties of the camera in this file. 
2) The `erc_bot.gazebo`: Here we add real camera sensor properties like resolution, field of view, noise,etc. to our simulated camera 

Let's add the camera first to the `erc_bot.urdf`:

Attach the camera to front of robot's base 
```xml
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.225 0 0.075" rpy="0 0 0"/>
    <child link="camera_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" />
  </joint>
```
Define mass and inertial properties
```xml
  <link name='camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>
```

Add collision model for physical interactions(Here camera model is definded as a 3cm cube box)
```xml
    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".03 .03 .03"/>
      </geometry>
    </collision>
```

Add visual model to give visual features of 3cm cube box to our simulated camera
```xml
    <visual name='camera_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".03 .03 .03"/>
      </geometry>
    </visual>
  </link>
```

Add gazebo colour to our camera
```xml
  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>
```

Here we need to define one more link `camera_link_optical` because normal ros robot coordinate frames are different from what computer vision algorithms expects, so our camera link species where its mounted on the robot body and camera_link_optical tells how the camera sees the world
```xml
  <joint type="fixed" name="camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="camera_link_optical"/>
    <parent link="camera_link"/>
  </joint>

  <link name="camera_link_optical">
  </link>
```
Now let's add the simulated camera properties into `erc_bot.gazebo`:
```xml
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>15</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
        <optical_frame_id>camera_link_optical</optical_frame_id>
        <camera_info_topic>camera/camera_info</camera_info_topic>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>camera/image</topic>
    </sensor>
  </gazebo>
```
Aslo remember to keep this code snippet between the `<robot>` tags!

With the above plugin we define a couple of things for Gazebo, let's see the important ones one by one:
- `<gazebo reference="camera_link">`, we have to refer to the `camera_link` that we defined in the `urdf`
- `<horizontal_fov>1.3962634</horizontal_fov>`, the field of view of the simulated camera
- `width`, `height`, `format` and `update_rate`, properties of the video stream
- `<optical_frame_id>camera_link_optical</optical_frame_id>`, we have to use the `camera_link_optical` that we checked in details above to ensure the right static transformations between the coordinate systems
- `<camera_info_topic>camera/camera_info</camera_info_topic>`, certain tools like rviz requires a `camera_info` topic that describes the physical properties of the camera. The topic's name must match camera's topic (in this case both are `camera/...`)
- `<topic>camera/image</topic>`, we define the camera topic here

We can now see the camera model and two different tf frames on it but yet we can't see the image stream, because gazebo is publishing them but ros cant receive since both are different softwares, so we have to bridge them.

For this we have gz_brige_node in spawn_robot.launch.py file. You can check here about the kind of topic types forwaded between ROS and Gazebo.

Lets extend the arguments in gz_bridge_node to forward two more image topics

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```
Lets rebuild the workspace and launch the robot
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

Now you can see the image stream 

![alt text][image5]

If you cant see camera window at right bottom of Rviz, then goto Add --> by topic --> image

You can also see the image renderings using `rqt` tool of ROS

![alt text][image6]

Just type `rqt` in another terminal, then goto plugins --> visualization --> image view 

Refresh if you cant see 

IMAGE TRANSPORT

We can see that both `/camera/camera_info` and `/camera/image` topics are forwarded. Although this is still not the ideal way to forward the camera image from Gazebo. ROS has a very handy feature with it's image transport protocol plugins, it's able to automatically compress the video stream in the background without any additional work on our side. But this feature doesn't work together with `parameter_bridge`. Without compression the 640x480 camera stream consumes almost 20 MB/s network bandwidth which is unacceptable for a wireless mobile robot.

Therefore there is a dedicated `image_bridge` node in the `ros_gz_image` package. Let's modify our launch file:

Comment the camera_image topic from gz_bridge_node
because we'll use a separate node to forward the compressed image only

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )
```

Dont forget to add the new node to the `launchDescription`:

```python
launchDescriptionObject.add_action(gz_image_bridge_node)
```

After rebuild we can try it using `rqt` and we will see huge improvement in the bandwith due to the `jpeg` compression.

![alt text][image7]

> If compressed images are not visible in rqt, you have to install the plugins you want to use:
> - `sudo apt install ros-jazzy-compressed-image-transport`: for jpeg and png compression
> - `sudo apt install ros-jazzy-theora-image-transport`: for theora compression
> - `sudo apt install ros-jazzy-zstd-image-transport`: for zstd compression

But we face another issue, this time in RViz, the uncompressed camera stream is visible as before but the compressed one isn't due to the following warning:

```
Camera Info
Expecting Camera Info on topic [/camera/image/camera_info]. No CameraInfo received. Topic may not exist.
```

It's because RViz always expect the `image` and `the camera_info` topics with the same prefix which works well for:

`/camera/image` &#8594; `/camera/camera_info`

But doesn't work for:

`/camera/image/compressed` &#8594; `/camera/image/camera_info`

Because we don't publish the `camera_info` to that topic. We could remap the `camera_info` to that topic, but then the uncompressed image won't work in RViz, so it's not the desired solution.

But there is another useful tool that we can use, the `relay` node from the `topic_tools` package:

```python
    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Of course, don't forget to add it to the `launchDescription` too:
```python
launchDescriptionObject.add_action(relay_camera_info_node)
```

> If `topic_tools` is not installed you can install it with `sudo apt install ros-jazzy-topic-tools`

Rebuild the workspace and let's try it!

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```
![alt text][image8]

RQT_RECONFIGURE

We already set up the `jpeg` quality in the `image_bridge` node with the following parameter:
```python
'camera.image.compressed.jpeg_quality': 75
```

But how do we know what is the name of the parameter and what other settings do we can change? To see that we will use the `rqt_reconfigure` node.

First start the simulation:

Terminal1
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

Then start rqt_reconfigure:

Terminal 2
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

![alt text][image9]

We can play with the parameters here, change the compression or the algorithm as we wish and we can monitor it's impact with `rqt` on real-time.

RGBD-CAMERA

To add an RGBD camera let's replace the conventional camera properties in `erc_bot.gazebo` with this one:
```xml
  <gazebo reference="camera_link">
    <sensor name="rgbd_camera" type="rgbd_camera">
      <camera>
        <horizontal_fov>1.25</horizontal_fov>
        <image>
          <width>320</width>
          <height>240</height>
        </image>
        <clip>
          <near>0.3</near>
          <far>15</far>
        </clip>
        <optical_frame_id>camera_link_optical</optical_frame_id>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>camera</topic>
      <gz_frame_id>camera_link</gz_frame_id>
    </sensor>
  </gazebo>
```
And let's forward 2 topics with the parameter_bridge:

- /camera/depth_image which provides a grayscale camera stream where the grayscale values correspond to the distance of the individual pixels. RViz is able to render depth image topic and the color image topic together as a depth cloud.

![alt text][image12]

- /camera/points which is a 3D pointcloud, the same type as the 3D lidar's point cloud. We can visualize it in RViz just as any point clouds.

Let's add the topics to the parameter bridge:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Rebuild the workspace and let's start the simulation:

```bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

![alt text][image10]

But we encounter a issue here as the orientation of 3D point cloud isn't correct because it's interpreted in the camera_link_optical frame, let's change it with `       <optical_frame_id>camera_link</optical_frame_id>`

![alt text][image11]

Here we can get a basic mapping of the environment by increasing the decay time under 
`Pointcloud2 camera` option, set it to around 30 seconds and move around to build the map

![alt text][image18]

#Lidar

Similar to the camera we create Lidar and add real sensor properties to it

# 2D lidar

First, we'll start with a simple 2D lidar, let's add it to the `erc_bot.urdf`:

```xml
  <joint type="fixed" name="scan_joint">
    <origin xyz="0.0 0 0.15" rpy="0 0 0"/>
    <child link="scan_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
  </joint>

  <link name='scan_link'>
    <inertial>
      <mass value="1e-5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>
    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".06 .06 .06"/>
      </geometry>
    </collision>

    <visual name='scan_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
       <box size=".06 .06 .06"/>
      </geometry>
    </visual>
  </link>
```

Then add the plugin to the `erc_bot.gazebo` file:

```xml
  <gazebo reference="scan_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <gz_frame_id>scan_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>720</samples>
            <!--(max_angle-min_angle)/samples * resolution -->
            <resolution>1</resolution>
            <min_angle>-3.14156</min_angle>
            <max_angle>3.14156</max_angle>
          </horizontal>
          <!-- Dirty hack for fake lidar detections with ogre 1 rendering in VM -->
        </scan>
        <range>
          <min>0.05</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
        </noise>
        <frame_id>scan_link</frame_id>
      </lidar>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
```

Before we can test our lidar we have to update the `parameter_bridge` to forward the lidar scan topic from gazebo to ROS:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Let's try it in the simulation!

```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image13]

We can see the red laserscan points in Rviz

Also verify the rendering of lidars in Gazebo, as we set it to `true` in the gazebo plugin file, with the `Visualize Lidar` tool:

![alt text][image14]

Here also we can get a simple mapping of the surroundings by increasing the decay time 

![alt text][image15]

# 3D lidar

If we want to simulate a 3D lidar we only have to add vertical samples together with the minimum and maximum angles.

So, let's modify the  erc_bot.gazebo to add vertical samples as well

Add this block after the horizontal scan parameters:
```xml
          <vertical>
              <samples>32</samples>
              <min_angle>-0.5353</min_angle>
              <max_angle>0.1862</max_angle>
          </vertical>
```

To properly visualize a 3D point cloud in RViz we have to forward one more topic with parameter_bridge:

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

Launch the bot again:
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

![alt text][image16]

We can get 3D mapping of the surrounding here by increasing the decay time similarly we did for the RGBD-Camera and 2D lidar

![alt text][image17]

IMU

An Inertial Measurement Unit (IMU) typically consists of a 3-axis accelerometer, 3-axis gyroscope, and sometimes a 3-axis magnetometer. It measures linear acceleration, angular velocity, and possibly magnetic heading (orientation)

But first, let's add our IMU to the `urdf`:

  <joint name="imu_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="base_link"/>
    <child link="imu_link" />
  </joint>

  <link name="imu_link">
  </link>

Which is a simple link and a fixed joint in the center of the base link. Let's add the plugin to the `erc_bot.gazebo` file too:

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>1</always_on>
      <update_rate>50</update_rate>
      <visualize>true</visualize>
      <topic>imu</topic>
      <enable_metrics>true</enable_metrics>
      <gz_frame_id>imu_link</gz_frame_id>
    </sensor>
  </gazebo>

With adding the IMU we aren't done yet, with the new Gazebo we also have to make sure that our simulated world has the right plugins within its `<world>` tag

So add this IMU plugin in the `home.sdf` world
```xml
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
```
Before we try it out we also have to bridge the topics from Gazebo towards ROS using the parameter_bridge. Let's add the `imu` topic - or what we defined as `<topic>` in the Gazebo plugin - to the parameter bridge, rebuild the workspace and we are ready to test it!

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

First,  launch the bot:

Terminal 1
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

Terminal 2

Launch the bot again:
```bash
rqt
```

Then goto Plugins --> Topics --> Topic Monitor 

Check on IMU and move the bot to see the changing IMU data of angular velocity, linear acceleration and orientation.

![alt text][image20]

SENSOR FUSION 

Sensor fusion is the process of combining data from multiple sensors (possibly of different types) to obtain a more accurate or more complete understanding of a system or environment than could be achieved by using the sensors separately.

A Kalman Filter (KF) is a mathematical algorithm that estimates the internal state of a system (e.g., position, velocity) based on noisy measurements and a predictive model of how the system behaves. The standard (linear) Kalman Filter assumes the system dynamics (state transitions) and measurement models are linear.

Real-world systems—especially those involving orientation, rotations, or non-linear sensor models (e.g., fusing IMU acceleration, odometry, GPS position, magnetometer) often do not follow purely linear equations. That’s where the Extended Kalman Filter (EKF) comes in. It's a widely used sensor fusion algorithm that handles non-linear system and measurement models by locally linearizing them. 

We'll learn ekf in depth during localization of our robot next session, but here we just see how the filtered odometry looks like.

First create a config directory inside the erc_gazebo_sensors package

Navigate to the package and run:
```bash
mkdir config
```
Don't forget to add it to `CMakeLists.txt`
```txt
cmake_minimum_required(VERSION 3.8)
project(erc_gazebo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config
  launch
  worlds
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

Then create `ekf.yaml` config file
```bash
touch ekf.yaml
```

Also we've to install the `robot-localization` package
```bash
sudo apt install ros-jazzy-robot-localization
```

Now add this code to the `ekf.yaml` config file
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_acceleration: false
    publish_tf: true


    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    odom0: odom
    odom0_config: [false, false, false,
                  false, false, false,
                  true, true, false,
                  false, false, true,
                  false, false, false]

    imu0: imu
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  false, false, true,
                  true, false, false]

    imu0_differential: false

    process_noise_covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]

    initial_estimate_covariance: [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]
```

Here encounter a issue about publishing out `tf` data, since earlier `gz_bridge_node` was doing this but we want ekf_filter_node to publish filtered `tf` data, so we'll comment the `tf` bridge 

```python
    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            #"/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

And then let's add the `robot_localization` and `trajectory_server`for both `odom` and `/odometry/filtered` to the launch file:

```python
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_erc_gazebo_sensors, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )
```

```python
    trajectory_odom_topic_node = Node(
        package='trajectory_server',
        executable='trajectory_server',
        name='trajectory_server_odom_topic',
        parameters=[{'trajectory_topic': 'trajectory_raw'},
                    {'odometry_topic': 'odom'}]
    )
```

```python
    trajectory_filtered_topic_node = Node(
    package='trajectory_server',
    executable='trajectory_server',
    name='trajectory_server_filtered',
    parameters=[
        {'trajectory_topic': 'trajectory'},
        {'odometry_topic': '/odometry/filtered'}
    ]
)
```

And of course, add the new nodes to the `launchDescription` :
```python
launchDescriptionObject.add_action(ekf_node)

launchDescriptionObject.add_action(trajectory_odom_topic_node)

launchDescriptionObject.add_action(trajectory_filtered_topic_node)
```

Rebuild the workspace and let's try it out!
```bash
ros2 launch bme_gazebo_sensors spawn_robot.launch.py
```

![alt text][image19]

We can see that the yellow (raw) odometry starts drifting away from the corrected one very quickly and we can easily bring the robot into a special situation if we drive on a curve and hit the wall.In this case the robot is unable to move and the wheels are slipping. The raw odometry believes from the encoder signals that the robot is still moving on a curve while the odometry after the ekf sensor fusion will believe that the robot moves forward straight. Although none of them are correct, but remember, neither the IMU and neither the odometry can tell if the robot is doing an uniform movement or it's stand still. At least the ekf is able to properly tell that the robot's orientation is not changing regardless what the encoders measure.

PERCEPTION

OPENCV

OpenCV (Open-Source Computer Vision Library) is an open-source library that includes several hundreds of computer vision algorithms. It helps us in performing various operations on images very easily.

In this section we'll look into one of the widely used application of openCV in ROS

We'll give our bot the intelligence to chase a red ball 

First create another package to store our nodes

Navigate to the src directory and run
```bash
ros2 pkg create --build-type ament_python erc_gazebo_sensors_py
```

Create a node inside the `eerc_gazebo_sensors_py` directory
```bash
touch chase_the_ball.py

chmod +x chase_the_ball.py
```

Don't forget to add this node to `setup.py`
```bash
entry_points={
     'console_scripts': [
         'chase_the_ball = erc_gazebo_sensors_py.chase_the_ball:main'
     ],
 },
```

Install the openCV package

Execute
```bash
pip --version
```
Ensure that pip is configured with python3.xx . If not you may have to use (pip3 --version).

Execute either

```bash
pip install opencv-contrib-python
#or
pip install opencv-python
```

Use pip3 in the above commands, if python3 is configured with one of them.

To test the installation, type python3 in Terminal to start Python interactive session and type following codes there.
```bash
import cv2 as cv
print(cv.__version__)
```

If you're encountering an issue with the cv2 (OpenCV) library and its interaction with the numpy library, then execute the following. We dont want the most recent version of numpy as it cannot interact with cv_bridge
```bash
pip install "numpy<2"
```

If the results are printed out without any errors, congratulations!! You have installed OpenCV-Python successfully.

Let's begin...

First making the node which subscribes to /camera/image topic as that contains the camera feed. It then converts it to OpenCV compatible frame and displays it using OpenCV.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None       

        # Flag to control the display loop
        self.running = True 

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                self.process_image(self.latest_frame)

                # Show the latest frame
                cv2.imshow("frame", self.latest_frame)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

            rclpy.spin_once(self, timeout_sec=0.05)

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Build and source your workspace.

Run
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```

In another terminal sourcing your workspace, run
```bash
ros2 run erc_gazebo_sensors_py chase_the_ball
```

![alt text][image21]

You can see the live camera feeds displayed through openCV window.

Now we'll process it with a new function `process_image` and create three more windows which will show the `binary - mask, object outlined - contour and tracking view - crosshair`, before that we need to do some more changes in our `__init__()` function and add some more functions:

Let's extend the node with better handling of the subscription to the image topic. If process_image() will take more time to run it will also block the execution of rclpy.spin_once(self, timeout_sec=0.05) that is needed to trigger the image_callback(). So let's move the spin functionality to a separate thread:

Update the `__init__()` constructor first:
```python
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()
```

Then add the `spin_thread_func()` function and also implement a thread lock in `image_callback()`:
```python
    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
```

Obviously, we don't need rclpy.spin_once(self, timeout_sec=0.05) anymore within display_image()!

Let's add a stop() function, too, to join the therads when we stop the node:
```python
    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()
```

And we'll call this stop() function when we stop the node in the main() function:
```python
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()
```

Finally create the `process_image` function after `display_image`
```python
    def process_image(self, img):
        """Image processing task."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        rows,cols = img.shape[:2]

        R,G,B = self.convert2rgb(img)

        redMask = self.threshold_binary(R, (220, 255))
        stackedMask = np.dstack((redMask, redMask, redMask))
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        # return value of findContours depends on OpenCV version
        (contours, hierarchy) = cv2.findContours(redMask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # Find the biggest contour (if detected)
        if len(contours) > 0:
            
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Show contour and centroid
            cv2.drawContours(contourMask, contours, -1, (0,255,0), 10)
            cv2.circle(contourMask, (cx, cy), 5, (0, 255, 0), -1)

            # Show crosshair and difference from middle point
            cv2.line(crosshairMask,(cx,0),(cx,rows),(0,0,255),10)
            cv2.line(crosshairMask,(0,cy),(cols,cy),(0,0,255),10)
            cv2.line(crosshairMask,(int(cols/2),0),(int(cols/2),rows),(255,0,0),10)

        # Return processed frames
        return redMask, contourMask, crosshairMask

    # Convert to RGB channels
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B

    # Apply threshold and result a binary image
    def threshold_binary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255
```

This time the node will open 4 OpenCV windows and try to find the red ball on the image. Let's add a red ball to the simulation first using the Resource Spawner plugin of Gazebo:

![alt text][image22]

Then let's see the new windows of our node:

![alt text][image23]

Handling many OpenCV windows can be uncomfortable, so before we start following the ball, let's overlay the output of the three image processing windows on the camera frame:

Define another function - `add_small_pictures`
```python
    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, size=(160, 120)):

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img
```

Then add this function in the `display_image`function and show only the `result` openCV window
```python
    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [mask, contour, crosshair])

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False
```

Finally give the bot velocity command based on the position of the ball in the tracking window

Add this to process_image function just before the `return`:
```python
...
            # Chase the ball
            if abs(cols/2 - cx) > 20:
                msg.linear.x = 0.0
                if cols/2 > cx:
                    msg.angular.z = 0.2
                else:
                    msg.angular.z = -0.2

            else:
                msg.linear.x = 0.2
                msg.angular.z = 0.0

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish cmd_vel
        self.publisher.publish(msg)
...
```
And now the robot is able to follow the red ball in the Gazebo simulation:

YOLOv8

In this section we'll dive into real-time object detection in ROS

Let's create another node, `yolo_detection_node.py`in the erc_gazebo_sensors_py package just like the `chase_the_ball.py` node and add it to `setup.py` 

Before we proceed, we have to install the `ultralytics` package

Here's the complete code of `yolo_detection_node`(refer recording or slides for reference)
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading
import time

from ultralytics import YOLO


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # Better model than yolov8n
        self.model = YOLO("yolov8s.pt")

        self.get_logger().info("YOLO model loaded")

        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1
        )

        self.bridge = CvBridge()

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.running = True

        self.spin_thread = threading.Thread(
            target=self.spin_thread_func,
            daemon=True
        )
        self.spin_thread.start()

        self.prev_time = time.time()

    def spin_thread_func(self):

        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        with self.frame_lock:
            self.latest_frame = frame

    def stop(self):

        self.running = False

        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1)

    def display_image(self):

        cv2.namedWindow(
            "YOLO Detection",
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
        )

        cv2.resizeWindow("YOLO Detection", 1600, 900)

        while rclpy.ok() and self.running:

            with self.frame_lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()

            if frame is not None:

                result = self.run_yolo(frame)

                cv2.imshow("YOLO Detection", result)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                self.running = False
                break

        cv2.destroyAllWindows()

    def run_yolo(self, frame):

        CONF_THRESHOLD = 0.35
        results = self.model(
            frame,
            conf=CONF_THRESHOLD,
            imgsz=640,
            verbose=False
        )

        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]
                detections.append(
                    f"{class_name} ({confidence:.2f})"
                )
                color = self.class_color(class_id)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2
                )
                label = f"{class_name} {confidence:.2f}"

                (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
                )
                text_y = max(y1 - 10, th + 10)

                cv2.rectangle(frame, (x1, text_y - th - baseline), (x1 + tw + 10, text_y + baseline), color, -1
                )
                cv2.putText(frame, label, (x1 + 5, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
                )
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(frame, (cx, cy), 5, color, -1
                )
                
        current_time = time.time()
        fps = 1.0 / max(current_time - self.prev_time, 1e-6)
        self.prev_time = current_time
        dashboard_width = 350
        dashboard = np.zeros(
            (frame.shape[0], dashboard_width, 3),
            dtype=np.uint8
        )

        cv2.putText(
            dashboard, "Detections", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2
        )

        cv2.putText(dashboard,f"FPS : {fps:.1f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        cv2.putText(dashboard, f"Objects : {len(detections)}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        y = 170

        for det in detections[:25]:

            cv2.putText(dashboard, det, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
            )

            y += 30

        combined = np.hstack((frame, dashboard))

        return combined

    def class_color(self, class_id):

        np.random.seed(class_id)

        return tuple(
            int(c)
            for c in np.random.randint(100, 255, 3)
        )


def main(args=None):

    print("OpenCV Version:", cv2.__version__)

    rclpy.init(args=args)

    node = YoloDetectorNode()

    try:
        node.display_image()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
