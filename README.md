# MujocoRosUtils
This repo originates from [this repo](https://github.com/isri-aist/MujocoRosUtils). It has been modified and improved so that it can be used in combination with [mujoco_ros_control](https://github.com/pucciland95/mujoco_ros2_control) to stream information from Mujoco simulation to ROS2 ecosystem or it can be used by itself.


## Features
The repo is composed of several scripts that implement [Mujoco Plugins](https://mujoco.readthedocs.io/en/stable/programming/extension.html#engine-plugins) to streamline the information from Mujoco to Ros2 and viceversa.
There are pluing for:
- Retrieve body poses
- Stream camera images
- Stream sensor data like Force/Torque sensors
- Send commands to actuators
- Apply external forces to the body in MuJoCo via ROS interfaces.

Since it is in plugin style, you can use it without rebuilding MuJoCo from the source.

## Install

### Requirements
- Compiler supporting C++17
- Tested with `Ubuntu 22.04 / ROS Humble`

### Dependencies
- [MuJoCo](https://github.com/deepmind/mujoco) (>= 2.3.5)

## Plugins
### MujocoRosUtils::ClockPublisher
Plugin to publish clock topic.

All of the following attributes are optional.
- `topic_name`: Topic name of clock. (Default is `/clock`)
- `publish_rate`: Publish rate. (Default is 100.0 [Hz])
- `use_sim_time`: Value of `use_sim_time` rosparam. (Default is `true`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ClockPublisher"/>
</extension>
<worldbody>
  <plugin plugin="MujocoRosUtils::ClockPublisher">
    <config key="topic_name" value="/clock"/>
    <config key="publish_rate" value="100"/>
    <config key="use_sim_time" value="true"/>
  </plugin>
</worldbody>
```
This plugin must be registered in worldbody.

### MujocoRosUtils::PosePublisher
Plugin to publish topics or broadcast TF of pose and velocity of the body.

All of the following attributes are optional.
- `frame_id`: Frame ID of topics header or TF parent. (Default is `map`)
- `pose_topic_name`: Topic name of pose. (Default is `mujoco/<body name>/pose`)
- `vel_topic_name`: Topic name of velocity. (Default is `mujoco/<body name>/vel`)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])
- `output_tf`: Whether to broadcast TF. (Default is `false`)
- `tf_child_frame_id`: Child frame ID for TF. Used only when `output_tf` is `true`. (Default is `<body name>`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::PosePublisher"/>
</extension>
<sensor>
  <plugin name="pose_publisher" plugin="MujocoRosUtils::PosePublisher" objtype="xbody" objname="object">
    <config key="frame_id" value="map"/>
    <config key="pose_topic_name" value="/pose"/>
    <config key="vel_topic_name" value="/vel"/>
    <config key="publish_rate" value="30"/>
    <config key="output_tf" value="false"/>
    <config key="tf_child_frame_id" value="object"/>
  </plugin>
</sensor>
```
The `objtype` attribute must be `xbody`.

### MujocoRosUtils::ExternalForce
Plugin to apply external force to the body.

All of the following attributes are optional.
- `topic_name`: Topic name of external force. (Default is `/external_force`)
- `vis_scale`: Arrow length scale. (Default is 0.1)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ExternalForce"/>
</extension>
<worldbody>
  <body name="object" pos="0 0 1">
    <freejoint/>
    <geom type="box" size="0.2 0.2 0.05" mass="0.1" rgba="0.5 0.5 0.5 0.3"/>
    <plugin plugin="MujocoRosUtils::ExternalForce">
      <config key="topic_name" value="/external_force"/>
      <config key="vis_scale" value="0.1"/>
    </plugin>
  </body>
</worldbody>
```

### MujocoRosUtils::ImagePublisher
Plugin to publish topics of color and depth images.

All of the following attributes are optional.
- `frame_id`: Frame ID of topics header or TF parent. (Default is `<camera name>`)
- `color_topic_name`: Topic name of color image. (Default is `mujoco/<camera name>/color`)
- `depth_topic_name`: Topic name of depth image. (Default is `mujoco/<camera name>/depth`)
- `info_topic_name`: Topic name of camera information. (Default is `mujoco/<camera name>/camera_info`)
- `height`: Image height. (Default is 240)
- `width`: Image width. (Default is 320)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ImagePublisher"/>
</extension>
<sensor>
  <plugin name="image_publisher" plugin="MujocoRosUtils::ImagePublisher" objtype="camera" objname="camera">
    <config key="frame_id" value="camera"/>
    <config key="color_topic_name" value="/image/color"/>
    <config key="depth_topic_name" value="/image/depth"/>
    <config key="info_topic_name" value="/image/camera_info"/>
    <config key="height" value="240"/>
    <config key="width" value="320"/>
    <config key="publish_rate" value="30"/>
  </plugin>
</sensor>
```
The `objtype` attribute must be `camera`.

### MujocoRosUtils::ActuatorCommand
Plugin to send a command to an actuator via ROS topic.

The following attributes are required.
- `actuator_name`: Actuator name to which the command is sent.
- `topic_name`: Topic name of actuator command. (Default is `mujoco/<actuator name>`)

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::ActuatorCommand"/>
</extension>
<actuator>
  <position name="camera_pan" joint="camera_pan"/>
  <plugin plugin="MujocoRosUtils::ActuatorCommand" joint="camera_pan">
    <config key="actuator_name" value="camera_pan"/>
    <config key="topic_name" value="/camera_pan"/>
  </plugin>
</actuator>
```
In the `plugin` element, you need to specify the `joint`, `body`, etc. of the actuator to be controlled.
This information is not used in the plugin, but is necessary to avoid errors in MJCF parsing.

The plugin itself is also added to the list of actuators, but it is a dummy actuator. The unwanted increase in the number of actuators (which also increases the dimension of `d->ctrl`) is a problem that should be solved in the future.

### MujocoRosUtils::SensorPublisher
Plugin to publish sensor data.

The following attributes are required.
- `sensor_name`: Name of sensor whose data is to be published.

The following attributes are optional.
- `frame_id`: Frame ID of message header. (Default is `map`)
- `topic_name`: Topic name. (Default is `mujoco/<sensor name>`)
- `publish_rate`: Publish rate. (Default is 30.0 [Hz])

An example of tags to be added to the MJCF file:
```xml
<extension>
  <plugin plugin="MujocoRosUtils::SensorPublisher"/>
</extension>
<sensor>
  <rangefinder name="box_rangefinder" site="object_center"/>
  <plugin name="sensor_publisher_scalar" plugin="MujocoRosUtils::SensorPublisher" objtype="xbody" objname="object">
    <config key="sensor_name" value="box_rangefinder"/>
    <config key="frame_id" value="map"/>
    <config key="topic_name" value="/box_rangefinder"/>
    <config key="publish_rate" value="30"/>
  </plugin>
</sensor>
```
In the `plugin` element, you need to specify the `objtype` and `objname`.
This information is not used in the plugin, but is necessary to avoid errors in MJCF parsing.
