## Ros Streamer Plugin

This is a ROS package for building and installing a GStreamer custom plugin for ROS streaming to a specific ROS topic


### Dependencies

`ros-stream` depends on [Deepstream](https://developer.nvidia.com/deepstream-sdk) libraries since this is custom tailored to deepstream pipelines

To validate that `deepstream` libraries are installed, make sure deepstream packages are installed to 

```
/opt/nvidia/deepstream/deepstream-5.0/libs
```

The build process requires that these libraries are installed to the default path according to the deepstream installiation process

### Installiation

Make sure ROS dependencies are installed using `rosdep`

```bash
# Make sure you are in your catkin_ws directory
$ roscd ../

# This will install packages for all projects within catkin_ws
~/catkin_ws$ rosdep install --from-paths src --ignore-src -r -y

# Alternativley, you can specify just the wvm-ros-stream package
~/catkin_ws$ rosdep install ros-stream

# Build catkin
~/catkin_ws$ catkin_make

# VERY IMPORTANT, COPY SHARED OBJECT FILE TO GST_PLUGIN_PATH!!!!!
# THIS CANNOT BE DONE USING INSTALL DUE TO PERMISSIONS AND CATKIN ENV
~/catkin_ws$ sudo cp -rv devel/lib/librosstream_plugin.so /opt/nvidia/deepstream/deepstream-5.0/lib/gst-plugins/
```

To validate that the installiation has gone correctly, run `gst-inspect-1.0` on the shared object library that is generated, the output should look similar to the following:

```bash
~/catkin_ws$ gst-inspect-1.0 /opt/nvidia/deepstream/deepstream-5.0/lib/gst-plugins/librosstream_plugin.so 

Plugin Details:
  Name                     rosstream_plugin
  Description              Ros Stream plugin for GST
  Filename                 /usr/lib/x86_64-linux-gnu/gstreamer-1.0/deepstream/librosstream_plugin.so
  Version                  1.0
  License                  MIT
  Source module            rosstream
  Binary package           Ros Stream plugin

  rosstream: RosStream Plugin

  1 features:
  +-- 1 elements

```

### Usage

The `rosstream` gst element provides an interface to directly stream images within a gst pipeline into ros. There are four optional arguments that can be set to better control this element

`ros-node-name`

`ros-topic-name`

`processing-width`

`processing-height`

This two arguments set the respective node and topic name within the pipeline

Before using the pipeline, ensure that you have a running `roscore` since this is embedded into gst, there is no easy way to launch roscore with the plugin.

An example plugin used to validate during my personal development is:

```bash
gst-launch-1.0 filesrc location=$HOME/sample_720p.mp4 ! qtdemux ! h264parse ! nvv4l2decoder ! m.sink_0 nvstreammux name=m batch-size=1 width=1280 height=720 ! nvvideoconvert ! rosstream ros-node-name=wvmrosstream ros_topic_name=camera_0  processing-width=1280 processing-height=720! nvdsosd ! nveglglessink
```

`rosstream ros-node-name=rosstream ros_topic_name=camera_0` is the main part of pipeline here which will launch the ros node with name `rosstream` and the topic name to `camera_0`

NOTE:

This pipeline assumes you have a video sample_720p.mp4 located in your $HOME directory. I got this video from the `/opt/nvidia/deepstream/deepstream-5.0/samples/streams/sample_720p.mp4`

So a quick copy can make the above pipeline work:

```bash
sudo cp /opt/nvidia/deepstream/deepstream-5.0/samples/streams/sample_720p.mp4 $HOME/
```
