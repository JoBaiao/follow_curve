# follow_curve
Package to simulate with turtlesim a non-holonomical robot following a curve using a vector field.

## How It Works

The script simulador.py contains the class TurtleBot and the main function, the latter creates an object TurtleBot and calls the method move_turtle. This method contains the coordinates of the desired curve and publishs the velocities necessited to follow it. The remaining methods are chained to this first one.

## Requirements

This package utilizes ROS. Follow download instructions for your distro in:
[ROS Download](http://wiki.ros.org/ROS/Installation)

## Instructions

Clone this repository into your `catkin_ws/src` folder:

```bash
$ git clone https://github.com/JoBaiao/follow_curve
```

Compile and source:

```bash
$ catkin build follow_curve
$ source ~/catkin_ws/devel/setup.bash
```

For a quick example, run:

```bash
$ roslaunch follow_curve simulador.launch
```

