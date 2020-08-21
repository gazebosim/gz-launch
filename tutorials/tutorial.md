# Ignition launch tutorial

Ignition Launch is used to run and manage plugins and programs. A configuration script can be used to specify which programs and plugins to execute. Alternatively, individual programs and plugins can be run from the command line.

In this tutorial we will explain the configuration scripts in the `examples` directory.

We can do many things:

* Run multiple executable commands
* Spawn a robot into simulation with plugins

To run a script use

`ign launch LAUNCH_FILE_NAME`

## Launch files

Every script starts with these two tags, which specify the `xml` version and `ignition` version.

```xml
<?xml version="1.0"?>
<ignition version='1.0'>
```

## Multiple executable commands

We can run multiple commands from one file.

Take a look at the [gazebo.ign](../examples/gazebo.ign) launch file. This script runs the Gazebo `server` and Gazebo `client`. We need to just write the command in the `command` tag.

```xml
<executable name='gazebo-server'>
    <command>ign gazebo -s</command>
</executable>
```

## Spawn a robot into simulation with plugins

Now take a look at the [factory.ign](../examples/factory.ign) launch file. We defined a `GazeboFactory` plugin under which we included an `X2 UGV` robot and added the `DiffDrive` plugin to control the robot. We also added the `StatePublisher` plugin which publishes the the robot state information.

## Launch simulation with plugins in separate processes

The [gazebo_plugins.ign](../examples/gazebo_plugins.ign) launch file loads some plugins and also starts simulation. The `joystick` plugin that will be launched in its own processes and will read from a joystick device and output data onto a topic. The `JoyToTwist` plugin also launches into a separate process and transforms a `joystick` message to a `twist` message. Finally, The `GazeboServer` plugin launches the Gazebo server.

The script can take an world as an argument. To run this script.

`ign launch gazebo_plugins.ign [worldName:=<worldName>]`

The [worldName] command line argument is optional. If left blank, or not specified, then "diff_drive" is used for the world name.

Example to load `the shapes.sdf`.

`ign launch gazebo_plugins.ign worldName:=shapes`
