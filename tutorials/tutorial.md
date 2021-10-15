\page basics Ignition launch tutorial

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

The [gazebo_plugins.ign](../examples/gazebo_plugins.ign) launch file loads some plugins
and also starts simulation. The `joystick` plugin will be launched in its own process
and will read from a joystick device and output data onto a topic. The `JoyToTwist`
plugin also launches into a separate process and transforms a `joystick` message to a
`twist` message. Finally, The `GazeboServer` plugin launches the Gazebo server.

The script can take a world as an argument. To run this script.

`ign launch gazebo_plugins.ign [worldName:=<worldName>]`

The [worldName] command line argument is optional. If left blank, or not specified, then "diff_drive" is used for the world name.

Example to load `the shapes.sdf`:

`ign launch gazebo_plugins.ign worldName:=shapes`

## Launch file lookup

There is a lookup process happening if the specified file is not an absolute
path. It searches for a file with the given name in paths as follows:

1. current directory
1. all paths specified in environment variable `IGN_LAUNCH_CONFIG_PATH`
1. a hardcoded install location (usually
   `/usr/share/ignition/ignition-launchN/configs/`)

The `IGN_LAUNCH_CONFIG_PATH` environment variable can contain either a single
path or a path list (_new since 2.2.2_). Path list is a colon-separated (on
UNIX) or semicolon-separated (on Windows) list of absolute paths.