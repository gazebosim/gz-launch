# Ignition launch tutorial

Ignition Launch is used to run and manage plugins and programs. A configuration script can be used to specify which programs and plugins to execute. Alternatively, individual programs and plugins can be run from the command line.

In this tutorial we will explain the configuration scripts in the `examples` directory.

We can do many things:

* Run multiple executable commands
* Launch a world with a punch of plugins

To run the script

`ign launch FILE_NAME`

## Launch files

Every script starts with these two tags. which specify the `xml` version and `ignition` version.

```xml
<?xml version="1.0"?>
<ignition version='1.0'>
```

## Multiple executable

We can run multiple commands from one file.

if you have a look at the [gazebo.ign](../examples/gazebo.ign). This script run the gazebo `server` and gazebo `client`. We need to just write the command in the `command` tag.

```xml
<executable name='gazebo-server'>
    <command>ign gazebo -s</command>
</executable>
```

## Add plugins and launch worlds

If we look at the [factory.ign](../examples/factory.ign). We defined a `GazeboFactory` plugin under which we included `X2 UGV` robot and added the `DiffDrive` plugin to control the robot. And added the `StatePublisher` which publishes the the robot state information.

## Launch plugins with a world

The [gazebo_plugins.ign](../examples/gazebo_plugins.ign). This script loads some plugins. The `joystick` plugin that will read from a device and output to a topic. The `JoyToTwist` plugin that transforms a `joystick` message to a `twist` message. The `GazeboServer` plugin launches the gazebo server.

The script can take an world as an argument. To run this script.

`ign launch gazebo_plugins.ign [worldName:=<worldName>]`

The [worldName] command line argument is optional. If left blank, or not specified, then "diff_drive" is used for the world name.

Example to load `the shapes.sdf`.

`ign launch gazebo_plugins.ign worldName:=shapes`
