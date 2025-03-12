# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Gazebo Launch 8.X to 9.X

The gz-launch package is deprecated. See below for replacement of core features
and plugins.

* Launch functionality for running Gazebo, i.e. `sim_server`, `sim_gui`,
  `sim_factory` plugins
  * [ros2/launch](https://github.com/ros2/launch) is now recommended for
    launching Gazebo. The [ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim)
    package uses `ros2 launch` for launching Gazebo and spawning entities.
    See examples in
    [ros_gz_examples](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos).

* Launch functionality for running executables
  * Use Python or Ruby scripts (or the programming language or your choice) directly
    for launching processes. Alternatively,
    [ros2/launch](https://github.com/ros2/launch) also has the functionality
    to execute a process.

* `websocket_server` plugin
  * The plugin has been ported to a gz-sim system, see the websocket server
    [tutorial](https://github.com/gazebosim/gz-sim/blob/main/tutorials/websocket_server.md)
    in gz-sim for more informaton.

* `joystick` plugin
  * See the standalone
    [joystick](https://github.com/gazebosim/gz-sim/tree/main/examples/standalone/joystick)
    example in gz-sim.

* `joy_to_twist` plugin
  * See the standalone
    [joys_to_twist](https://github.com/gazebosim/gz-sim/tree/gz-sim9/examples/standalone/joy_to_twist)
    example in gz-sim.

### Other notes

* The environment variable `GZ_LAUNCH_INITIAL_CONFIG_PATH` is removed. Use
`gz::launch::getInitialConfigPath()` instead.

* The environment variable `GZ_LAUNCH_PLUGIN_INSTALL_PATH` is removed. Use
`gz::launch::getPluginInstallPath()` instead.

## Gazebo Launch 6.X to 7.X

- Removed Ignition
    - Removed the ignition include directory.
    - Launch scripts must use the `<gz>` element.

## Gazebo Launch 5.X to 6.X

- The `ignition` namespace is deprecated and will be removed in future versions.
  Use `gz` instead.

- Header files under `ignition/...` are deprecated and will be removed in future versions.
  Use `gz/...` instead.

- Migrate `IGN_LAUNCH_PLUGIN_PATH` environment variable to `GZ_LAUNCH_PLUGIN_PATH` for finding
  plugin.
  With tick-tock.

- Migrate `IGN_LAUNCH_CONFIG_PATH` environment variable to `GZ_LAUNCH_CONFIG_PATH` for finding
  configs.
  With tick-tock.

- The default config file path has been hard-tocked to `~/.gz/launch/gui.config`.
  `~/.ignition/launch/gui.config` will no longer work.

- The shared libraries have `gz` where there used to be `ignition`.
  - Using the un-migrated version is still possible due to tick-tocks, but will be removed in future versions.
- Launch files have been hard-tocked to the `.gzlaunch` extension instead of `.ign`.

## Gazebo Launch 2.2.2

- Environment variable `GZ_LAUNCH_CONFIG_PATH` started to be treated as a path
  list (colon-separated on Linux, semicolon-separated on Windows). Before, only
  a single path could be set here, and setting a path list would break the whole
  launch file lookup functionality.

## Gazebo Launch 0.X to N.M
