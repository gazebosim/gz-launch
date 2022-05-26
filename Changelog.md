## Gazebo Launch 6.x

### Gazebo Launch 6.X.X (202X-XX-XX)

## Gazebo Launch 5.x

### Gazebo Launch 5.X.X (20XX-XX-XX)

### Gazebo Launch 5.1.0 (2022-03-21)
1. Use exec instead of popen to run ign-launch binary
    * [Pull request #151](https://github.com/gazebosim/gz-launch/pull/151)

1. Expose headless_rendering sdf tag
    * [Pull request #148](https://github.com/gazebosim/gz-launch/pull/148)

1. Remove spammy CMake message
    * [Pull request #142](https://github.com/gazebosim/gz-launch/pull/142)

### Gazebo Launch 5.0.0 (2021-10-01)

1. Port ign-launch to Windows
    * [Pull request #120](https://github.com/gazebosim/gz-launch/pull/120)

1. Add standalone executables
    * [Pull request #121](https://github.com/gazebosim/gz-launch/pull/121)
    * [Pull request #131](https://github.com/gazebosim/gz-launch/pull/131)

1. Remove unused includes (fix build)
    * [Pull request #119](https://github.com/gazebosim/gz-launch/pull/119)

1. Bumps dependencies for fortress
    * [Pull request #110](https://github.com/gazebosim/gz-launch/pull/110)

1. Infrastructure
    * [Pull request #113](https://github.com/gazebosim/gz-launch/pull/113)
    * [Pull request #106](https://github.com/gazebosim/gz-launch/pull/106)

## Gazebo Launch 4.x

### Gazebo Launch 4.X.X (20XX-XX-XX)

### Gazebo Launch 4.1.0 (2021-10-14)

1. All changes included in Gazebo Launch 3.4.2.

### Gazebo Launch 4.0.0 (2021-03-30)

1. Bump in edifice: ign-common4
    * [Pull request #90](https://github.com/gazebosim/gz-launch/pull/90)

1. Bump in edifice: sdformat11
    * [Pull request #87](https://github.com/gazebosim/gz-launch/pull/87)

1. Bump in edifice: ign-msgs7
    * [Pull request #86](https://github.com/gazebosim/gz-launch/pull/86)

1. Bump in edifice: ign-rendering5
    * [Pull request #69](https://github.com/gazebosim/gz-launch/pull/69)

## Gazebo Launch 3.x

### Gazebo Launch 3.4.2 (2021-10-14)

1. All changes included in Gazebo Launch 2.2.2.

### Gazebo Launch 3.4.1 (2021-07-15)

1. Generate a better error websocket error code for `max_connections`.
    * [Pull request 123](https://github.com/gazebosim/gz-launch/pull/123)

### Gazebo Launch 3.4.0 (2021-06-09)

1. Extend websocket server to support message limits and throttling.
    * [Pull request 116](https://github.com/gazebosim/gz-launch/pull/116)

### Gazebo Launch 3.3.0 (2021-05-06)

1. Convert depth and thermal image data to RGB before sending over websockets
    * [Pull request 112](https://github.com/gazebosim/gz-launch/pull/112)

### Gazebo Launch 3.2.1 (2021-04-19)

1. Add header to image msg.
    * [Pull request 109](https://github.com/gazebosim/gz-launch/pull/109)

### Gazebo Launch 3.2.0 (2021-04-12)

1. Support unsubscribing from a topic in the websocket server.
    * [Pull request 107](https://github.com/gazebosim/gz-launch/pull/107)

1. Support particle_emitters in the websocket server.
    * [Pull request 104](https://github.com/gazebosim/gz-launch/pull/104)

1. Support getting topic names and message types in the websocket server.
    * [Pull request 102](https://github.com/gazebosim/gz-launch/pull/102)

1. Image streaming over websocket.
    * [Pull request 97](https://github.com/gazebosim/gz-launch/pull/97)

1. Treat GZ_LAUNCH_CONFIG_PATH as a path list.
    * [Pull request 93](https://github.com/gazebosim/gz-launch/pull/93)


### Gazebo Launch 3.1.1 (2021-01-08)

1. All changes up to and including those in version 2.2.1.

### Gazebo Launch 3.1.0 (2020-12-10)

1. All changes up to and including those in version 2.2.0.

### Gazebo Launch 3.0.0 (2020-09-30)

1. Migration from BitBucket to GitHub
    * [Pull request 13](https://github.com/gazebosim/gz-launch/pull/13)
    * [Pull request 16](https://github.com/gazebosim/gz-launch/pull/16)
    * [Pull request 28](https://github.com/gazebosim/gz-launch/pull/28)

1. Add PKGCONFIG information to ignition-tools ign_find_package
    * [Pull request 44](https://github.com/gazebosim/gz-launch/pull/44)

1. Depend on ign-msgs6, ign-transport9
    * [Pull request 39](https://github.com/gazebosim/gz-launch/pull/39)

1. Depend on ign-gazebo4, ign-gui4
    * [BitBucket pull request 64](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/64)

## Gazebo Launch 2.x

### Gazebo Launch 2.2.2 (2021-10-11)

1. Master branch updates.
    * [Pull request 98](https://github.com/gazebosim/gz-launch/pull/98)

1. Treat `GZ_LAUNCH_CONFIG_PATH` as a path list.
    * [Pull request 93](https://github.com/gazebosim/gz-launch/pull/93)

1. Remove tools/code_check and update codecov.
    * [Pull request 115](https://github.com/gazebosim/gz-launch/pull/115)

1. Update gtest for Windows compilation.
    * [Pull request 122](https://github.com/gazebosim/gz-launch/pull/122)

1. Remove bitbucket-pipelines.yml.
    * [Pull request 128](https://github.com/gazebosim/gz-launch/pull/128)

### Gazebo Launch 2.2.1 (2021-01-08)

1.  Fix env parsing by placing it before executable parsing.
    * [Pull request 81](https://github.com/gazebosim/gz-launch/pull/81)
    * [Pull request 82](https://github.com/gazebosim/gz-launch/pull/82)

### Gazebo Launch 2.2.0 (2020-10-14)

1. All changes up to and including those in version 1.10.0

1. Added a tutorial.
    * [Pull request 48](https://github.com/gazebosim/gz-launch/pull/48)

### Gazebo Launch 2.1.0 (2020-05-21)

1. Merged ign-launch1 forward.
    * [Pull request 41](https://github.com/gazebosim/gz-launch/pull/31)

### Gazebo Launch 2.0.0

1. Use Gazebo Citadel dependencies.
    * [BitBucket pull request 47](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/47)
    * [BitBucket pull request 50](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/50)
    * [BitBucket pull request 53](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/53)

1. Allow specifying a custom window title instead of "Gazebo"
    * [BitBucket pull request 31](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/31)

1. Unversioned lib name for cmds
    * [BitBucket pull request 55](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/55)

## Gazebo Launch 1.x

### Gazebo Launch 1.X.X (20XX-XX-XX)

### Gazebo Launch 1.10.0 (2020-09-25)

* Modernize Github Actions CI.
    * [Pull request 42](https://github.com/gazebosim/gz-launch/pull/42)

1. Add PKGCONFIG information to ignition-tools ign_find_package
   * [Pull Request 44](https://github.com/gazebosim/gz-launch/pull/44)

1. Fix factory.ign launch file
   * [Pull Request 55](https://github.com/gazebosim/gz-launch/pull/55)

1. Use random name for manager semaphore
   * [Pull Request 57](https://github.com/gazebosim/gz-launch/pull/57)

1. Add support for specifying topics to record
   * [Pull Request 54](https://github.com/gazebosim/gz-launch/pull/54)

1. Fix race condition in websocket server.
   * [Pull Request 68](https://github.com/gazebosim/gz-launch/pull/68)

### Gazebo Launch 1.9.0 (2020-08-13)

1. Added HTTP handling support to websocket server and a metrics HTTP endpoint
   to monitor websocket server status.
   * [Pull Request 49](https://github.com/gazebosim/gz-launch/pull/49)

### Gazebo Launch 1.8.0 (2020-07-28)

1. Added `<max_connections>` to the websocket server that supports
   specifying the maximum allowed websocket connections.
    * [Pull Request 40](https://github.com/gazebosim/gz-launch/pull/40)

### Gazebo Launch 1.7.1 (2020-06-23)

1. Improve websocket performance by throttling the busy loop, and fix empty SSL
   XML elements.
    * [Pull Request 37](https://github.com/gazebosim/gz-launch/pull/37)

### Gazebo Launch 1.7.0 (2020-06-16)

1. Added SSL to websocket server.
    * [Pull Request 34](https://github.com/gazebosim/gz-launch/pull/34)

### Gazebo Launch 1.6.0 (2020-06-11)

1. Improved websockets by: adding simple authentication, access to
   protobuf message definitions, access to scene and world information, and
   definition of custom message framing.
    * [Pull Request 22](https://github.com/gazebosim/gz-launch/pull/22)
    * [Pull Request 21](https://github.com/gazebosim/gz-launch/pull/21)
    * [Pull Request 17](https://github.com/gazebosim/gz-launch/pull/17)
    * [Pull Request 33](https://github.com/gazebosim/gz-launch/pull/33)

### Gazebo Launch 1.5.0 (2020-05-20)

1. Added support for spawning multiple entities in the same simulation step.
    * [Pull Request 30](https://github.com/gazebosim/gz-launch/pull/30)

### Gazebo Launch 1.4.2 (2020-05-18)

1.  Use the new GUI API of ign-gazebo. This adds support for saving worlds to SDFormat from the GUI.
    * [Pull Request 19](https://github.com/gazebosim/gz-launch/pull/19)

### Gazebo Launch 1.4.1 (2019-12-05)

1. Unversioned lib name for cmds
    * [BitBucket pull request 56](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/56)

### Gazebo Launch 1.4.0 (2019-11-26)

1. Default GUI config for Launch
    * [BitBucket pull request 51](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/51)

### Gazebo Launch 1.3.0 (2019-11-13)

1. Add filepath to ERB so that constants like `__FILE__` in `.ign` files work as expected
    * [BitBucket pull request 48](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/48)

1.  Add backward support to ign-launch to capture backtraces.
    * [BitBucket pull request 41](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/41)

### Gazebo Launch 1.2.3 (2019-09-09)

1. Add QML import path to IgnGazebo modules.
    * [BitBucket pull request 42](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/42)

### Gazebo Launch 1.2.2

1. Add support for console logging.
    * [BitBucket pull request 40](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/40)

1. Support setting custom window icons.
    * [BitBucket pull request 39](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/39)

### Gazebo Launch 1.2.1

1. Eliminate potential deadlock from SIGCHLD signal handler
    * [BitBucket pull request 36](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/36)

### Gazebo Launch 1.2.0

1. Support for custom random seed in the GazeboServer plugin.
    * [BitBucket pull request 33](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/33)

1. Allow specifying a custom window title
    * [BitBucket pull request 32](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/32)

### Gazebo Launch 1.1.0 (2019-06-11)

1. Added command line parameters of the form <name>:=<value>. These parameters are passed to ERB.
    * [BitBucket pull request 27](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/27)

### Gazebo Launch 1.0.1 (2019-05-22)

1. Fix GazeboFactory set performer topic
    * [BitBucket pull request 26](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/26)

### Gazebo Launch 1.0.0 (2019-05-21)

1. Enable logging with `<record>` tag in ign launch file.
    * [BitBucket pull request 23](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/23)

1. Fix gazebo gui to use GuiRunner.
    * [BitBucket pull request 22](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/22)

1. Fix parsing positional argument to ign tool.
    * [BitBucket pull request 21](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/21)

1. Added world stats to Websocket server.
    * [BitBucket pull request 20](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/20)

1. Change project name to ignition-launch1.
    * [BitBucket pull request 19](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/19)

1. Depend on gazebo2, gui2, msgs4, transport7.
    * [BitBucket pull request 17](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/17)

1. Added Websocket server.
    * [BitBucket pull request 15](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/15)
    * [BitBucket pull request 18](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/18)

## Gazebo Launch 0.x

1. Install examples folder.
    * [BitBucket pull request 14](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-launch/pull-requests/14)

### Gazebo Launch 0.2.0
