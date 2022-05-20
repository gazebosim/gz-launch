# Ignition Launch : Run and manage programs and plugins

**Maintainer:** nate AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-launch.svg)](https://github.com/ignitionrobotics/ign-launch/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-launch.svg)](https://github.com/ignitionrobotics/ign-launch/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-launch/branch/main/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-launch)
Ubuntu Focal | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_launch-ci-main-focal-amd64)](https://build.osrfoundation.org/job/ignition_launch-ci-main-focal-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_launch-ci-main-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_launch-ci-main-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/job/ign_launch-ign-5-win/badge/icon)](https://build.osrfoundation.org/job/ign_launch-ign-5-win/)

Ignition Launch, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides a command line interface
to run and manager application and plugins.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Documentation](#documentation)

[Testing](#testing)

[Folder Structure](#folder-structure)

[Contributing](#contributing)

[Code of Conduct](#code-of-conduct)

[Versioning](#versioning)

[License](#license)

# Features

Ignition Launch is used to run and manage plugins and programs. A
configuration script can be used to specify which programs and plugins to
execute. Alternatively, individual programs and plugins can be run from the
command line. Example configuration scripts are located in the `examples`
directory.

1. Automatic ERB parsing of configuration files.
1. Pass arguments to launch files from the command line.
1. Plugins to launch Gazebo, joystick interface, and a websocket server for
   simulation.

# Install

See the [installation tutorial](https://ignitionrobotics.org/api/launch/5.0/install.html).

# Usage

Sample launch configuration files are in the [examples directory](https://github.com/ignitionrobotics/ign-launch/blob/main/examples/).

**Example**

1. Run a configuration that launches [Gazebo](https://ignitionrobotics.org/libs/gazebo).

    ```
    ign launch gazebo.ign
    ```

## Known issue of command line tools

In the event that the installation is a mix of Debian and from source, command
line tools from `ign-tools` may not work correctly.

A workaround for a single package is to define the environment variable
`GZ_CONFIG_PATH` to point to the location of the Ignition library installation,
where the YAML file for the package is found, such as
```
export GZ_CONFIG_PATH=/usr/local/share/ignition
```

However, that environment variable only takes a single path, which means if the
installations from source are in different locations, only one can be specified.

Another workaround for working with multiple Ignition libraries on the command
line is using symbolic links to each library's YAML file.
```
mkdir ~/.ignition/tools/configs -p
cd ~/.ignition/tools/configs/
ln -s /usr/local/share/ignition/fuel4.yaml .
ln -s /usr/local/share/ignition/transport7.yaml .
ln -s /usr/local/share/ignition/transportlog7.yaml .
...
export GZ_CONFIG_PATH=$HOME/.ignition/tools/configs
```

This issue is tracked [here](https://github.com/ignitionrobotics/ign-tools/issues/8).

# Documentation

See the [installation tutorial](https://ignitionrobotics.org/api/launch/5.0/install.html).

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
ign-launch
├── examples                 Example launch configurations.
├── include/ignition/launch  Header files.
├── plugins                  Launch plugins, one per subdirectory.
├── src                      Source files and unit tests.
├── test
│    ├── integration         Integration tests.
│    ├── performance         Performance tests.
│    └── regression          Regression tests.
├── tools                    Some useful tools such as linters
├── tutorials                Tutorials
├── Changelog.md             Changelog.
└── CMakeLists.txt           CMake build script.
```

# Contributing

Please see the [contribution guide](https://ignitionrobotics.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-launch/blob/main/LICENSE) file.
