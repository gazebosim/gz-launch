# Ignition Launch : Run and manage programs and plugins

**Maintainer:** nate AT openrobotics DOT org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-launch.svg)](https://github.com/ignitionrobotics/ign-launch/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-launch.svg)](https://github.com/ignitionrobotics/ign-launch/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-launch/branch/master/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-launch)
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_launch-ci-master-bionic-amd64)](https://build.osrfoundation.org/job/ignition_launch-ci-master-bionic-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_launch-ci-master-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_launch-ci-master-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_launch-ci-master-windows7-amd64)](https://build.osrfoundation.org/job/ignition_launch-ci-master-windows7-amd64)

Ignition Launch, a component of [Ignition
Robotics](https://ignitionrobotics.org), provides a command line interface
to run and manager application and plugins. 

# Table of Contents

[Features](#features)

[Install](#install)

* [Binary Install](#binary-install)

* [Source Install](#source-install)

    * [Prerequisites](#prerequisites)

    * [Building from Source](#building-from-source)

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

We recommend following the [Binary Install](#binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Binary Install

On Ubuntu systems, `apt` can be used to install `ignition-launch`:

```
sudo apt install libignition-launch<#>
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## Source Install

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

**[Ubuntu Bionic](http://releases.ubuntu.com/18.04/)**

1. Install third-party libraries:

    ```
    sudo apt-get -y install cmake build-essential curl cppcheck g++-8 doxygen ruby-ronn libtinyxml2-dev software-properties-common
    ```

1. Install required Ignition libraries

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
    ```

    ```
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

    ```
    sudo apt-get update
    ```

    ```
    sudo apt-get -y install libignition-cmake2-dev libignition-gazebo2-dev 
    ```

### Building from source

1. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-launch
    ```

2. Configure and build

    ```
    cd ign-launch; mkdir build; cd build; cmake ..; make
    ```

3. Optionally, install Ignition Launch

    ```
    sudo make install
    ```

# Usage

Sample launch configuration files are in the [examples directory](https://github.com/ignitionrobotics/ign-launch/blob/master/examples/).

**Example**

1. Run a configuration that launches [Gazebo](https://ignitionrobotics.org/libs/gazebo).

    ```
    ign launch gazebo.ign
    ```

## Known issue of command line tools

In the event that the installation is a mix of Debian and from source, command
line tools from `ign-tools` may not work correctly.

A workaround for a single package is to define the environment variable
`IGN_CONFIG_PATH` to point to the location of the Ignition library installation,
where the YAML file for the package is found, such as
```
export IGN_CONFIG_PATH=/usr/local/share/ignition
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
export IGN_CONFIG_PATH=$HOME/.ignition/tools/configs
```

This issue is tracked [here](https://github.com/ignitionrobotics/ign-tools/issues/8).

# Documentation

API and tutorials can be found at [https://ignitionrobotics.org/libs/launch](https://ignitionrobotics.org/libs/launch).

You can also generate the documentation from a clone of this repository by following these steps.

1. You will need Doxygen. On Ubuntu Doxygen can be installed using

    ```
    sudo apt-get install doxygen
    ```

2. Clone the repository

    ```
    git clone https://github.com/ignitionrobotics/ign-launch
    ```

3. Configure and build the documentation.

    ```
    cd ign-launch; mkdir build; cd build; cmake ../; make doc
    ```

4. View the documentation by running the following command from the build directory.

    ```
    firefox doxygen/html/index.html
    ```

# Testing

Follow these steps to run tests and static code analysis in your clone of this repository.

1. Follow the [source install instruction](#source-install).

2. Run tests.

    ```
    make test
    ```

3. Static code checker.

    ```
    make codecheck
    ```

# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
ign-launch
├── examples                 Example launch configurations.
├── include/ignition/launch  Header files.
├── src                      Source files and unit tests.
├── test
│    ├── integration         Integration tests.
│    ├── performance         Performance tests.
│    └── regression          Regression tests.
├── plugins                  Launch plugins, one per subdirectory.
├── Changelog.md             Changelog.
└── CMakeLists.txt           CMake build script.
```

# Contributing

Please see
[CONTRIBUTING.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CONTRIBUTING.md).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/master/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-launch/blob/master/LICENSE) file.
