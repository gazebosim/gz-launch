\page install Installation

# Install

These instructions are for installing only Ignition Launch. If you're interested
in using all the Ignition libraries, not only Igniton Launch, check out this
[Ignition installation](https://ignitionrobotics.org/docs/latest/install).

We recommend following the binary install instructions to get up and running as
quickly and painlessly as possible.

The source install instructions should be used if you need the very latest
software improvements, if you need to modify the code, or if you plan to make a
contribution.

# Binary Install

We recommend following the [Binary Install](#binary-install) instructions to get up and running as quickly and painlessly as possible.

The [Source Install](#source-install) instructions should be used if you need the very latest software improvements, you need to modify the code, or you plan to make a contribution.

## Ubuntu 20.04 or above

On Ubuntu systems, `apt` can be used to install `ignition-launch`:

```
sudo apt install libignition-launch<#>
```

Be sure to replace `<#>` with a number value, such as 1 or 2, depending on
which version you need.

## macOS

On macOS, add OSRF packages:
  ```
  ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
  brew tap osrf/simulation
  ```

Install Ignition GUI:
  ```
  brew install ignition-launch<#>
  ```

Be sure to replace `<#>` with a number value, such as 4 or 5, depending on
which version you need.

# Source Install


## Ubuntu 20.04 or above

Source installation can be performed in UNIX systems by first installing the
necessary prerequisites followed by building from source.

### Prerequisites

**[Ubuntu Focal](http://releases.ubuntu.com/20.04/)** or higher

1. Install third-party libraries:

    ```
    sudo apt -y install \
      $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
    ```

1. Install required Ignition libraries

    ```sh
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get -y install libignition-cmake2-dev libignition-gazebo2-dev
    ```

### Building from source

1. Clone the repository

    ```sh
    git clone https://github.com/ignitionrobotics/ign-launch
    ```

2. Configure and build

    ```sh
    cd ign-launch; mkdir build; cd build; cmake ..; make
    ```

3. Optionally, install Ignition Launch

    ```sh
    sudo make install
    ```

## macOS

1. Clone the repository
  ```
  git clone https://github.com/ignitionrobotics/ign-launch -b ign-launch<#>
  ```
  Be sure to replace `<#>` with a number value, such as 4 or 5, depending on
  which version you need.

2. Install dependencies
  ```
  brew install --only-dependencies ignition-launch<#>
  ```
  Be sure to replace `<#>` with a number value, such as 5 or 6, depending on
  which version you need.

3. Configure and build
  ```
  cd ign-launch
  mkdir build
  cd build
  cmake ..
  make
  ```

4. Optionally, install
  ```
  sudo make install
  ```

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
