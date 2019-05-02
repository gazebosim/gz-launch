# Ignition Launch - Run and manage programs and plugins 

  http://ignitionrobotics.org

## Build and Install


```
mkdir build
```

```
cd build
```


```
cmake ../
```

```
make install
```

## Run

Igntion Lauch is a system that runs and manages plugins and programs. A
configuration script can be used to specify which programs and plugins to
run. Alternatively, individual programs and plugins can be run from the
command line. Example configuration scripts are located in the `config`
directory.

**Example**

1. Run a configuration that launches [Gazebo](https://ignitionrobotics.org/libs/gazebo).

    ```
    ign launch gazebo.ign
    ```
