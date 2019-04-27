# Get Started

1. [Install ignition-acropolis](https://ignitionrobotics.org/docs/acropolis/install).

2. Compile and install ign-launch from source. You will have to `sudo make
   install` so that `ign-tools` find the launch.yaml files.

```
sudo apt-get install libwebsockets-dev
mkdir build
cd build
cmake ../
make
sudo make install
```

3. Run ign-gazebo.

```
ign-gazebo -v 4
```

4. Run the websocket server. The follow assumes you are in the root of the
   source tree.

```
ign launch -f examples/websocket.ign -v 4
```

5. Open the `plugins/websocket_server/index.html` in a web browser.

```
firefox plugins/websocket_server/index.html
```
