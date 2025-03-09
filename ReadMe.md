# NNG-Sub
Use nng connect ROS1(Ubuntu 18.04) and ROS2(Ubuntu 20.04).

ROS1(Pub) -> ROS2(Sub)

This one is for **ROS2 sub**, should use with ROS1 pub (`https://github.com/Si-lhouette/realmodule_for_unitree/tree/main/src/network/nng`).


### Dependency
Download the v1.10.1 NNG from github release: `https://github.com/nanomsg/nng/releases`
```
sudo apt install ninja-build
cd nng-1.10.1
mkdir build
cmake -G Ninja ..
ninja
ninja test
sudo ninja install
```
The `ninja test` may report that `99% succ, but 1 fail`, just omit it, we can use it anyway.

Check the installaion:
```
nngcat --version
```

### Run 
Run the ROS1 pub:
```
rosrun nng nng_pub
```
the pub will subs the Ros topic, then send the topic with NNG.

Run the ROS2 sub:
```
ros2 run nng_ros2 nng_sub
```
the sub will receive the msg send with NNG, and repub in ROS topic.


