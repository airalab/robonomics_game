# Robonomics Game
for Airalab Industry 4.0 testbed

[3D model visualization on YouTube](https://youtu.be/sFSC-9TZYOY)

Implementation of [AIP-3, Game software proposal](https://github.com/airalab/DAO-Airalab/blob/master/AIPs/aip-3.md)

Usage
-----
First launch ROS OPC UA client and connect to OPC UA server.
```
roslaunch ros_opcua_impl_freeopcua client.launch
rosservice call /opcua/opcua_client/connect "endpoint: 'opc.tcp://0.0.0.0:53880'"
```

Check example launch files to see how to run nodes.
