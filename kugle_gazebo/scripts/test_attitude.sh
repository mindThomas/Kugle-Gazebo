#!/bin/bash
rostopic pub /kugle/cmd_attitude geometry_msgs/Quaternion --once '{x: -0.1, y: 0.0, z: 0.0, w: 1.0}'
rostopic pub /kugle/cmd_attitude geometry_msgs/Quaternion --once '{x: 0.2, y: 0.0, z: 0.0, w: 1.0}'
rostopic pub /kugle/cmd_attitude geometry_msgs/Quaternion --once '{x: -0.1, y: 0.0, z: 0.0, w: 1.0}'
