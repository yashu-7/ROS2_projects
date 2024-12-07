# Line Follower robot using Camera
---
## To use the package place it on **src** folder in your **ROS2 workspace**
* Build the package using colcon build
```
# cd <path-to-your-ros2-ws>
colcon build
```
---
* Then launch the gazebo harmonic simulator using ros2 launh command
```
# This launches simulator along with the world with red track
ros2 launch line_follower gz_sim.launch.py
```

> There are 2 codes one is for open loop control and 1 for proportional control
* Now run the node that captures the camera feed and returns a binary image where the red color is segmented
```
ros2 run line_follower image_segment
```
---
* Now run the 2 controller codes one after another to check the algorithm
```
# first try the open loop control
ros2 run line_follower control
```
---
> stop the control node and run the next one
```
# proportional controller
ros2 run line_follower p_control
```
## You can notice the difference in working 