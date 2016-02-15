# Barn2ROS

## Save current map
```rosrun map_server map_saver -f map```

## Record all sensor data
```rosbag record -a```

## Playback sensor data
```rosbag play --clock file_name.bag```

## Setup machine
    export ROS_MASTER_URI=http://10.7.51.75:11311/
    export ROS_IP=10.7.51.{UNIQUE DIGITS}
    catkin_make # if build needed
    source devel/setup.{file format for your shell (bash or zsh)}
    
## Start ROS server of Jetson
    ssh ubuntu@10.7.51.75
    cd Barn2ROS
    source devel/setup.zsh
    roslaunch launch robot_localized.launch