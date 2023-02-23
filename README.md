# PARC-2023-EL

## Install GPS Sensor Plugin
`sudo apt-get install ros-noetic-hector-gazebo-plugins`

## Copy Farmland to Gazebo Models
`cp -r ~/catkin_ws/src/PARC-2023-EL/parc_robot/models/farmland ~/.gazebo/models`

## Run Tasks
Task 1
`roslaunch parc_robot parc_task1.launch`

Task 2
`roslaunch parc_robot parc_task2.launch`

Task 3
`roslaunch parc_robot parc_task3.launch`