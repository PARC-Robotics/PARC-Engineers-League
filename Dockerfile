# This pulls osrf/ros:noetic-desktop-full and adds the necessary packages
# to run the robot code.

FROM osrf/ros:noetic-desktop-full

# Installs git

RUN apt-get update && apt-get install -y \
  git \
  libasio-dev \
  ros-noetic-teleop-twist-keyboard

# Creates a catkin workspace

RUN mkdir -p /home/parc/catkin_ws/src
WORKDIR /home/parc/catkin_ws/src

# Clones the robot code

RUN git clone https://github.com/PARC-Robotics/PARC-2023-EL.git --recursive --branch master

# Installs the necessary packages

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; rosdep install --from-paths . --ignore-src -r -y"

# Builds the robot code

WORKDIR /home/parc/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

# Add various sources to the bashrc

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/parc/.bashrc
RUN echo "source /home/parc/catkin_ws/devel/setup.bash" >> /home/parc/.bashrc

# Sets the entrypoint

WORKDIR /home/parc/catkin_ws/src/PARC-2023-EL

# This is the command that will be run when the container is started: roscore
ENTRYPOINT ["/bin/bash", "-c", "source /home/parc/.bashrc; roscore"]

