# start with:
# docker run --mount source=plugin_volume,target=/plugin_volume -it iotbot_with_bt:latest
# + things from IOTBot regarding -v /dev:/dev --network host --priviliged....

# ros2 run bt_bot_plugin_manager manager --ros-args -p "watch_path:=/lalala"

#FROM ros:foxy-ros-base-focal
#FROM gramss/ros2:galactic-dev 
#FROM gramss/underlay_vs_workspace:latest
FROM gramss/ros2:galactic-tree-deps

USER root

# using bash
SHELL ["/bin/bash", "-c"]

# remove debconf errors (must run before any apt-get calls)
RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections

# install needed linux packages
# apt-get   https://wiki.ubuntuusers.de/apt/apt-get/
# -y        yes to all displayed questions     
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y apt-utils \
                          software-properties-common \
                          git \
                          vim \
                          nano \
                          nodejs

# install mraa lib for UART communication in shield node
RUN add-apt-repository ppa:mraa/mraa \
    && apt-get update \
    && apt-get install -y libmraa2 \
                          libmraa-dev \
                          libmraa-java \
                          python-mraa \
                          python3-mraa mraa-tools

# install Joystick Driver for ROS2 galactic
RUN sudo apt-get install -y ros-${ROS_DISTRO}-joy

USER ros
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno2</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

# set up iotbot workspace
RUN mkdir -p /home/ros/ws/overlay/src/iotbot

COPY . /home/ros/ws/overlay/src/iotbot

WORKDIR /home/ros/ws/overlay

# fix but don't know why..
RUN sudo rm -rf $HOME/.ros/rosdep
RUN sudo rm -rf /etc/ros/rosdep
RUN sudo rosdep init
RUN sudo rosdep update

# look for ROS2 package dependencies
# rosdep            https://docs.ros.org/en/independent/api/rosdep/html/commands.html
# -v, --verbose     verbose display
# -y, --default-yes tell the package manager to default to y or fail when installing
# -r                continue installing despite errors
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /home/ros/ws/underlay/install/setup.bash \
    && rosdep update \
    && sudo rosdep install -y -r \
                           --from-paths src \
                           --ignore-src \
                           --rosdistro=${ROS_DISTRO} \
                           --skip-keys "nav2_behavior_tree behaviortree_cpp_v3 nav2_util"

# build iotbot_interface first
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --packages-select iotbot_interface
    # && colcon test

# build other nodes after interface to find interface messages in those nodes
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /home/ros/ws/underlay/install/setup.bash \
    && source install/setup.bash \
    && colcon build --symlink-install 
    # M1 should handle this.. .)
    # && MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential
    # && colcon test

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /home/ros/ws/overlay/install/setup.bash" >> ~/.bashrc

# set an ENV variable in order to prepare for later Industrial EDGE usage
ENV plugin_volume=/plugin_volume
RUN sudo mkdir -p ${plugin_volume}/interfaces

# First: Find the shared library for the behavior_tree plugin and copy it into a accessible docker volume
# Second: Find the rosidl_interfaces libraries (2) and copy them in a dedicated interfaces folder

#CMD sudo find install/ -maxdepth 1 -name "*_bt_plugins" -exec cp -rf -L '{}/export/.' ${plugin_volume} \; \
#    && sudo find install/ -maxdepth 1 -name "*_interface" -exec find '{}/lib/' -name "*cpp.so" \; | xargs -I '{}' cp -rf {} ${plugin_volume}/interfaces/ \
#    && source install/setup.bash \
#    && ros2 launch iotbot_bringup drz_skid_performance_no_joy_launch.py

CMD source install/setup.bash \
    && ros2 launch iotbot_bringup drz_skid_performance_no_joy_launch.py