FROM ros:melodic-perception

SHELL [ "/bin/bash", "-c" ]

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update && \
    apt-get install -y git ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-ros-numpy ros-$ROS_DISTRO-geometry && \
    apt-get clean

# install bio_ik
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /bio_ik_ws/src && \
    cd /bio_ik_ws/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/TAMS-Group/bio_ik.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /bio_ik_ws

RUN chmod +x /workspace/src/install.sh
RUN sudo sh /workspace/src/install.sh
RUN chmod +x /workspace/src/install-opencv.sh
RUN sudo sh /workspace/src/install-opencv.sh
RUN chmod +x /workspace/src/install-darknet.sh
RUN sudo sh /workspace/src/install-darknet.sh

# create workspace folder
RUN mkdir -p /workspace/src

# copy our algorithm to workspace folder
ADD . /workspace/src

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# compile and install our algorithm
RUN cd /workspace && /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO

RUN mkdir ~/workspace
RUN cd ~/workspace
RUN mkdir -p catkin_ws/src

RUN cd ~/workspace/catkin_ws/src
RUN git clone https://github.com/ARTenshi/wrs2020.git

#RUN cd ~/workspace/catkin_ws/src/wrs2020
#RUN chmod +x install.sh
#RUN sudo ./install.sh

#RUN cd ~/workspace/catkin_ws/src/wrs2020
#RUN chmod +x install-opencv.sh
#RUN sudo ./install-opencv.sh

#RUN cd ~/workspace/catkin_ws/src/wrs2020
#RUN chmod +x install-darknet.sh
#RUN sudo ./install-darknet.sh

RUN cd ~/workspace/catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin_make

RUN rosparam set /hsrb/omni_base_controller/base_coordinates "[odom_x,odom_y,odom_r]"
RUN rosparam set /hsrb/impedance_control/joint_names "[]"
RUN source ~/workspace/catkin_ws/devel/setup.bash

RUN rosnode kill /iris_lama_loc2d

# command to run the algorithm
CMD roslaunch wrs_challenge run.launch
