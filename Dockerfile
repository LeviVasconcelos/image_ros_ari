FROM nvidia/cuda:10.2-cudnn7-devel
##Mudei versao cuda pra 10.0
#ENV DEBIAN_FRONTEND="noninteractive"
#RUN echo "Installing dependencies..." && \
#	apt-get -y --no-install-recommends update && \
#	apt-get -y --no-install-recommends upgrade && \
#	apt-get install -y --no-install-recommends \
#	build-essential \
#        wget \
#        libssl-dev 
RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3-dev python3-pip git g++ wget make libprotobuf-dev protobuf-compiler libopencv-dev \
libgoogle-glog-dev libboost-all-dev libhdf5-dev libatlas-base-dev 

#for python api
RUN pip3 install numpy opencv-python 

#replace cmake as old version has CUDA variable bugs
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.16.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.16.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.16.0-Linux-x86_64/bin:${PATH}"

#get openpose
WORKDIR /openpose
# https://github.com/LeviVasconcelos/openpose.git .
#https://github.com/CMU-Perceptual-Computing-Lab/openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git .
#build it
WORKDIR /openpose/build
RUN cmake -DBUILD_PYTHON=ON .. && make -j `nproc`
RUN make install
WORKDIR /openpose

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

#############

RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-melodic-perception=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update
RUN apt-get install git python-rosinstall ros-melodic-desktop-full python-catkin-tools ros-melodic-joint-state-controller ros-melodic-twist-mux ros-melodic-ompl ros-melodic-controller-manager ros-melodic-moveit-core ros-melodic-moveit-ros-perception ros-melodic-moveit-ros-move-group ros-melodic-moveit-kinematics ros-melodic-moveit-ros-planning-interface ros-melodic-moveit-simple-controller-manager ros-melodic-moveit-planners-ompl ros-melodic-joy ros-melodic-joy-teleop ros-melodic-teleop-tools ros-melodic-control-toolbox ros-melodic-sound-play ros-melodic-navigation ros-melodic-depthimage-to-laserscan ros-melodic-moveit-commander  -y 
# Install ARI modules
WORKDIR /ari_public_ws
RUN pwd
ADD https://raw.githubusercontent.com/pal-robotics/ari_tutorials/master/ari_public-melodic.rosinstall ari_public-melodic.rosinstall
RUN rosinstall src /opt/ros/melodic ari_public-melodic.rosinstall
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update

##### PYTORCH
RUN apt-get install python-pip -y
RUN pip install torch torchvision rospkg future

## Installing lirealsense2 y hand :P
RUN git clone https://github.com/IntelRealSense/librealsense/
WORKDIR /ari_public_ws/librealsense/build
RUN pwd
RUN cmake .. 
RUN make -j $(nproc)
RUN make install
WORKDIR /ari_public_ws
RUN rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 librealsense2-dev librealsense2-dkms hey5_transmissions" -y 
WORKDIR /ari_public_ws/src
RUN git clone https://github.com/LeviVasconcelos/ros_openpose.git
RUN git clone https://github.com/LeviVasconcelos/FaceMaskDetection.git
RUN git clone https://github.com/LeviVasconcelos/entity_tracker.git
RUN chmod +x FaceMaskDetection/rosnode_facemask.py
RUN chmod +x ros_openpose/scripts/*.py
WORKDIR /ari_public_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
RUN catkin config --extend /opt/ros/$ROS_DISTRO
RUN catkin build -DCATKIN_ENABLE_TESTING=0
ADD launch.launch openpose_mask.launch
ADD head3_1280x960x20fps.bag head3_1280x960x20fps.bag
ADD mask_levi.bag mask_levi.bag
RUN apt-get install vim -y

WORKDIR /ari_public_ws
#ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
