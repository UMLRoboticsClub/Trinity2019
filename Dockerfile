FROM docker.pkg.github.com/roshanr10/ros-docker/ros:1.12.14
SHELL ["/bin/bash", "-c"]
WORKDIR /

# Install pigpio
RUN git clone https://github.com/joan2937/pigpio /pigpio \
    && cd /pigpio \
    && make \
    && make install \
    && rm -rf /pigpio

# Initialize Catkin Workspace
RUN mkdir -p /Trinity2019/trinity_2019_ws/src \
    && cd /Trinity2019/trinity_2019_ws/ \
    && catkin init

# Copy Trinity 2019 Source & Update Src
COPY src /Trinity2019/trinity_2019_ws/src

# Install ROSDeps for Trinity 2019 Source
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    && apt-get update -y \
    && apt-get upgrade -y && apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake \
    && rosdep update \
    && rosdep install --from-paths /Trinity2019/trinity_2019_ws/src --ignore-src --rosdistro kinetic -y

# Catkin Build
WORKDIR /Trinity2019/trinity_2019_ws/
RUN git clone https://github.com/ros/catkin.git \
    && source /opt/ros/kinetic/setup.bash \
    && catkin build \
    && source /Trinity2019/trinity_2019_ws/devel/setup.bash