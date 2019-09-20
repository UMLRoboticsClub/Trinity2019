FROM docker.pkg.github.com/roshanr10/ros-docker/ros:1.12.14
SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc

# Install pigpio
WORKDIR /
RUN git clone https://github.com/joan2937/pigpio
WORKDIR /pigpio
RUN make
RUN make install
RUN rm -rf /pigpio

# Initialize Catkin Workspace
RUN mkdir -p /Trinity2019/trinity_2019_ws/src
WORKDIR /Trinity2019/trinity_2019_ws
RUN catkin init

# Copy Trinity 2019 Source & Update Src
COPY . /Trinity2019/trinity_2019_ws/src

# Install ROSDeps for Trinity 2019 Source
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

# Init Catkin
WORKDIR /Trinity2019/trinity_2019_ws/src
RUN git clone https://github.com/ros/catkin.git
RUN catkin clean

# Catkin Build
WORKDIR /Trinity2019/trinity_2019_ws/src
#RUN catkin build