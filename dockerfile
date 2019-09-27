FROM osrf/ros:kinetic-desktop-full-xenial
SHELL ["/bin/bash", "-c"]

COPY ./docker-entrypoint.sh /
ENTRYPOINT [ "/docker-entrypoint.sh" ]

# INSTALL PIP
RUN curl https://bootstrap.pypa.io/get-pip.py -o /get-pip.py \
    && python /get-pip.py \
    && rm /get-pip.py

# INSTALL CATKIN (per RTD)
RUN git clone https://github.com/catkin/catkin_tools.git /catkin_tools \
    && cd /catkin_tools \
    && git checkout e9504fcd574560bf85118e9edb73ee7eebbfe06f \
    && pip install -r requirements.txt --upgrade --ignore-installed PyYAML \
    && python setup.py install --record install_manifest.txt \
    && rm -rf /catkin_tools

# INSTALL pigpio
RUN git clone \https://github.com/joan2937/pigpio /pigpio \
    && cd /pigpio \
    && make \
    && make install \
    && rm -rf /pigpio

# Initialize Trinity Catkin Workspace
RUN mkdir -p /home/ros/catkin_ws/src \
    && cd /home/ros/catkin_ws/ \
    && catkin init

# Prepare ROS Dep.
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
        --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    && apt-get update -y \
    && apt-get upgrade -y \
    && apt-get install -y \
        python-rosdep \
        python-rosinstall-generator \
        python-wstool \
        python-rosinstall \
        build-essential \
        cmake \
        ros-kinetic-joy

# Copy & Configure Trinity Source
COPY src /home/ros/catkin_ws/src
RUN rosdep update && rosdep install \
    --from-paths /home/ros/catkin_ws/src \
    --ignore-src \
    --rosdistro kinetic \
    -y

# Catkin Build
WORKDIR /home/ros/catkin_ws/
RUN source /opt/ros/kinetic/setup.bash \
    && catkin build