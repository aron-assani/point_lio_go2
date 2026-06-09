ARG NETWORK_INTERFACE=enx2887ba59bf0a
ARG GIT_PAT=""
ARG POINT_LIO_REPO="https://github.com/aron-assani/point_lio_go2.git"

# Builder stage: clone the private repo using the provided PAT (if any)
FROM alpine/git:latest AS gitclone
ARG GIT_PAT
ARG POINT_LIO_REPO
WORKDIR /src
RUN if [ -n "$GIT_PAT" ]; then \
      git clone https://$GIT_PAT@github.com/aron-assani/point_lio_go2.git point_lio; \
    else \
      git clone $POINT_LIO_REPO point_lio; \
    fi

# Final image
FROM osrf/ros:humble-desktop

ARG NETWORK_INTERFACE
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/root/ros2_ws

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///root/cyclonedds.xml

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    nano \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2 \
    python3-tk \
    python3-pip \
    procps \
    && rm -rf /var/lib/apt/lists/*

# Point-LIO Python dependencies
RUN pip3 install "numpy>=2.0.0" "scipy>=1.14.0" motioncapture pyyaml   

# CycloneDDS configuration
RUN cat <<EOF > /root/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="${NETWORK_INTERFACE}"/>
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
    </Domain>
</CycloneDDS>
EOF

RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy the repo from the builder stage
COPY --from=gitclone /src/point_lio ${ROS_WS}/src/point_lio

# Clone unilidar_sdk2 repository
RUN git clone https://github.com/unitreerobotics/unilidar_sdk2.git src/unilidar_sdk2 && rm -rf src/unilidar_sdk2/unitree_lidar_ros

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Install unitree_sdk2_python from source
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git src/unitree_sdk2_python && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && cd src/unitree_sdk2_python && pip3 install -e ."

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

# Default to interactive bash
CMD ["/bin/bash"]