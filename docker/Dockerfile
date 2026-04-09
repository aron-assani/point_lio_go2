FROM osrf/ros:humble-desktop

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
    ros-humble-tf-transformations \
    ros-humble-rosbag2 \
    python3-transforms3d \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install motioncapture scipy pyyaml numpy

RUN cat <<EOF > /root/cyclonedds.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="enx2887ba59bf0a"/>
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
    </Domain>
</CycloneDDS>
EOF

RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Clone unilidar_sdk2 repository
RUN git clone https://github.com/unitreerobotics/unilidar_sdk2.git src/unilidar_sdk2
RUN rm -rf src/unilidar_sdk2/unitree_lidar_ros

# Clone point_lio_go2 repository
RUN git clone https://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@github.com/aron-assani/point_lio_go2.git src/point_lio

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]