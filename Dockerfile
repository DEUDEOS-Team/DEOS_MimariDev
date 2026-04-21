FROM ros:jazzy

WORKDIR /ros2_ws

# 1. AŞAMA: Python Kütüphaneleri ve Kritik ROS 2 Paketleri
# Numpy, setuptools ve tf2_geometry_msgs gibi derlemeyi durduran tüm eksikleri baştan kuruyoruz.
RUN apt-get update && apt-get install -y \
    python3-setuptools \
    python3-pip \
    python3-numpy \
    python-is-python3 \
    python3-opencv \
    python3-colcon-common-extensions \
    ros-jazzy-pcl-ros \
    ros-jazzy-perception-pcl \
    ros-jazzy-pcl-conversions \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-nav-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-robot-localization \
    ros-jazzy-sick-scan-xd \
    && rm -rf /var/lib/apt/lists/*

# 2. AŞAMA: Kodların Kopyalanması
COPY . src/

# 3. AŞAMA: Rosdep ve Bypass İşlemi
# Rosdep'i güncelliyor ve bulamadığı donanım/dış paketleri (hailo, pcl) atlayarak kuruluma zorluyoruz.
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --skip-keys "hailort hailo_platform pcl_localization" || true

# 4. AŞAMA: Derleme (Build)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

CMD ["bash"]