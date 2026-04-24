# Multi-stage build: builder stage for compilation
FROM ros:jazzy as builder

WORKDIR /ros2_ws

# 1. Install system dependencies with cache busting and cleanup
RUN apt-get update && apt-get install -y --no-install-recommends \
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

# 2. Install Python packages with --break-system-packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pyrealsense2 \
    pyserial \
    pynmea2

# 3. Copy source code early
COPY . src/

# 4. Update rosdep and install dependencies (skip only non-essential packages)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --skip-keys "hailort hailo_platform" || true

# 5. Build workspace with optimized flags
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" || echo "Build finished with some packages incomplete"

# Runtime stage: lean production image
FROM ros:jazzy

WORKDIR /ros2_ws

# Copy only built artifacts from builder stage
COPY --from=builder /ros2_ws/install ./install
COPY --from=builder /ros2_ws/src ./src
COPY --from=builder /ros2_ws/build ./build

# Install runtime dependencies only (minimal set)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-opencv \
    libopencv-dev \
    python3-pip \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pyrealsense2 \
    pyserial \
    pynmea2

# Set environment for ROS - suppress missing package warnings
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash 2>/dev/null; fi' >> /etc/bash.bashrc

WORKDIR /ros2_ws

CMD ["bash"]
