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
    # --- DDS: CycloneDDS + iceoryx (iox-roudi) ---
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-cyclonedds \
    ros-jazzy-iceoryx-posh \
    ros-jazzy-iceoryx-introspection \
    # --- C++ bileşen altyapısı (licam_fusion, deos_cpp_common) ---
    ros-jazzy-rclcpp-components \
    ros-jazzy-message-filters \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-vision-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-ament-cmake-gtest \
    libeigen3-dev \
    ccache \
    procps \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Hailo runtime via apt (aynı versiyon host Pi ile uyumlu olmalı)
#    Pi OS Bookworm: hailo-all apt reposu /etc/apt/sources.list.d/ altında zaten ekli.
#    Bu image Ubuntu 24.04 tabanlı — Pi'de aşağıdaki komutla repo eklenmişse burada da çalışır:
#      sudo apt install hailo-all  (Pi OS Bookworm varsayılan reposundan)
#    Eğer Hailo apt reposu host'ta /etc/apt/sources.list.d/hailo.list ise docker run sırasında
#    volume-mount ile de geçirilebilir; alternatif olarak pip wheel kullanılabilir.
RUN apt-get update && apt-get install -y --no-install-recommends \
    hailort \
    && rm -rf /var/lib/apt/lists/* || true

# 3. Install Python packages with --break-system-packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pyrealsense2 \
    pyserial \
    pynmea2 \
    ultralytics || true

# Install hailo_platform for Python 3.12 (built from HailoRT v4.23.0 source)
COPY deps/hailo_platform /usr/lib/python3/dist-packages/hailo_platform
# COPY DEOS/deos_ws/src/libhailort.so.4.23.0 /usr/lib/libhailort.so.4.23.0
# RUN ln -sf /usr/lib/libhailort.so.4.23.0 /usr/lib/libhailort.so && ldconfig

# 4. Copy only the ROS workspace source (not the full repo) so builder and
#    runtime volume-mount paths both land at /ros2_ws/src/<package>.
COPY DEOS/deos_ws/src ./src/
# DDS + colcon yapılandırması (compose ile ./config olarak da mount edilir)
COPY config ./config

# 5. Update rosdep and install dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --skip-keys "hailort hailo_platform" || true

# 6. Build workspace — no --symlink-install so install has real files,
#    not symlinks that break when the build context disappears.
# Build tipi, ccache ve paralellik config/colcon/defaults.yaml'dan gelir.
ENV COLCON_DEFAULTS_FILE=/ros2_ws/config/colcon/defaults.yaml \
    CCACHE_DIR=/ros2_ws/.ccache
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Runtime stage: lean production image
FROM ros:jazzy

WORKDIR /ros2_ws

# Copy only built artifacts from builder stage
COPY --from=builder /ros2_ws/install ./install
COPY --from=builder /ros2_ws/src ./src
COPY --from=builder /ros2_ws/build ./build
COPY --from=builder /ros2_ws/config ./config

# --- Middleware: CycloneDDS varsayılan ---
# docker exec ile açılan shell'ler de aynı RMW'yi kullansın diye ENV olarak da
# tanımlı. SHM (iceoryx) seçimi çalışma zamanında ops/scripts/deos_dds_env.sh
# tarafından yapılır (DEOS_DDS_SHM=1).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///ros2_ws/config/dds/cyclonedds.xml \
    COLCON_DEFAULTS_FILE=/ros2_ws/config/colcon/defaults.yaml \
    CCACHE_DIR=/ros2_ws/.ccache

# Install runtime dependencies (mirrors builder stage system packages)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-setuptools \
    python3-pip \
    python3-numpy \
    python-is-python3 \
    python3-opencv \
    libopencv-dev \
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
    # --- DDS: CycloneDDS + iceoryx (iox-roudi) ---
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-cyclonedds \
    ros-jazzy-iceoryx-posh \
    ros-jazzy-iceoryx-introspection \
    # --- C++ bileşen altyapısı (licam_fusion, deos_cpp_common) ---
    ros-jazzy-rclcpp-components \
    ros-jazzy-message-filters \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-vision-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-ament-cmake-gtest \
    libeigen3-dev \
    ccache \
    procps \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pyrealsense2 \
    pyserial \
    pynmea2 \
    ultralytics || true

# Install hailo_platform for Python 3.12 (built from HailoRT v4.23.0 source)
COPY deps/hailo_platform /usr/lib/python3/dist-packages/hailo_platform
# COPY DEOS/deos_ws/src/libhailort.so.4.23.0 /usr/lib/libhailort.so.4.23.0
# RUN ln -sf /usr/lib/libhailort.so.4.23.0 /usr/lib/libhailort.so && ldconfig

# Set environment for ROS - suppress missing package warnings
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash 2>/dev/null; fi' >> /etc/bash.bashrc && \
    echo 'if pgrep -x iox-roudi >/dev/null 2>&1; then export CYCLONEDDS_URI=file:///ros2_ws/config/dds/cyclonedds_shm.xml; fi' >> /etc/bash.bashrc

WORKDIR /ros2_ws

CMD ["bash"]
