# HailoRT sürümü: hosttaki "hailort --version" çıktısıyla eşleşmeli.
# Build sırasında override etmek için: docker build --build-arg HAILO_VERSION=4.19.0 ...
ARG HAILO_VERSION=4.19.0
ARG TARGETARCH=arm64

# Multi-stage build: builder stage for compilation
FROM ros:jazzy AS builder

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
    pynmea2 \
    ultralytics

# 3. Copy only the ROS workspace source (not the full repo) so builder and
#    runtime volume-mount paths both land at /ros2_ws/src/<package>.
COPY DEOS/deos_ws/src ./src/

# 4. Update rosdep and install dependencies (skip only non-essential packages)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y \
    --skip-keys "hailort hailo_platform" || true

# 5. Build workspace — no --symlink-install so install has real files,
#    not symlinks that break when the build context disappears.
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Runtime stage: lean production image
FROM ros:jazzy

# ARG'ı runtime stage'e de taşı (FROM sonrası sıfırlanır)
ARG HAILO_VERSION=4.19.0
ARG TARGETARCH=arm64

WORKDIR /ros2_ws

# Copy only built artifacts from builder stage
COPY --from=builder /ros2_ws/install ./install
COPY --from=builder /ros2_ws/src ./src
COPY --from=builder /ros2_ws/build ./build

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
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir --break-system-packages \
    pyrealsense2 \
    pyserial \
    pynmea2 \
    ultralytics

# ---------------------------------------------------------------------------
# HailoRT kurulumu
#
# Hosttaki kernel driver ile aynı sürümde olması ZORUNLU.
# Hostta sürümü kontrol et: hailort --version  (veya dmesg | grep hailo)
#
# Seçenek A (varsayılan): .deb'i GitHub'dan indir
# Seçenek B: Host'tan kopyalanan .deb'i kullan → aşağıdaki COPY satırını aç,
#            build öncesi: cp /usr/bin/hailort*.deb <repo>/hailo_debs/
# ---------------------------------------------------------------------------
RUN set -eux; \
    BASE_URL="https://github.com/hailo-ai/hailort/releases/download/v${HAILO_VERSION}"; \
    wget -q "${BASE_URL}/hailort_${HAILO_VERSION}_${TARGETARCH}.deb" \
         -O /tmp/hailort.deb; \
    wget -q "${BASE_URL}/hailort-pcie-driver_${HAILO_VERSION}_${TARGETARCH}.deb" \
         -O /tmp/hailort-driver.deb 2>/dev/null || true; \
    dpkg -i /tmp/hailort.deb; \
    rm -f /tmp/hailort.deb /tmp/hailort-driver.deb

# hailo_platform Python bağlantısı (hailort .deb ile birlikte gelir;
# gelmiyorsa ayrıca pip install ile kur)
RUN pip3 install --no-cache-dir --break-system-packages hailo_platform==${HAILO_VERSION} \
    2>/dev/null || echo "hailo_platform pip paketi bulunamadı; .deb'den kurulu olan kullanılacak"

# Hailo cihaz grubunu ekle (host'ta genellikle 'hailo' veya 'video' grubu altındadır)
RUN groupadd -f hailo && usermod -aG hailo root 2>/dev/null || true

# Set environment for ROS - suppress missing package warnings
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash 2>/dev/null; fi' >> /etc/bash.bashrc

WORKDIR /ros2_ws

CMD ["bash"]
