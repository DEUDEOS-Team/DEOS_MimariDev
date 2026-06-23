#!/usr/bin/env bash
# DEOS konteynerini Hailo-8 erişimiyle başlat.
# Kullanım:
#   ./run_deos.sh                    → interaktif bash
#   ./run_deos.sh ros2 launch ...    → doğrudan komut

set -euo pipefail

IMAGE="${DEOS_IMAGE:-deos_ros2:latest}"

# --- Hailo cihaz kontrolü ---
if [[ ! -c /dev/hailo0 ]]; then
    echo "[UYARI] /dev/hailo0 bulunamadı. Hailo-8 bağlı ve sürücü yüklü mü?" >&2
fi

# Hosttaki hailo grubu varsa container'a ekle (cihaz izni için)
HAILO_GROUP_ID=""
if getent group hailo &>/dev/null; then
    HAILO_GROUP_ID=$(getent group hailo | cut -d: -f3)
elif getent group video &>/dev/null; then
    HAILO_GROUP_ID=$(getent group video | cut -d: -f3)
fi

GROUP_ARGS=()
if [[ -n "$HAILO_GROUP_ID" ]]; then
    GROUP_ARGS+=(--group-add "$HAILO_GROUP_ID")
fi

# Hailo PCIe cihaz dosyaları (/dev/hailo0 zorunlu, diğerleri opsiyonel)
DEVICE_ARGS=(--device=/dev/hailo0:/dev/hailo0)
for dev in /dev/hailo_control /dev/hailo_board_test; do
    [[ -c "$dev" ]] && DEVICE_ARGS+=(--device="${dev}:${dev}")
done

exec docker run -it --rm \
    "${DEVICE_ARGS[@]}" \
    "${GROUP_ARGS[@]}" \
    \
    `# Hailo IPC için paylaşılan bellek` \
    -v /dev/shm:/dev/shm \
    \
    `# ROS2 DDS discovery (host ağıyla aynı namespace)` \
    --network host \
    \
    `# Model dosyaları host'tan mount (isteğe bağlı; build içine gömülüyse kaldır)` \
    -v /home/pi/deos_models:/ros2_ws/models:ro \
    \
    `# Hailo userspace runtime kütüphanelerini host'tan bind-mount` \
    `# (container'daki sürüm host ile eşleşmiyorsa bu satır version mismatch hatasını çözer)` \
    -v /usr/lib/libhailort.so:/usr/lib/libhailort.so:ro \
    \
    "$IMAGE" \
    "${@:-bash}"
