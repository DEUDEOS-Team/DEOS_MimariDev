#!/usr/bin/env bash
# DEOS Sistem Başlatma Kontrolü
# Docker container içinde çalışır; çıktı web log monitöründe görünür.

LOG_DIR="/ros2_ws/logs/startup"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/$(date +%Y-%m-%d_%H-%M-%S).log"

PASS=0
FAIL=0

log() {
    local level="$1"
    local msg="$2"
    local ts
    ts=$(date '+%Y-%m-%d %H:%M:%S.000')
    printf '%s | %-8s | %s\n' "$ts" "$level" "$msg" | tee -a "$LOG_FILE"
}

check_ok() {
    log "INFO" "✓ $1"
    ((PASS++)) || true
}

check_fail() {
    log "ERROR" "✗ $1"
    ((FAIL++)) || true
}

check_warn() {
    log "WARNING" "! $1"
    ((FAIL++)) || true
}

log "INFO" "========================================="
log "INFO" "DEOS Sistem Başlatma Kontrolü"
log "INFO" "Tarih: $(date '+%Y-%m-%d %H:%M:%S %Z')"
log "INFO" "========================================="

# --- Donanım Kontrolleri ---
log "INFO" "--- Donanım ---"

if [ -e /dev/hailo0 ]; then
    check_ok "Hailo AI HAT (/dev/hailo0)"
else
    check_fail "Hailo AI HAT bulunamadı (/dev/hailo0 yok)"
fi

if ls /dev/bus/usb 2>/dev/null | grep -q .; then
    check_ok "USB bus erişilebilir — RealSense D415 kontrol edilebilir"
else
    check_fail "USB bus erişilemiyor — RealSense D415 bağlı olmayabilir"
fi

if [ -e /dev/ttyUSB0 ]; then
    check_ok "GPS modülü (/dev/ttyUSB0)"
else
    check_fail "GPS modülü bulunamadı (/dev/ttyUSB0 yok)"
fi

if [ -e /dev/ttyUSB1 ]; then
    check_ok "IMU (/dev/ttyUSB1)"
else
    check_warn "IMU bulunamadı (/dev/ttyUSB1 yok) — opsiyonel"
fi

# --- ROS Workspace ---
log "INFO" "--- ROS Workspace ---"

if [ -f /ros2_ws/install/setup.bash ]; then
    check_ok "ROS workspace derlenmiş (/ros2_ws/install/setup.bash)"
else
    check_fail "ROS workspace derlenmemiş — colcon build gerekli"
fi

LAUNCH_PKG=$(find /ros2_ws/install -name "main.launch.py" 2>/dev/null | head -1)
if [ -n "$LAUNCH_PKG" ]; then
    check_ok "vehicle_bringup launch dosyası bulundu"
else
    check_fail "main.launch.py bulunamadı — vehicle_bringup paketi eksik olabilir"
fi

# --- Sistem Kaynakları ---
log "INFO" "--- Sistem Kaynakları ---"

DISK_USAGE=$(df /ros2_ws --output=pcent 2>/dev/null | tail -1 | tr -d ' %')
if [ -n "$DISK_USAGE" ] && [ "$DISK_USAGE" -lt 85 ]; then
    check_ok "Disk: %${DISK_USAGE} kullanımda"
elif [ -n "$DISK_USAGE" ] && [ "$DISK_USAGE" -lt 95 ]; then
    check_warn "Disk: %${DISK_USAGE} kullanımda (yüksek)"
else
    check_fail "Disk: %${DISK_USAGE} kullanımda (kritik)"
fi

RAM_FREE_MB=$(awk '/MemAvailable/ {printf "%d", $2/1024}' /proc/meminfo)
RAM_TOTAL_MB=$(awk '/MemTotal/ {printf "%d", $2/1024}' /proc/meminfo)
if [ "$RAM_FREE_MB" -gt 512 ]; then
    check_ok "RAM: ${RAM_FREE_MB}MB boş / ${RAM_TOTAL_MB}MB toplam"
else
    check_warn "RAM: Yalnızca ${RAM_FREE_MB}MB boş / ${RAM_TOTAL_MB}MB"
fi

# --- Model Dosyaları ---
log "INFO" "--- Model Dosyaları ---"

if [ -f /ros2_ws/models/detection.hef ]; then
    check_ok "Hailo detection modeli (detection.hef)"
else
    check_warn "detection.hef bulunamadı — nesne tespiti çalışmayabilir"
fi

if [ -f /ros2_ws/models/lane_seg.hef ]; then
    check_ok "Şerit segmentasyon modeli (lane_seg.hef)"
else
    check_warn "lane_seg.hef bulunamadı — şerit tespiti çalışmayabilir"
fi

# --- Sonuç ---
log "INFO" "-----------------------------------------"
if [ "$FAIL" -eq 0 ]; then
    log "INFO"    "SONUÇ: Tüm kontroller geçti ($PASS/$((PASS+FAIL))) — sistem tam hazır"
else
    log "WARNING" "SONUÇ: $PASS geçti, $FAIL başarısız — sistem kısıtlı modda başlıyor"
fi
log "INFO" "========================================="
