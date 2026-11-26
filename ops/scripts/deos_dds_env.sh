#!/usr/bin/env bash
# DEOS DDS ortamı: RMW seçimi + (isteğe bağlı) iceoryx RouDi başlatma.
#
# Bu dosya SOURCE edilir (çalıştırılmaz):
#     source /ros2_ws/ops/scripts/deos_dds_env.sh
#
# Değişkenler (docker-compose.yml'den gelir, hepsinin varsayılanı var):
#   RMW_IMPLEMENTATION   rmw_cyclonedds_cpp (varsayılan)
#   DEOS_DDS_SHM         0 = sadece UDP/loopback, 1 = iceoryx shared memory
#   DEOS_DDS_CONFIG_DIR  /ros2_ws/config/dds
#
# DEOS_DDS_SHM=1 iken RouDi başlatılamazsa SHM'siz yapılandırmaya düşülür
# (araç yine de ayağa kalkar, log'a WARNING düşer).

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
DEOS_DDS_SHM="${DEOS_DDS_SHM:-0}"
DEOS_DDS_CONFIG_DIR="${DEOS_DDS_CONFIG_DIR:-/ros2_ws/config/dds}"
DEOS_ROUDI_LOG="${DEOS_ROUDI_LOG:-/ros2_ws/logs/roudi.log}"

_deos_dds_log() { printf '[deos-dds] %s\n' "$*"; }

_deos_roudi_running() {
    pgrep -x iox-roudi >/dev/null 2>&1
}

_deos_start_roudi() {
    if _deos_roudi_running; then
        _deos_dds_log "iox-roudi zaten çalışıyor"
        return 0
    fi
    if ! command -v iox-roudi >/dev/null 2>&1; then
        _deos_dds_log "WARNING: iox-roudi bulunamadı (ros-jazzy-iceoryx-posh kurulu mu?)"
        return 1
    fi
    mkdir -p "$(dirname "$DEOS_ROUDI_LOG")"
    _deos_dds_log "iox-roudi başlatılıyor (config: $DEOS_DDS_CONFIG_DIR/roudi.toml)"
    nohup iox-roudi -c "$DEOS_DDS_CONFIG_DIR/roudi.toml" >"$DEOS_ROUDI_LOG" 2>&1 &
    # RouDi hazır olana kadar bekle (en fazla ~5 s)
    for _ in $(seq 1 50); do
        if _deos_roudi_running && grep -q "RouDi is ready for clients" "$DEOS_ROUDI_LOG" 2>/dev/null; then
            _deos_dds_log "iox-roudi hazır"
            return 0
        fi
        sleep 0.1
    done
    _deos_dds_log "WARNING: iox-roudi 5 s içinde hazır olmadı, log: $DEOS_ROUDI_LOG"
    return 1
}

if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
    if [ "$DEOS_DDS_SHM" = "1" ] && _deos_start_roudi; then
        export CYCLONEDDS_URI="file://$DEOS_DDS_CONFIG_DIR/cyclonedds_shm.xml"
    else
        [ "$DEOS_DDS_SHM" = "1" ] && _deos_dds_log "WARNING: SHM devre dışı, UDP yapılandırmasına düşülüyor"
        export CYCLONEDDS_URI="file://$DEOS_DDS_CONFIG_DIR/cyclonedds.xml"
    fi
    _deos_dds_log "RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI"
else
    _deos_dds_log "RMW=$RMW_IMPLEMENTATION (CycloneDDS dışı, CYCLONEDDS_URI kullanılmıyor)"
fi
