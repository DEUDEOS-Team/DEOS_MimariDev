#!/usr/bin/env bash
# DEOS - CycloneDDS için host (Raspberry Pi) kernel ağ tamponları.
# Container network_mode: host kullandığı için bu ayar HOST'ta yapılmalıdır.
#
#   sudo bash ops/scripts/setup_dds_sysctl.sh
#
# config/dds/cyclonedds*.xml içindeki SocketReceiveBufferSize max="16MB"
# değerinin gerçekten uygulanabilmesi için rmem_max >= 16MB olmalıdır.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Bu script root olarak çalıştırılmalı: sudo bash $0" >&2
    exit 1
fi

CONF=/etc/sysctl.d/60-deos-dds.conf
cat >"$CONF" <<'EOF'
# DEOS CycloneDDS: büyük PointCloud2 / Image mesajları için UDP alım tamponu
net.core.rmem_max=33554432
net.core.rmem_default=4194304
# IP fragment yeniden birleştirme belleği (64 KB'lık RTPS parçaları)
net.ipv4.ipfrag_high_thresh=134217728
net.ipv4.ipfrag_time=3
EOF

sysctl --system >/dev/null
echo "[deos] $CONF yazıldı ve uygulandı:"
sysctl net.core.rmem_max net.core.rmem_default net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_time
