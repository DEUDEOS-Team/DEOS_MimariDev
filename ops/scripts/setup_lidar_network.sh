#!/bin/bash
# SICK Multiscan165 için Pi ethernet arayüzünde statik IP ayarlar.
# LiDAR'ın fabrika IP'si: 192.168.0.1
# Pi'nin bu ağda ulaşılabilir olması için: 192.168.0.10 (veya başka .x adresi)
#
# Kullanım:
#   sudo bash ops/scripts/setup_lidar_network.sh
#   sudo bash ops/scripts/setup_lidar_network.sh eth0 192.168.0.10  # özel arayüz/IP

IFACE="${1:-eth0}"
PI_IP="${2:-192.168.0.10}"
LIDAR_IP="192.168.0.1"
PREFIX="24"

echo ">>> LiDAR ağ ayarı: $IFACE → $PI_IP/$PREFIX (LiDAR: $LIDAR_IP)"

# Mevcut adresi temizle ve yeni adresi ata
ip addr flush dev "$IFACE" 2>/dev/null || true
ip addr add "$PI_IP/$PREFIX" dev "$IFACE"
ip link set "$IFACE" up

# Bağlantıyı doğrula
echo ">>> Ping testi: $LIDAR_IP"
if ping -c 2 -W 2 "$LIDAR_IP" &>/dev/null; then
    echo "OK: LiDAR ($LIDAR_IP) ulaşılabilir."
else
    echo "UYARI: LiDAR ping yanıt vermedi. Kablo bağlantısını ve LiDAR IP'sini kontrol et."
fi
