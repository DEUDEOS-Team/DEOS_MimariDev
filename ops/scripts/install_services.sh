#!/usr/bin/env bash
# DEOS systemd servislerini kurar.
# Kullanım: sudo bash ops/scripts/install_services.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CURRENT_USER="${SUDO_USER:-$USER}"
SYSTEMD_SRC="$REPO_ROOT/ops/systemd"

echo "[deos-install] Repo    : $REPO_ROOT"
echo "[deos-install] Kullanıcı: $CURRENT_USER"

# Docker grubuna ekle
if ! id -nG "$CURRENT_USER" | grep -qw docker; then
    usermod -aG docker "$CURRENT_USER"
    echo "[deos-install] '$CURRENT_USER' docker grubuna eklendi (oturumu yeniden aç)"
fi

install_service() {
    local name="$1"
    sed \
        -e "s|__USER__|$CURRENT_USER|g" \
        -e "s|__REPO_ROOT__|$REPO_ROOT|g" \
        "$SYSTEMD_SRC/$name" > "/etc/systemd/system/$name"
    echo "[deos-install] /etc/systemd/system/$name yazıldı"
}

install_service deos-log-monitor.service
install_service deos-compose.service

systemctl daemon-reload
systemctl enable deos-log-monitor.service deos-compose.service

echo ""
echo "[deos-install] Kurulum tamamlandı. Hemen başlatmak için:"
echo "  sudo systemctl start deos-log-monitor"
echo "  sudo systemctl start deos-compose"
