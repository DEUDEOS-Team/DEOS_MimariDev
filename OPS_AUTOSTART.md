# RasPi güç verildiğinde otomatik bringup + health

Bu doküman, Raspberry Pi’ye güç verildiğinde sistemin otomatik kalkması ve periyodik sağlık kontrolü yapması için gerekli yapıyı kurar.

## Neler eklendi?

- **systemd unit’leri**:
  - `ops/systemd/deos-bringup.service`: docker compose stack’i başlatır
  - `ops/systemd/deos-health.service`: 10 sn’de bir docker/ROS graph kontrolü yapar, gerekirse bringup’ı restart eder
- **compose override**:
  - `ops/compose/docker-compose.autostart.yml`: `deos` servisini otomatik `colcon build` + `ros2 launch` çalıştıracak şekilde override eder
- **scriptler**:
  - `ops/scripts/deos_compose_up.sh`: compose’u override ile foreground çalıştırır
  - `ops/scripts/health_monitor.py`: health döngüsü (JSON log yazar)

## Kurulum (Raspberry Pi)

> Varsayım: repo cihazda `/opt/deos` altında duruyor ve docker + docker compose yüklü.

1) Dosyaları doğru yere koy:

- Repo: `/opt/deos`
- Unit’ler: `/etc/systemd/system/`

```bash
sudo mkdir -p /opt/deos
# repo zaten /opt/deos ise geç

sudo cp /opt/deos/ops/systemd/deos-bringup.service /etc/systemd/system/deos-bringup.service
sudo cp /opt/deos/ops/systemd/deos-health.service  /etc/systemd/system/deos-health.service
sudo chmod +x /opt/deos/ops/scripts/deos_compose_up.sh
sudo chmod +x /opt/deos/ops/scripts/health_monitor.py
```

2) systemd reload + enable:

```bash
sudo systemctl daemon-reload
sudo systemctl enable deos-bringup.service
sudo systemctl enable deos-health.service
```

3) İlk çalıştırma:

```bash
sudo systemctl start deos-bringup.service
sudo systemctl start deos-health.service
```

## İzleme / debug

```bash
sudo journalctl -u deos-bringup -f
sudo journalctl -u deos-health -f

docker ps
docker compose -f /opt/deos/docker-compose.yml logs -f micro_ros_agent
```

`deos-health` JSON formatında satır log basar (journald’a).

## Yarışma günü notları (offline)

- Bu yapı **`docker compose pull` kullanmaz**; imajların cihazda hazır olması gerekir.
- `ops/compose/docker-compose.autostart.yml` içinde `install/` yoksa `colcon build` yapılır.
  - Daha hızlı boot için yarışma öncesi bir kez build alıp `install/` oluşturmanız önerilir.

## Yapılandırma

`deos-health.service` içinde env ile ayarlanabilir:

- `DEOS_HEALTH_INTERVAL_S` (default 10)
- `DEOS_HEALTH_FAIL_THRESHOLD` (default 3)

STM32 subscriber kontrolü şu an **rapor amaçlı** (default `require_stm32_subscriber=False`); STM32 yarışma senaryosunda kesin şart ise `ops/scripts/health_monitor.py` içinden açılabilir.

