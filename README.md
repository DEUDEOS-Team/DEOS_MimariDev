# DEOS_Mimari

## STM32 + Ethernet UDP + micro-ROS

Bu repoda ROS2 servisi zaten `host` ağ modunda çalışıyor. STM32 ile Ethernet üzerinden `UDP + micro-ROS` haberleşmesi için en pratik kurulum:

1. `micro-ROS Agent` ayrı bir container olarak çalışsın.
2. `deos` ve `micro_ros_agent` aynı `ROS_DOMAIN_ID` değerini kullansın.
3. İki container da Raspberry Pi üzerinde `network_mode: host` ile çalışsın.
4. STM32, Raspberry Pi'nin Ethernet IP adresine `UDP 8888` üzerinden bağlansın.

Eklenen `docker-compose.yml` servisi bunu sağlar:

- `deos`: ana ROS2 stack
- `micro_ros_agent`: `microros/micro-ros-agent:jazzy` image'i ile `udp4 -p 8888`

### Veri akışı

`STM32 (micro-ROS client)` -> `UDP/8888` -> `micro_ros_agent container` -> `ROS 2 DDS graph` -> `deos container`

### Çalıştırma

```bash
docker compose up -d micro_ros_agent deos
```

### STM32 tarafı

STM32 firmware içinde micro-ROS UDP transport hedefi olarak:

- Agent IP: Raspberry Pi'nin Ethernet IP'si
- Agent Port: `8888`

Örnek:

- Raspberry Pi IP: `192.168.0.100`
- STM32 agent hedefi: `192.168.0.100:8888`

### Kontrol

`micro_ros_agent` loglarında istemci oturumu görülmelidir. Ardından `deos` container içinde:

```bash
ros2 topic list
ros2 node list
```

ile STM32'nin yayınladığı topic/node'lar görünmelidir.

### Not

Bu yapıda iki container Docker bridge üzerinden değil, doğrudan host ağ yığını üzerinden haberleşir. Raspberry Pi üzerinde Ethernet tabanlı sensörler ve `micro-ROS UDP` için bu yaklaşım en sorunsuz seçenektir.
