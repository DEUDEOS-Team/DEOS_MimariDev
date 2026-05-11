#include "udp_transport.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "ethernetif.h"
#include <string.h>
#include "lwip/timeouts.h"

/* main.c içindeki LwIP değişkenlerini buraya çağırıyoruz */
extern struct netif gnetif;
extern uint32_t HAL_GetTick(void);

static struct udp_pcb *uros_upcb = NULL;
static struct pbuf *rx_pbuf = NULL;
static bool data_received = false;

/* --- UDP Veri Geldiğinde Tetiklenen Kesme (Callback) --- */
static void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        /* Eğer okunmamış eski bir paket varsa onu silip yenisini al */
        if (rx_pbuf != NULL) {
            pbuf_free(rx_pbuf);
        }
        rx_pbuf = p;
        data_received = true;
    }
}

/* 1. BAĞLANTIYI AÇ */
bool stm32_udp_transport_open(struct uxrCustomTransport * transport) {
    ip_addr_t agent_ip;

    /* PC'nin (Agent) IP Adresi: 192.168.1.10 */
    IP4_ADDR(&agent_ip, 192, 168, 1, 10);

    uros_upcb = udp_new();
    if (uros_upcb == NULL) return false;

    /* Herhangi bir yerel porttan bağlan ve PC'nin 8888 portunu hedefle */
    udp_bind(uros_upcb, IP_ANY_TYPE, 8888);
    udp_connect(uros_upcb, &agent_ip, 8888);

    /* Veri gelirse udp_receive_callback fonksiyonunu çalıştır */
    udp_recv(uros_upcb, udp_receive_callback, NULL);
    return true;
}

/* 2. BAĞLANTIYI KAPAT */
bool stm32_udp_transport_close(struct uxrCustomTransport * transport) {
    if (uros_upcb != NULL) {
        udp_disconnect(uros_upcb);
        udp_remove(uros_upcb);
        uros_upcb = NULL;
    }
    if (rx_pbuf != NULL) {
        pbuf_free(rx_pbuf);
        rx_pbuf = NULL;
    }
    return true;
}

/* 3. AĞA VERİ YAZ (GÖNDER) */
size_t stm32_udp_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) return 0;

    pbuf_take(p, buf, len);
    err_t lwip_err = udp_send(uros_upcb, p);
    pbuf_free(p);

    return (lwip_err == ERR_OK) ? len : 0;
}

/* 4. AĞDAN VERİ OKU (DİNLE) */
size_t stm32_udp_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    uint32_t start_time = HAL_GetTick();

    /* Timeout süresi dolana kadar bekle */
    while (!data_received) {

        /* BARE-METAL LwIP İÇİN KRİTİK NOKTA: İşletim sistemi olmadığı için
           ağı sürekli manuel olarak yenilememiz (poll) gerekiyor! */
        ethernetif_input(&gnetif);
        sys_check_timeouts();

        if ((HAL_GetTick() - start_time) >= timeout) {
            *err = 1; // Timeout hatası
            return 0;
        }
    }

    /* Veri geldiyse buffer'a kopyala */
    size_t bytes_to_copy = 0;
    if (rx_pbuf != NULL) {
        bytes_to_copy = (rx_pbuf->tot_len > len) ? len : rx_pbuf->tot_len;
        pbuf_copy_partial(rx_pbuf, buf, bytes_to_copy, 0);
        pbuf_free(rx_pbuf);
        rx_pbuf = NULL;
    }

    data_received = false;
    return bytes_to_copy;
}