/*
 * Verbesserte Version des Mehrkanal‑Audio‑Streamers für die ESP32‑Plattform.
 *
 * Dieses Beispiel liest sechs 16‑Bit‑Kanäle im TDM‑Modus über die I²S‑Peripherie
 * des ESP32 und überträgt die Daten per UDP an einen externen Empfänger.
 * Gegenüber dem einfachen Beispiel wurden einige Optimierungen vorgenommen:
 *
 *  - Der Ringpuffer verwendet eine konsistente Blockgröße (ein DMA‑Block).
 *  - Jedes UDP‑Paket enthält einen kleinen Header mit Sequenznummer und
 *    Zeitstempel, sodass der Empfänger Paketverlust oder falsche Reihenfolge
 *    erkennen und ggf. kompensieren kann.
 *  - Die UDP‑Paketgröße orientiert sich an der typischen MTU, um Overhead zu
 *    verringern, aber Fragmentierung zu vermeiden.
 *  - Kommentare und Konstanten wurden bereinigt, um die Konfiguration
 *    nachvollziehbar zu machen.
 *
 * Hinweis: Dies ist nur die Sender‑Seite. Für eine robuste Übertragung muss
 * der Empfänger einen Jitter‑Puffer implementieren und auf Basis der
 * Sequenznummern/ Zeitstempel verlorene Pakete erkennen und ggf. auffüllen.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/i2s_tdm.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ------------------------ Ringbuffer / I2S ------------------------ */
/*
 * Ein DMA-Block besteht aus einer festen Anzahl von TDM-Frames. Jeder Frame
 * enthaelt 12 Bytes (6 Kanaele * 2 Bytes). Um einen kompletten Block zu
 * verarbeiten, legen wir die Blockgroeße auf 144 Bytes fest (12 Frames). Der
 * Ringpuffer besteht aus mehreren dieser Bloecke. Puffergroeße und Anzahl
 * koennen nach Bedarf angepasst werden (groeßerer Puffer = mehr Jitter-Reserve).
 */
#define TDM_FRAME_BYTES   12U         // 6 Kanaele * 16 Bit pro Sample
#define DMA_BLOCK_FRAMES  12U         // 12 Frames pro DMA-Block
#define DMA_BLOCK_BYTES   (TDM_FRAME_BYTES * DMA_BLOCK_FRAMES) // 144 Byte
#define RB_NUM_BLOCKS     64U         // 64 Bloecke = 64*144 B ca. 9 kB Puffer
static RingbufHandle_t audio_rb;

/* -------------------------- I/O-Pins ----------------------------- */
#define PIN_LED      2
#define I2S_NUM         I2S_NUM_0
#define I2S_CLK      5      // Bit Clock (SCK)
#define I2S_WS       4      // Word Select (WS)
#define I2S_SD       12     // Serial Data (SD)

/* -------------------------- WLAN / UDP --------------------------- */
static const char *TAG = "UDP_unicast";
static const char *WIFI_SSID = "iPhone von Leonard ";
static const char *WIFI_PASS = "12341234";
static const char *HOST_IP_ADDR = "172.20.10.2";
#define DEST_PORT    7000

/* UDP‑Payload‑Größen
 * Die maximale Größe eines Ethernet‑Frames ohne Fragmentierung beträgt
 * üblicherweise 1500 Bytes. Abzüglich 20 Byte IPv4‑Header und 8 Byte
 * UDP‑Header bleiben 1472 Byte für Nutzdaten. Wir verwenden einen kleinen
 * eigenen Header (8 Byte), sodass 1464 Byte Audio pro Paket übertragen
 * werden. 1464 / 12 Bytes pro TDM‑Frame = 122 vollständige Frames pro Paket.
 */
#define AUDIO_PAYLOAD_SIZE 1464U     // Bytes reiner Audiodaten pro UDP‑Paket
#define PKT_HEADER_SIZE     8U       // Sequenznummer (4 B) + Zeitstempel (4 B)
#define UDP_PAYLOAD_SIZE   (PKT_HEADER_SIZE + AUDIO_PAYLOAD_SIZE)

// Event‑Gruppe für Verbindungsstatus
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
static const char *TAG_WIFI = "wifi_init";
static i2s_chan_handle_t rx_handle;

/* --------------------- LED helper --------------------- */
static inline void led_on (void) { gpio_set_level(PIN_LED, 1); }
static inline void led_off(void) { gpio_set_level(PIN_LED, 0); }

/* ----------------- Struktur des Paket-Headers ----------------- */
typedef struct __attribute__((packed)) {
    uint32_t seq;    // fortlaufende Sequenznummer (Big‑Endian)
    uint32_t ts;     // Zeitstempel in Mikrosekunden (Big‑Endian)
} audio_pkt_header_t;

/* ------------------------- I/O‑Initialisierung ------------------------- */
static void io_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
}

/* ---------------------- TDM / I2S‑Initialisierung ---------------------- */
void i2s_init_tdm(void)
{
    // Kanal konfigurieren
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    // TDM-Konfiguration für 6 Mikrofone (16 Bit)
    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_MONO,
            I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 |
            I2S_TDM_SLOT3 | I2S_TDM_SLOT4 | I2S_TDM_SLOT5
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_CLK,
            .ws   = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(rx_handle, &tdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

/* -------------------- I2S-Lesetask -------------------------------------- */
static void i2s_read_task(void *arg)
{
    // DMA‑Puffer für Rohdaten (ein Block = 144 Bytes)
    uint8_t dma_buf[DMA_BLOCK_BYTES];
    size_t bytes_read;

    while (true) {
        // Lese genau einen DMA‑Block (es darf kein Timeout auftreten)
        esp_err_t res = i2s_channel_read(arg, dma_buf, sizeof(dma_buf), &bytes_read, portMAX_DELAY);
        if (res != ESP_OK || bytes_read != sizeof(dma_buf)) {
            ESP_LOGW(TAG, "i2s_channel_read Fehlerrueckgabewert %d, bytes_read=%d", res, (int)bytes_read);
            continue;
        }

        // frames = Anzahl der kompletten TDM‑Frames im Block
        int frames = bytes_read / TDM_FRAME_BYTES;

        // Temporärer Puffer für die gesplitteten Samples (Big‑Endian)
        uint8_t pcm_buf[DMA_BLOCK_BYTES];

        // Zeiger setzen
        uint8_t *r = dma_buf;
        uint8_t *w = pcm_buf;

        for (int i = 0; i < frames; i++) {
            for (int channel = 0; channel < 6; channel++) {
                // Lese 16‑Bit‑Sample (MSB, LSB)
                uint16_t sample = ((uint16_t)r[0] << 8) | r[1];

                // Momentan ohne Verstärkung
                float gain = 1.0f;
                int16_t sample_signed = (int16_t)sample;
                sample_signed = (int16_t)(sample_signed * gain);

                // Schreibe Big‑Endian in PCM‑Puffer
                *w++ = (sample_signed >> 8) & 0xFF;
                *w++ = sample_signed & 0xFF;

                r += 2; // zum nächsten Kanal
            }
        }

        // Daten in Ringbuffer kopieren
        if (xRingbufferSend(audio_rb, pcm_buf, DMA_BLOCK_BYTES, portMAX_DELAY) != pdTRUE) {
            ESP_LOGW(TAG, "Ringbuffer voll, Datenverlust!");
        }
    }
}

/* --------------------- Wi‑Fi Event‑Handler --------------------- */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "Disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ---------------------- Wi-Fi Initialisierung --------------------- */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        }
    };
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Waiting for IP...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG_WIFI, "Connected!");
}

/* ---------------------- UDP-Stream-Task ---------------------- */
static void udp_stream_task(void *arg)
{
    // Sequenznummer (wird bei jedem Paket erhöht)
    uint32_t sequence_number = 0;
    // LED-Blinksteuerung
    uint32_t pkt_cnt = 0;

    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket fail");
        vTaskDelete(NULL);
        return;
    }

    // Paketpuffer: Header (8 Byte) + Audio (1464 Byte)
    uint8_t pkt[UDP_PAYLOAD_SIZE];

    while (true) {
        size_t grabbed = 0;
        // Fülle die Audio-Nutzlast (1464 Byte) aus dem Ringbuffer
        while (grabbed < AUDIO_PAYLOAD_SIZE) {
            size_t got;
            uint8_t *p = xRingbufferReceiveUpTo(audio_rb, &got,
                                                portMAX_DELAY,
                                                AUDIO_PAYLOAD_SIZE - grabbed);
            if (p == NULL) {
                ESP_LOGW(TAG, "Ringbuffer leer - Warte auf Daten");
                grabbed = 0;
                continue; // Erneut versuchen
            }
            memcpy(pkt + PKT_HEADER_SIZE + grabbed, p, got);
            grabbed += got;
            vRingbufferReturnItem(audio_rb, p);
        }

        // Header füllen
        audio_pkt_header_t hdr;
        hdr.seq = htonl(sequence_number++);
        hdr.ts  = htonl((uint32_t)(esp_timer_get_time() & 0xFFFFFFFF));
        memcpy(pkt, &hdr, sizeof(hdr));

        // UDP‑Paket versenden, falls der Socket bereit ist
        ssize_t sent = sendto(sock, pkt, sizeof(pkt), 0,
                              (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            int err = errno;
            ESP_LOGE(TAG, "sendto failed: errno=%d", err);
            // Bei vollem Puffer kurz warten
            if (err == ENOMEM) {
                vTaskDelay(pdMS_TO_TICKS(5));
            } else {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        } else {
            // LED‑Heartbeat: blinkt alle ~50 Pakete
            if (++pkt_cnt >= 50) {
                pkt_cnt = 0;
                static bool led_state = false;
                led_state = !led_state;
                gpio_set_level(PIN_LED, led_state);
            }
        }
    }
}

/* ------------------------- app_main() ------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    io_init();
    // Erzeuge Ringbuffer mit definierter Blockgröße
    audio_rb = xRingbufferCreate(DMA_BLOCK_BYTES * RB_NUM_BLOCKS, RINGBUF_TYPE_BYTEBUF);
    if (audio_rb == NULL) {
        ESP_LOGE(TAG, "Ringbuffer konnte nicht erstellt werden");
        return;
    }

    i2s_init_tdm();
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(100)); // kleine Startpause

    xTaskCreatePinnedToCore(i2s_read_task, "i2s_read_task",
                        4096, rx_handle, 22, NULL, 1);
    xTaskCreate(udp_stream_task, "udp_stream_task",
                4096, NULL, 10, NULL);
}