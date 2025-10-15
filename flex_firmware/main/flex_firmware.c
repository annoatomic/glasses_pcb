// SPDX-License-Identifier: MIT
// Refactored ESP32-S3: ADAU7118 (TDM 8x16) -> I2S RX -> Ringbuffer -> UDP 2ch Stream


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/i2s_tdm.h"
#include "driver/i2c_master.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ─────────────────────────── User Config (menuconfig) ───────────────────────────
 * Set these in: menuconfig -> Example Configuration
 */
#define CONFIG_WIFI_SSID         CONFIG_EXAMPLE_WIFI_SSID
#define CONFIG_WIFI_PASSWORD     CONFIG_EXAMPLE_WIFI_PASSWORD
#define CONFIG_UDP_DEST_IP       CONFIG_EXAMPLE_UDP_DEST_IP
#define CONFIG_UDP_DEST_PORT     CONFIG_EXAMPLE_UDP_DEST_PORT

/* ─────────────────────────── Logging Tags ─────────────────────────── */
static const char *TAG      = "AudioUDP";
static const char *TAG_WIFI = "WiFi";

/* ─────────────────────────── Hardware Pins (ESP32-S3) ─────────────────────────── */
#define I2C_PORT          I2C_NUM_0
#define I2C_SDA_GPIO      7
#define I2C_SCL_GPIO      6
#define I2C_FREQ_HZ       100000
#define ADAU7118_ADDR     0x14  // 7-bit

#define I2S_PIN_BCLK      5
#define I2S_PIN_WS        4
#define I2S_PIN_DIN       12

/* ─────────────────────────── Audio/TDM Params ─────────────────────────── */
#define SAMPLE_RATE_HZ        48000
#define TDM_TOTAL_SLOTS       8
#define TDM_SLOT_BITS         16
#define TDM_ACTIVE_MASK       ( I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 | \
                                 I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7 )
#define BYTES_PER_SAMPLE      (TDM_SLOT_BITS / 8)
#define AUDIO_FRAME_BYTES     (TDM_TOTAL_SLOTS * BYTES_PER_SAMPLE)

/* ───────── Mehrkanal-Streaming ───────── */
#define TX_NUM_CHANNELS     6  // 6 für C0..C5; auf 8 setzen, wenn Slots 6/7 aktiv
static const uint8_t TX_SLOT_IDX[TX_NUM_CHANNELS] = {0,1,2,3,4,5}; // für 8ch: {0,1,2,3,4,5,6,7}

/* UDP-Payloadgröße steuern (unter MTU bleiben) */
#define FRAMES_PER_UDP      96 


/* ─────────────────────────── Ringbuffer / Chunking ───────────────────────────
 * CHUNK_FRAMES * AUDIO_FRAME_BYTES = 4096 B => gute UDP-Batches ohne Fragmentierung
 * Struktur der Audiodaten:
 *   - 8 TDM-Slots à 16 Bit = 8 × 2 B = 16 B pro Frame
 *   - 48 000 Frames/s bei 16 Bit → 768 kB/s Eingangsrate
 *
 * Wir lesen CHUNK_FRAMES Frames pro Durchgang vom I2S:
 *   CHUNK_FRAMES (256) × AUDIO_FRAME_BYTES (16 B) = 4096 B pro Chunk
 *
 * Diese 4096 B bilden also EIN Element im Ringbuffer.
 * Der Stream-Task entnimmt daraus die TX_NUM_CHANNELS genutzten Slots (z. B. 6 Kanäle):
 * TX_NUM_CHANNELS × 2 B × FRAMES_PER_UDP = bei 6ch und FRAMES_PER_UDP=96 → 1152 B UDP-Payload.
 *
 * Damit bleibt das UDP-Paket deutlich unter der typischen MTU (1500 B),
 * → keine IP-Fragmentierung, stabiler Stream.
 *
 * Gesamt-Ringbuffergröße:
 *   RB_NUM_CHUNKS (24) × RB_CHUNK_BYTES (4096 B) ≈ 96 kB,
 *   das ergibt genug Puffer für Netzwerk- und I2S-Latenz.
 */
#define CHUNK_FRAMES          256U
#define RB_CHUNK_BYTES        (AUDIO_FRAME_BYTES * CHUNK_FRAMES) // 4096 B
#define RB_NUM_CHUNKS         24U                                // ≈96 kB
_Static_assert((RB_CHUNK_BYTES % AUDIO_FRAME_BYTES) == 0, "Chunk must be frame-aligned");
_Static_assert(TDM_SLOT_BITS == 16, "Dieses Setup erwartet 16 Bit pro Slot");



/* ─────────────────────────── Globals ─────────────────────────── */
static RingbufHandle_t   g_audio_rb;
static i2s_chan_handle_t g_i2s_rx;
static EventGroupHandle_t g_wifi_evt_group;
#define WIFI_CONNECTED_BIT BIT0

/* I2C bus/dev */
static i2c_master_bus_handle_t g_i2c_bus;
static i2c_master_dev_handle_t g_adau_dev;

/* ─────────────────────────── Small Utils ─────────────────────────── */
static inline int16_t bswap16(int16_t v) {
    uint16_t u = (uint16_t)v;
    u = (uint16_t)((u >> 8) | (u << 8));
    return (int16_t)u;
}

/* ─────────────────────────── I2C + ADAU7118 ─────────────────────────── */
static void i2c_init_or_die(void) {
    i2c_master_bus_config_t bus = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &g_i2c_bus));

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ADAU7118_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus, &dev, &g_adau_dev));
}

static esp_err_t adau_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(g_adau_dev, buf, sizeof(buf), -1);
}
static esp_err_t adau_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(g_adau_dev, &reg, 1, data, len, -1);
}

/* Minimale, robuste Grundkonfig gemäß deinem Stand */
static void adau7118_init_or_die(void) {
    ESP_ERROR_CHECK(adau_write(0x05, 0xC0)); // DEC=64, CLK-Zuweisung Standard
    ESP_ERROR_CHECK(adau_write(0x06, 0xF1)); // HPF an; 0xF0 = aus
    ESP_ERROR_CHECK(adau_write(0x07, 0x53)); // TDM, 16-bit Slots, left-justified, 0 delay
    ESP_ERROR_CHECK(adau_write(0x08, 0x00)); // Polarität normal

    ESP_ERROR_CHECK(adau_write(0x09, 0x01)); // C0 -> Slot0,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0A, 0x11)); // C1 -> Slot1,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0B, 0x21)); // C2 -> Slot2,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0C, 0x31)); // C3 -> Slot3,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0D, 0x41)); // C4 -> Slot4,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0E, 0x51)); // C5 -> Slot5,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0F, 0x60)); // C6 -> Slot6,  drive=0 (High-R)
    ESP_ERROR_CHECK(adau_write(0x10, 0x70)); // C7 -> Slot7,  drive=0 (High-R)

    ESP_ERROR_CHECK(adau_write(0x11, 0x3F)); // Drive = 15mA
    ESP_ERROR_CHECK(adau_write(0x12, 0x00)); // Reset aus
    ESP_ERROR_CHECK(adau_write(0x04, 0x3F)); // CH0..5 + CLK0/1 an (6 Kanäle + 2 Clocks)

    uint8_t reg0A;
    if (adau_read(0x0A, &reg0A, 1) == ESP_OK) {
        ESP_LOGI(TAG, "ADAU Reg 0x0A = 0x%02X", reg0A);
    } else {
        ESP_LOGW(TAG, "ADAU readback failed (0x0A)");
    }
}

/* ─────────────────────────── I2S/TDM RX ─────────────────────────── */
static i2s_chan_handle_t i2s_tdm_rx_init_or_die(void) {
    i2s_chan_handle_t rx = NULL;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx));

    i2s_tdm_slot_config_t slot_cfg =
        I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, TDM_ACTIVE_MASK);
    slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    slot_cfg.total_slot     = TDM_TOTAL_SLOTS;
    slot_cfg.ws_width       = 1;      // TDM-konformer kurzer WS-Puls
    slot_cfg.ws_pol         = true;   // Frame-Start HIGH-Puls

    i2s_tdm_clk_config_t clk_cfg = {
        .clk_src        = I2S_CLK_SRC_DEFAULT,
        .sample_rate_hz = SAMPLE_RATE_HZ,
        .mclk_multiple  = I2S_MCLK_MULTIPLE_384,
    };

    i2s_tdm_gpio_config_t gpio_cfg = {
        .mclk = -1,        // kein MCLK nötig
        .bclk = I2S_PIN_BCLK,
        .ws   = I2S_PIN_WS,
        .dout = -1,        // kein TX
        .din  = I2S_PIN_DIN,
    };

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg  = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = gpio_cfg,
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(rx, &tdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx)); // Enable hier, damit die Read-Task nicht stolpert
    return rx;
}

/* ─────────────────────────── Wi-Fi ─────────────────────────── */
static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "Disconnected, retrying…");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*) data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(g_wifi_evt_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_and_wait_ip_or_die(void) {
    // NVS vor Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_wifi_evt_group = xEventGroupCreate();
    configASSERT(g_wifi_evt_group);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = { 0 };
    strlcpy((char*)wifi_cfg.sta.ssid,     CONFIG_WIFI_SSID,     sizeof(wifi_cfg.sta.ssid));
    strlcpy((char*)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // weniger Jitter

    ESP_LOGI(TAG_WIFI, "Waiting for IP…");
    xEventGroupWaitBits(g_wifi_evt_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG_WIFI, "Connected.");
}

/* ─────────────────────────── Tasks ─────────────────────────── */
static void audio_read_task(void *arg) {
    (void)arg;

    uint8_t *chunk = (uint8_t *)malloc(RB_CHUNK_BYTES);
    if (!chunk) { ESP_LOGE(TAG, "alloc %u failed", (unsigned)RB_CHUNK_BYTES); vTaskDelete(NULL); }

    ESP_LOGI(TAG, "reader start: chunk=%uB, frames=%u, frame_bytes=%u",
             (unsigned)RB_CHUNK_BYTES, (unsigned)CHUNK_FRAMES, (unsigned)AUDIO_FRAME_BYTES);

    for (;;) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(g_i2s_rx, chunk, RB_CHUNK_BYTES, &bytes_read, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s read err=%d", err);
            continue;
        }

        // Frame-Alignment (Sicherheitsnetz)
        size_t aligned = bytes_read - (bytes_read % AUDIO_FRAME_BYTES);
        if (!aligned) continue;

        if (xRingbufferSend(g_audio_rb, chunk, aligned, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "ringbuffer full, dropped %u", (unsigned)aligned);
        }
    }
}

static void stream_task(void *arg) {
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ESP_LOGE(TAG, "socket(): %d", errno); vTaskDelete(NULL); }

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(CONFIG_UDP_DEST_PORT),
        .sin_addr.s_addr = inet_addr(CONFIG_UDP_DEST_IP),
    };

    // QoS: DSCP EF -> Voice (WMM AC_VO)
    int tos = 0xB8;  // DSCP 46 << 2
    if (setsockopt(sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) < 0) {
        ESP_LOGW(TAG, "Failed to set DSCP/TOS");
    }

    // --- Minimaler Header (8 B) ---
    struct __attribute__((packed)) {
        uint32_t seq;    // +1 je Paket
        uint32_t ts48k;  // Samples in 48 kHz-Ticks
    } header;

    uint32_t seq = 0;
    uint32_t ts48k = 0;

    // Max. PCM-Bytes pro Paket (ohne Header)
    enum { MAX_PCM_BYTES_PER_PKT = TX_NUM_CHANNELS * (int)sizeof(int16_t) * FRAMES_PER_UDP };

    // Gemeinsamer TX-Puffer: Header + PCM (auf Stack ok: ~8 + 1152 B)
    uint8_t txbuf[sizeof(header) + MAX_PCM_BYTES_PER_PKT];

    const size_t pkt_bytes = MAX_PCM_BYTES_PER_PKT;  // nur zur Doku
    uint8_t *out = (uint8_t *)malloc(pkt_bytes);
    if (!out) { ESP_LOGE(TAG, "alloc out failed"); close(sock); vTaskDelete(NULL); }

    ESP_LOGI(TAG, "stream: %uch int16 LE -> %s:%u", (unsigned)TX_NUM_CHANNELS, CONFIG_UDP_DEST_IP, (unsigned)CONFIG_UDP_DEST_PORT);

    for (;;) {
        size_t item_size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceive(g_audio_rb, &item_size, portMAX_DELAY);
        if (!data) continue;

        int16_t *samples = (int16_t *)data;
        size_t nframes   = item_size / AUDIO_FRAME_BYTES;

        for (size_t f0 = 0; f0 < nframes; f0 += FRAMES_PER_UDP) {
            const size_t fcount = (f0 + FRAMES_PER_UDP <= nframes) ? FRAMES_PER_UDP : (nframes - f0);
            int16_t *out_i16 = (int16_t *)out;

            // Deinterleave: nur die gewählten Slots/Kanäle
            for (size_t f = 0; f < fcount; ++f) {
                size_t base = (f0 + f) * TDM_TOTAL_SLOTS;
                for (size_t ch = 0; ch < TX_NUM_CHANNELS; ++ch) {
                    int16_t v = samples[base + TX_SLOT_IDX[ch]];
                    v = bswap16(v); // falls RX MSB-first liefert
                    out_i16[f * TX_NUM_CHANNELS + ch] = v;
                }
            }

            const size_t pcm_bytes = fcount * TX_NUM_CHANNELS * sizeof(int16_t);

            // --- Header füllen (Network Byte Order) ---
            header.seq   = htonl(seq++);
            header.ts48k = htonl(ts48k);
            ts48k += (uint32_t)fcount;  // wichtig: tatsächliche Framezahl!

            // --- Header + PCM zusammen senden ---
            memcpy(txbuf, &header, sizeof(header));
            memcpy(txbuf + sizeof(header), out, pcm_bytes);

            ssize_t sent = sendto(sock, txbuf, sizeof(header) + pcm_bytes, 0, (struct sockaddr *)&dest, sizeof(dest));
            if (sent < 0 && (errno == ENOMEM || errno == ENOBUFS || errno == EAGAIN)) {
            vTaskDelay(pdMS_TO_TICKS(1));   
            continue;                       
            }
        }

        vRingbufferReturnItem(g_audio_rb, data);
    }
}


/* ─────────────────────────── app_main ─────────────────────────── */
void app_main(void) {
    i2c_init_or_die();
    adau7118_init_or_die();

    g_i2s_rx = i2s_tdm_rx_init_or_die();

    wifi_init_and_wait_ip_or_die();

    g_audio_rb = xRingbufferCreate(RB_CHUNK_BYTES * RB_NUM_CHUNKS, RINGBUF_TYPE_NOSPLIT);
    configASSERT(g_audio_rb);


    BaseType_t ok1 = xTaskCreatePinnedToCore(audio_read_task, "reader", 4096, NULL, 5, NULL, 1);
    BaseType_t ok2 = xTaskCreatePinnedToCore(stream_task,     "stream", 4096, NULL, 4, NULL, 0);
    configASSERT(ok1 == pdPASS && ok2 == pdPASS);
}
