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



/* ------- I²S / Ringbuffer ------- */
#define RB_CHUNK_BYTES   144            // ∑ genau ein DMA-Block
#define RB_NUM_CHUNKS    64             // 64 × 128 B = 8 kB (anpassen ≙ Pufferlänge)
static RingbufHandle_t audio_rb;        // globales Handle
//----------------------------------------

#define PIN_LED      2
#define I2S_NUM         I2S_NUM_0
#define I2S_CLK      5      // Bit Clock (SCK)
#define I2S_WS       4      // Word Select (WS)
#define I2S_SD       12      // Serial Data (SD)

static const char *TAG = "UDP_unicast";
#define WIFI_SSID     "iPhone von Leonard " //"FRITZ!Box 7412"
#define WIFI_PASS      "12341234"//"03829891029701011627"
#define HOST_IP_ADDR "172.20.10.3"//"192.168.178.41"   // Empfänger
#define DEST_PORT    7000

// Event-Gruppe für Verbindungsstatus
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static const char *TAG_WIFI = "wifi_init";
static i2s_chan_handle_t rx_handle;

/* ---------- LED helper ---------- */
static inline void led_on (void) { gpio_set_level(PIN_LED, 1); }
static inline void led_off(void) { gpio_set_level(PIN_LED, 0); }
//-----------------------------------------------

static void io_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
}




void i2s_init_tdm(void)
{
    i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 8,
    .dma_frame_num = 9,
    .auto_clear = true,
    };
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE("I2S_TDM", "Fehler bei i2s_new_channel: %s", esp_err_to_name(err));
        return;
    }

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 16000,          // 16 kHz
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_192, // 192-fache BCLK Frequenz 
            .bclk_div = 0,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_24BIT, // 16 Bit pro Kanal
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,            // Jeder Kanal nutzt einen Slot
            .slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 |
                         I2S_TDM_SLOT3, // 4 Kanäle
            .ws_width = 1,         // FSYNC Pulsbreite: 1 Bitclock
            .ws_pol = false,       // FSYNC positive Polarität
            .bit_shift = true,    // 1bit Verzögerung
            .left_align = true,
            .big_endian = false,    
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // Kein MCLK nötig für ADAU7118
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

    err = i2s_channel_init_tdm_mode(rx_handle, &tdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE("I2S_TDM", "Fehler bei i2s_channel_init_tdm_mode: %s", esp_err_to_name(err));
        return;
    }

    err = i2s_channel_enable(rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE("I2S_TDM", "Fehler bei i2s_channel_enable: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI("I2S_TDM", "I2S TDM erfolgreich für ADAU7118 initialisiert");
}



static void i2s_read_task(void *arg)
{
    uint8_t dma_buf[RB_CHUNK_BYTES]; // Eingang: Vielfaches der Framegröße
    size_t bytes_read;

    while (1) {
        // I²S-TDM lesen
        esp_err_t ret = i2s_channel_read(arg, dma_buf, sizeof(dma_buf), &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK || bytes_read == 0) {
            ESP_LOGW(TAG, "Keine Daten vom I²S-Kanal erhalten (ret=%d)", ret);
            continue;
        }

        // 4 Kanäle × 4 Bytes (32-bit Slot) = 16 Bytes pro Frame
        int frames = (int)(bytes_read / 16);
        if (frames <= 0) {
            ESP_LOGW(TAG, "Keine vollständigen Frames erhalten (bytes=%u).", (unsigned)bytes_read);
            continue;
        }

        // 16-bit PCM: 2 Bytes * 4 Kanäle pro Frame => 8 Bytes pro Frame
        size_t out_bytes = (size_t)frames * 8;
        uint8_t *pcm_buf = (uint8_t *)malloc(out_bytes);
        if (!pcm_buf) {
            ESP_LOGE(TAG, "malloc(%u) für PCM-Ausgabe fehlgeschlagen!", (unsigned)out_bytes);
            continue;
        }

        const uint8_t *r = dma_buf;
        uint8_t *w = pcm_buf;

        for (int frame = 0; frame < frames; frame++) {
            for (int channel = 0; channel < 4; channel++) {
                /*
                 * DMA-Layout bei 24-in-32 left-justified, little-endian im RAM:
                 *   r[0] = Padding/LSB (typ. 0)
                 *   r[1] = Bits 15..8
                 *   r[2] = Bits 23..16
                 *   r[3] = MSB inkl. Vorzeichen
                 *
                 * 24 -> 16 Bit: oberste 16 Bits nehmen (Sample >> 8)
                 */
                int16_t s16 = (int16_t)((((int)r[3]) << 8) | r[2]);

                // Optionaler Gain mit Sättigung
                // float gain = 1.0f;
                // if (gain != 1.0f) {
                //     float v = (float)s16 * gain;
                //     if (v > 32767.0f) v = 32767.0f;
                //     if (v < -32768.0f) v = -32768.0f;
                //     s16 = (int16_t)v;
                // }

                // Big-Endian (MSB zuerst) in den PCM-Puffer schreiben
                *w++ = (uint8_t)((s16 >> 8) & 0xFF);
                *w++ = (uint8_t)(s16 & 0xFF);

                r += 4; // nächster Slot (32-bit)
            }
        }

        // Debug: erster Frame, 4 Kanäle (je 16-bit)
        ESP_LOGI(TAG, "Frame[0]: [%04X %04X %04X %04X]",
                 (pcm_buf[0] << 8) | pcm_buf[1],
                 (pcm_buf[2] << 8) | pcm_buf[3],
                 (pcm_buf[4] << 8) | pcm_buf[5],
                 (pcm_buf[6] << 8) | pcm_buf[7]);

        // In den Ringbuffer: 8 Bytes/Frame (4 Kanäle * 2 Bytes)
        if (xRingbufferSend(audio_rb, pcm_buf, out_bytes, portMAX_DELAY) != pdTRUE) {
            ESP_LOGW(TAG, "Ringbuffer voll, Datenverlust!");
        }

        free(pcm_buf);
    }
}



// Event-Handler für Wi-Fi Events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "Disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

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
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Waiting for IP...");
    // Warte, bis die Verbindung hergestellt ist
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG_WIFI, "Connected!");
}

static void udp_stream_task(void *arg)
{
    //uint32_t pkt_cnt = 0;
    uint32_t pkt_cnt = 0, pkt_sec = 0;
    int64_t last = esp_timer_get_time();
    

    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);



    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ESP_LOGE(TAG, "socket fail"); vTaskDelete(NULL); }

    
    const size_t MTU_PAYLOAD = 1024;            // maximal ohne Fragmentierung
    uint8_t pkt[MTU_PAYLOAD];

    while (1) {
        /* 1472 Byte ≈ 8,3 ms Audio → ~120 Pakete/s                   */
        size_t grabbed = 0;
        while (grabbed < MTU_PAYLOAD) {
            size_t got;
            uint8_t *p = xRingbufferReceiveUpTo(audio_rb, &got,
                                                portMAX_DELAY,
                                                MTU_PAYLOAD - grabbed);
            
            if (p == NULL) {
                ESP_LOGW(TAG, "Timeout: Ringbuffer leer");
                grabbed = 0; 
                continue; // nächster Versuch
                }
            
            memcpy(pkt + grabbed, p, got);
            grabbed += got;
            vRingbufferReturnItem(audio_rb, p);
        }

        /* schicken, ggf. mit Back-off warten                           */
        while (1) {
            ssize_t sent = sendto(sock, pkt, MTU_PAYLOAD, 0,
                (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            
            if (sent >= 0) {
                    // nach jedem erfolgreichen sendto:
                    pkt_sec++;
                    int64_t now = esp_timer_get_time();
                    if (now - last >= 1000000) { // jede Sekunde
                    printf("UDP-Pakete gesendet pro Sekunde: %ld\n", pkt_sec);
                    pkt_sec = 0;
                    last = now;
                    }

                 /* ---------- LED heartbeat ---------- */
                if (++pkt_cnt >= 50) {                 // alle 10 Pakete = 5 Hz
                pkt_cnt = 0;
                static bool led = false;
                led = !led;
                gpio_set_level(PIN_LED, led);
                }
                break;                    // Erfolg
            }
            if (errno == ENOMEM) {                   // Puffer voll
                vTaskDelay(pdMS_TO_TICKS(5));        // 5 ms Pause
            } else {
                ESP_LOGE(TAG, "sendto errno=%d", errno);
                vTaskDelay(pdMS_TO_TICKS(20));       // andere Fehler
            }
              
        }
    }
}




void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
   

    io_init();
    i2s_init_tdm();           
    wifi_init_sta();      // eigenes STA-Setup
    
    audio_rb = xRingbufferCreate(
        RB_CHUNK_BYTES * RB_NUM_CHUNKS,
        RINGBUF_TYPE_BYTEBUF
    );
    if (audio_rb == NULL) {
        ESP_LOGE(TAG, "Ringbuffer-Init fehlgeschlagen");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // 100 ms warten, bevor Tasks starten

    xTaskCreatePinnedToCore(i2s_read_task, "i2s_read_task",
                        4096, rx_handle, 22, NULL, 1);

    xTaskCreate(udp_stream_task, "udp_send",
                4096, NULL, 18, NULL);
}
