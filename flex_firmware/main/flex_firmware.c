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
#include "driver/i2c_master.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"


/* ===== Pins/Parameter (ESP32-S3) ===== */
#define I2C_PORT      I2C_NUM_0
#define I2C_SDA_GPIO  7
#define I2C_SCL_GPIO  6
#define I2C_FREQ_HZ   100000
#define ADAU7118_ADDR 0x14  // 7-bit

static const char *TAG = "UDP_unicast";

#define SAMPLE_RATE   48000
#define ACTIVE_MASK ( I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 | I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7 )
#define TOTAL_SLOTS   8
#define SLOT_BITS     16
#define ACTIVE_SLOTS     8
#define BYTES_PER_SAMPLE   (SLOT_BITS / 8)  
#define FRAME_BYTES      (TOTAL_SLOTS * BYTES_PER_SAMPLE)
// S3-Pins anpassen:
#define PIN_BCLK      5
#define PIN_WS        4
#define PIN_DIN       12

/* ------- I²S / Ringbuffer ------- */
#define CHUNK_FRAMES     256                              // ≈5.33 ms @ 48 kHz, 8×16 Bit -> Anzahl der Frames pro Schleife
#define RB_CHUNK_BYTES   (FRAME_BYTES * CHUNK_FRAMES)     // = 4096 B
#define RB_NUM_CHUNKS    24                               // 24 × 4096 B ≈ 96 kB Gesamt
static RingbufHandle_t audio_rb;
static i2s_chan_handle_t rx_chan;
_Static_assert((RB_CHUNK_BYTES % FRAME_BYTES) == 0, "Chunk must be frame-aligned");
_Static_assert(SLOT_BITS == 16, "Dieses Setup erwartet 16 Bit pro Slot");
//----------------------------------------

//-----------------WLAN--------------------
#define WIFI_SSID    "iPhone von Leonard " //"FRITZ!Box 7412"
#define WIFI_PASS      "12341234"//"03829891029701011627"
#define HOST_IP_ADDR "172.20.10.3"//"192.168.178.41"   // Empfänger 
#define DEST_PORT    7000


// Event-Gruppe für Verbindungsstatus
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static const char *TAG_WIFI = "wifi_init";
//-----------------------------------------


// Bus + Device anlegen
i2c_master_bus_handle_t i2c_bus;
i2c_master_dev_handle_t adau_dev;

/* ===== I2C ===== */
void i2c_init(void) {
    i2c_master_bus_config_t bus = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &i2c_bus));

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ADAU7118_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev, &adau_dev));
}


// Konfigurierbares Gain
#define GAIN_DB   40
// 6 dB ≙ ×2, also Schift = GAIN_DB/6:
#define GAIN_SHIFT  (GAIN_DB/6)   // 24 dB -> 4

static inline int16_t sat_shift_up_i16(int16_t v, int sh)
{
    // saturierende Linksverschiebung
    int32_t t = (int32_t)v << sh;
    if (t > 32767)  return 32767;
    if (t < -32768) return -32768;
    return (int16_t)t;
}

//byteswap
static inline int16_t bswap16(int16_t v) {
    uint16_t u = (uint16_t)v;
    u = (uint16_t)((u >> 8) | (u << 8));
    return (int16_t)u;
}

static i2s_chan_handle_t tdm_init(void)
{
    i2s_chan_handle_t handle = NULL;

    // RX-Kanal erzeugen (ESP als Master)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &handle));

    // Slot-Config: MSB (= Left-Justified, 0-Bit Delay), 16-Bit Daten, Slots 0..7 aktiv
    i2s_tdm_slot_config_t slot_cfg =
        I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, ACTIVE_MASK);
    slot_cfg.slot_bit_width  = I2S_SLOT_BIT_WIDTH_16BIT;    // 16 BCLK pro Slot
    slot_cfg.total_slot  = TOTAL_SLOTS;                 // 8 Slots pro Frame
    slot_cfg.ws_width        = 1;                           // WS-Puls 1 BCLK (TDM-konform zum ADAU7118)
    slot_cfg.ws_pol   = true;  // kurzer HIGH-Puls am Frame-Start

    // Clock-Config: 48 kHz; MCLK nicht ausgeben (mclk=-1 unten)
    i2s_tdm_clk_config_t clk_cfg = {
        .clk_src         = I2S_CLK_SRC_DEFAULT,
        .sample_rate_hz  = SAMPLE_RATE,
        .mclk_multiple   = I2S_MCLK_MULTIPLE_384,          
    };

    // GPIO-Zuordnung (nur Eingang; ADAU7118 hat nur SDATA-Out)
    i2s_tdm_gpio_config_t gpio_cfg = {
        .mclk = -1,                // ADAU7118 benötigt keinen externen MCLK
        .bclk = PIN_BCLK,
        .ws   = PIN_WS,
        .dout = -1,                // kein TX
        .din  = PIN_DIN,
    };

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg  = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = gpio_cfg,
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(handle, &tdm_cfg));
    return handle;
}


esp_err_t adau_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(adau_dev, buf, sizeof(buf), -1);
}

esp_err_t adau_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(adau_dev, &reg, 1, data, len, -1);
}

/* ===== ADAU7118: einfache Grundkonfig + Enable ===== */
static void adau7118_init(void) {
  /*  // optional: IDs loggen
    uint8_t ids[4];
    if (adau_read(0x00, ids, sizeof(ids)) == ESP_OK) {
        ESP_LOGI(TAG, "ADAU7118 ID: %02X %02X %02X %02X", ids[0], ids[1], ids[2], ids[3]);
    }
    */
    // 0x05: DEC=64, DAT0/1->CLK0, DAT2/3->CLK1 (Standard)
    ESP_ERROR_CHECK(adau_write(0x05, 0xC0));
    // 0x06: HPF an (0.00000486*fs). Falls kein HPF gewünscht: 0xF0
    ESP_ERROR_CHECK(adau_write(0x06, 0xF1));
    // 0x07: TDM, 16-bit Slots, left justified, 0 delay
    ESP_ERROR_CHECK(adau_write(0x07, 0x53));
    // 0x08: normale Polarität
    ESP_ERROR_CHECK(adau_write(0x08, 0x00));
    
    ESP_ERROR_CHECK(adau_write(0x09, 0x01)); // C0 -> Slot0,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0A, 0x11)); // C1 -> Slot1,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0B, 0x21)); // C2 -> Slot2,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0C, 0x31)); // C3 -> Slot3,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0D, 0x41)); // C4 -> Slot4,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0E, 0x51)); // C5 -> Slot5,  drive=1
    ESP_ERROR_CHECK(adau_write(0x0F, 0x60)); // C6 -> Slot6,  drive=0 (High-R)
    ESP_ERROR_CHECK(adau_write(0x10, 0x70)); // C7 -> Slot7,  drive=0 (High-R)

    // Drive Strength auf 15mA auf allen Leitungen
    ESP_ERROR_CHECK(adau_write(0x11, 0x3F));
    // Reset Register aus
    ESP_ERROR_CHECK(adau_write(0x12, 0x00));

    // 0x04: CH0..5 + CLK0/1 an (Enable) 0x37 für dat6/7 disable
    ESP_ERROR_CHECK(adau_write(0x04, 0x3F));

    uint8_t val;

    if (adau_read(0x0A, &val, 1) == ESP_OK) {
        ESP_LOGI(TAG, "Reg 0x0A = 0x%02X", val);
    } else {
        ESP_LOGE(TAG, "Failed to read Reg 0x0A");
    }
}


// Read-Task: liest von I2S/TDM und schreibt CHUNKs in den Ringbuffer.
// Erwartet: rx_chan ist initialisiert, audio_rb existiert.
// Schreibt nur vollständige, frame-ausgerichtete CHUNKs (RB_CHUNK_BYTES).

static void audio_read_task(void *arg)
{
    (void)arg;
    esp_err_t err;
    size_t bytes_read = 0;

    //Puffer  anlegen 
    uint8_t *chunk_buf = (uint8_t *)malloc(RB_CHUNK_BYTES);
    if (!chunk_buf) {
        ESP_LOGE(TAG, "buffer alloc failed (%u bytes)", (unsigned)RB_CHUNK_BYTES);
        vTaskDelete(NULL);
        return;
    }

    // RX einschalten (ok, falls bereits enabled)
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    ESP_LOGI(TAG, "audio_read_task started (chunk=%u B, frames=%u, frame_bytes=%u)",
             (unsigned)RB_CHUNK_BYTES, (unsigned)CHUNK_FRAMES, (unsigned)FRAME_BYTES);

    for (;;) {
        // Blockierendes Lesen exakt auf CHUNK-Größe
        err = i2s_channel_read(rx_chan, chunk_buf, RB_CHUNK_BYTES, &bytes_read, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_read err=%d", err);
            continue; // nächster Versuch
        }

        // Sicherheits-Alignment auf ganze Frames 
        size_t aligned = bytes_read - (bytes_read % FRAME_BYTES);
        if (aligned == 0) {
            continue;
        }

        // In den Ringbuffer kopieren; mit Timeout, damit Task nicht permanent blockiert
        if (xRingbufferSend(audio_rb, chunk_buf, aligned, pdMS_TO_TICKS(100)) != pdTRUE) {
            // Buffer voll -> Drop (nur Log; kein Retry, um Latenz klein zu halten)
            ESP_LOGW(TAG, "Ringbuffer full, dropped %u bytes", (unsigned)aligned);
        }

    }
}

static void stream_task(void *arg)
{
    (void)arg;



    // UDP-Ziel
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ESP_LOGE(TAG, "socket(): %d", errno); vTaskDelete(NULL); return; }

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(DEST_PORT),
        .sin_addr.s_addr = inet_addr(HOST_IP_ADDR),
    };

    // Wir senden NUR Slot 0 & 1 -> 2 Kanäle (int16)
    const size_t SLOTS_OUT = 2;

    // Max Frames pro RB-Item (z. B. 256)
    const size_t MAX_FRAMES = RB_CHUNK_BYTES / FRAME_BYTES;

    // Ausgabepuffer: 2 Kanäle * 2 Bytes * MAX_FRAMES
    const size_t OUT_MAX_BYTES = SLOTS_OUT * sizeof(int16_t) * MAX_FRAMES;
    uint8_t *out = (uint8_t *)malloc(OUT_MAX_BYTES);
    if (!out) { ESP_LOGE(TAG, "malloc out failed"); close(sock); vTaskDelete(NULL); return; }

    ESP_LOGI(TAG, "stream_task: 2ch UDP (raw LE) -> %s:%u", HOST_IP_ADDR, DEST_PORT);

    for (;;) {
        size_t item_size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceive(audio_rb, &item_size, portMAX_DELAY);
        if (!data) continue;

        int16_t *samples = (int16_t *)data;
        size_t nframes = item_size / FRAME_BYTES;           // FRAME_BYTES = TOTAL_SLOTS * 2

        // Slots 0 & 1 extrahieren (LE roh, kein htons)
        int16_t *out_i16 = (int16_t *)out;
        for (size_t f = 0; f < nframes; ++f) {
            size_t base = f * TOTAL_SLOTS;   // bei dir: 8
            int16_t l = samples[base + 1];   // Slot 0 (oder 2/3 je nach Auswahl)
            int16_t r = samples[base + 2];

            // byte swap
             l = bswap16(l); 
             r = bswap16(r);

             out_i16[2 * f + 0] = l; // sat_shift_up_i16(l, GAIN_SHIFT);
             out_i16[2 * f + 1] = r; // sat_shift_up_i16(r, GAIN_SHIFT);
        }


        size_t total = nframes * SLOTS_OUT * sizeof(int16_t); // = nframes * 4
        (void)sendto(sock, out, total, 0, (struct sockaddr *)&dest, sizeof(dest));

        vRingbufferReturnItem(audio_rb, data);
    }

    // (nie erreicht)
    // free(out);
    // close(sock);
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
    // --- NVS MUSS vor Wi-Fi initialisiert sein ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Event-Gruppe anlegen (einmalig)
    s_wifi_event_group = xEventGroupCreate();

    // Netif + Event Loop (einmalig)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Wi-Fi Init
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Event-Handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL, NULL));

    // Achtung: SSID hat in deinem Code ein Leerzeichen am Ende.
    // Wenn das so beim Hotspot nicht exakt passt, schlägt das Verbinden fehl.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // weniger Jitter


    ESP_LOGI(TAG_WIFI, "Waiting for IP...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG_WIFI, "Connected!");
}





void app_main(void)
{
    i2c_init();
    adau7118_init();
    rx_chan = tdm_init();
    wifi_init_sta();
    // Ringbuffer: ganze Chunks, keine Splits
    audio_rb = xRingbufferCreate(RB_CHUNK_BYTES * RB_NUM_CHUNKS, RINGBUF_TYPE_NOSPLIT);
    configASSERT(audio_rb);

    // Tasks starten
    configASSERT(xTaskCreatePinnedToCore(audio_read_task, "reader", 4096, NULL, 4, NULL, 1) == pdPASS);
    configASSERT(xTaskCreatePinnedToCore(stream_task,  "stream", 4096, NULL, 3, NULL, 0) == pdPASS);

     // Sine-Streamer starten
    //xTaskCreatePinnedToCore(stream_sine_task, "sine", 4096, NULL, 3, NULL, 0);

}

