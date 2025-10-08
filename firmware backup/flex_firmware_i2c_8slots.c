#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/i2s_tdm.h"
#include "esp_log.h"
#include "esp_err.h"
#include <stdio.h>

static const char *TAG = "ADAU7118_TDM16";

// ------- I2C / ADAU -------
#define I2C_PORT      I2C_NUM_0
#define I2C_SDA_GPIO  7
#define I2C_SCL_GPIO  6
#define I2C_FREQ_HZ   400000
#define ADAU7118_ADDR 0x14  // 7-bit

// ------- I2S-TDM Host (ESP32 als Master) -------
#define SAMPLE_RATE   16000
#define SLOT_WIDTH    16
#define TOTAL_SLOTS   8             // TDM-8 (128×fS bei 16 Bit)
#define READ_SLOTS    6             // wir nutzen/lesen nur Slots 0..5
#define PIN_BCLK      5
#define PIN_WS        4             // FSYNC
#define PIN_DIN       12            // SDATA vom ADAU -> ESP DIN

static i2s_chan_handle_t rx_handle;

// --- I2C helpers ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}
static esp_err_t adau_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, ADAU7118_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(50));
}
static esp_err_t adau_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, ADAU7118_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(50));
}

// --- ADAU7118: 6ch aktiv, 16-bit Slots, LJ(0), TDM ---
static esp_err_t adau7118_init_6ch_16bit_LJ0(void) {
    esp_err_t err;

    // ENABLES (0x04):
    // CLK0/1 an, CH01/23/45 an, CH67 aus  -> 0x37
    if ((err = adau_write(0x04, 0x37)) != ESP_OK) return err;  // CH6/7 disabled, 0..5 enabled. :contentReference[oaicite:4]{index=4}

    // DEC_RATIO_CLK_MAP (0x05): 64× Decimation, Default Clock-Mapping
    if ((err = adau_write(0x05, 0xC0)) != ESP_OK) return err;  // :contentReference[oaicite:5]{index=5}

    // HPF_CONTROL (0x06): HPF an, fc ≈ 0.00251·fS (~40 Hz @16kHz)
    if ((err = adau_write(0x06, 0x61)) != ESP_OK) return err;  // :contentReference[oaicite:6]{index=6}

    // SPT_CTRL1 (0x07):
    // TRI_STATE=1, SLOT_WIDTH=01 (16b), DATA_FORMAT=001 (LJ 0 delay), SAI_MODE=1 (TDM)
    // -> 0b0101_0011 = 0x53
    if ((err = adau_write(0x07, 0x53)) != ESP_OK) return err;  // :contentReference[oaicite:7]{index=7}

    // SPT_CTRL2 (0x08): normale Polarität
    if ((err = adau_write(0x08, 0x00)) != ESP_OK) return err;  // :contentReference[oaicite:8]{index=8}

    // SPT_C6 (0x0F) & SPT_C7 (0x10): Slot-Nummern belassen, DRV=0 (High-Z)
    if ((err = adau_write(0x0F, 0x60)) != ESP_OK) return err;  // CH6 -> Slot6, DRV=0
    if ((err = adau_write(0x10, 0x70)) != ESP_OK) return err;  // CH7 -> Slot7, DRV=0  :contentReference[oaicite:9]{index=9}

    return ESP_OK;
}

// --- I2S-TDM Init (RX, Master, 8 Slots à 16 Bit; wir lesen 6) ---
static void i2s_tdm_init(void) {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 8,
        .dma_frame_num = 64,
        .auto_clear = true,
    };
    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_192,
            .bclk_div = 0, // automatisch
        },
        .slot_cfg = {
            // wir generieren 8 Slots, interessieren uns aber nur für 0..5
            .slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 |
                         I2S_TDM_SLOT3 | I2S_TDM_SLOT4 | I2S_TDM_SLOT5,
            .total_slot = TOTAL_SLOTS,                    // 8
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,   // 16-Bit-Slots
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,   // 16-Bit Daten lesen
            .slot_mode = I2S_SLOT_MODE_MONO,
            .ws_width = 1,     // FSYNC Pulsbreite 1 BCLK (TDM-Puls) :contentReference[oaicite:10]{index=10}
            .ws_pol = false,   // positiver Puls (normal) :contentReference[oaicite:11]{index=11}
            .bit_shift = false,// LJ(0) → kein 1-Bit-Shift (I²S wäre true) :contentReference[oaicite:12]{index=12}
            .big_endian = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_BCLK,
            .ws   = PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_DIN,   // ADAU SDATA -> hier rein (Pin 11) :contentReference[oaicite:13]{index=13}
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        }
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(rx_handle, &tdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

// --- Reader Task: Slots 0..5 auslesen ---
static void i2s_reader_task(void *arg) {
    const int slot_bytes  = SLOT_WIDTH / 8;                 // 2
    const int frame_bytes = TOTAL_SLOTS * slot_bytes;       // 16 bytes/Frame
    uint8_t dma_buf[frame_bytes * 32];
    size_t bytes_read;

    while (1) {
        esp_err_t err = i2s_channel_read(rx_handle, dma_buf, sizeof(dma_buf), &bytes_read, portMAX_DELAY);
        if (err != ESP_OK || bytes_read == 0) {
            ESP_LOGW(TAG, "Keine Daten (err=%d)", err);
            continue;
        }
        int frames = bytes_read / frame_bytes;
        uint8_t *p = dma_buf;

        for (int f = 0; f < frames; f++) {
            printf("F%03d: ", f);
            // nur 6 Slots (0..5) ausgeben
            for (int ch = 0; ch < READ_SLOTS; ch++) {
                // 16-Bit signiertes PCM (MSB-first im Slot)
                int16_t s = (int16_t)((p[0] << 8) | p[1]);
                printf("CH%d=%d ", ch, (int)s);
                p += slot_bytes;
            }
            // 2 ungenutzte Slots (6,7) überspringen
            p += (TOTAL_SLOTS - READ_SLOTS) * slot_bytes;
            printf("\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t ids[4];
    if (adau_read(0x00, ids, sizeof(ids)) == ESP_OK) {
        ESP_LOGI(TAG, "ADAU7118 IDs: VENDOR=0x%02X DEV1=0x%02X DEV2=0x%02X REV=0x%02X",
                 ids[0], ids[1], ids[2], ids[3]); // 0x41, 0x71, 0x18, Rev 0x00 :contentReference[oaicite:14]{index=14}
    }

    ESP_ERROR_CHECK(adau7118_init_6ch_16bit_LJ0());
    // Hinweis: Der ADAU sendet erst nach anliegenden Takten; 16 Frames bis PDM_CLK, +48 bis SDATA. :contentReference[oaicite:15]{index=15}

    i2s_tdm_init();
    xTaskCreate(i2s_reader_task, "i2s_reader_task", 4096, NULL, 10, NULL);
}
