// ── Transmissor LoRa P2P - TTGO LoRa32 V2.0 (com display SSD1306 + RTC DS3231)
//
// ADICIONADO:
//   [1] RTC DS3231 via I2C — timestamp em cada leitura e no display
//
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "ssd1306.h"

#define REG_RX_NB_BYTES          0x13
#define REG_FIFO_RX_CURRENT_ADDR 0x10
// ── Pinos TTGO LoRa32 V2.0 ───────────────────────────────────────────────────
#define PIN_SCLK   5
#define PIN_MOSI   27
#define PIN_MISO   19
#define PIN_NSS    18
#define PIN_RST    14
#define PIN_DIO0   26
#define PIN_LED    25

// ── Pinos I2C / OLED + DS3231 ────────────────────────────────────────────────
// OLED e DS3231 compartilham o mesmo barramento I2C (endereços diferentes)
#define OLED_SDA        4
#define OLED_SCL        15
#define OLED_ADDR       0x3C   // endereço I2C do SSD1306
#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     400000

static const char *TAG = "I2C_SCAN";
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      (OLED_HEIGHT / 8)

// ── DS3231 ────────────────────────────────────────────────────────────────────
#define DS3231_ADDR     0x68   // endereço I2C fixo do DS3231

typedef struct {
    uint8_t seconds;   // 0–59
    uint8_t minutes;   // 0–59
    uint8_t hours;     // 0–23
    uint8_t day;       // 1–7
    uint8_t date;      // 1–31
    uint8_t month;     // 1–12
    uint8_t year;      // 0–99 (+ 2000)
} ds3231_time_t;

static inline uint8_t bcd2dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }
static inline uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }

// ── Registradores SX1276 ─────────────────────────────────────────────────────
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_IRQ_FLAGS            0x12
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_SYNC_WORD            0x39
#define REG_PA_DAC               0x4D
#define REG_VERSION              0x42

// ── Modos SX1276 ─────────────────────────────────────────────────────────────
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

// ── Configurações LoRa ────────────────────────────────────────────────────────
#define LORA_FREQUENCY           915000000
#define LORA_SYNC_WORD           0x12
#define TX_INTERVAL_MS           10000

static spi_device_handle_t spi;
static SSD1306_t dev;


// ── Payload de sensores ───────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    int16_t  temperature_x10;
    uint32_t counter;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
} sensor_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
    uint8_t  date;
    uint8_t  month;
    uint16_t year;
} time_payload_t;
// ═════════════════════════════════════════════════════════════════════════════
// ── DS3231 ───────────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

// NOTA: i2c_driver_install() é chamado apenas em oled_init().
// O DS3231 compartilha o mesmo barramento — não inicializa I2C novamente.
static uint8_t  rtc_hours   = 0;
static uint8_t  rtc_minutes = 0;
static uint8_t  rtc_seconds = 0;
static uint8_t  rtc_date    = 1;
static uint8_t  rtc_month   = 1;
static uint16_t rtc_year    = 2025;
static bool     rtc_synced  = false;
static esp_err_t ds3231_get_time(ds3231_time_t *t)
{
    // Aponta para registrador 0x00
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Lê 7 bytes
    uint8_t data[7];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    t->seconds = bcd2dec(data[0] & 0x7F);
    t->minutes = bcd2dec(data[1]);
    t->hours   = bcd2dec(data[2] & 0x3F);
    t->day     = bcd2dec(data[3] & 0x07);
    t->date    = bcd2dec(data[4]);
    t->month   = bcd2dec(data[5] & 0x1F);
    t->year    = bcd2dec(data[6]);
    return ESP_OK;
}

// ── (Opcional) Acerto de hora — descomente e chame uma vez se necessário ──────
/*
static esp_err_t ds3231_set_time(const ds3231_time_t *t)
{
    uint8_t data[8] = {
        0x00,
        dec2bcd(t->seconds), dec2bcd(t->minutes), dec2bcd(t->hours),
        dec2bcd(t->day),     dec2bcd(t->date),    dec2bcd(t->month),
        dec2bcd(t->year),
    };
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
*/
static void rtc_tick(void)
{
    if (!rtc_synced) return;
    rtc_seconds++;
    if (rtc_seconds >= 60) { rtc_seconds = 0; rtc_minutes++; }
    if (rtc_minutes >= 60) { rtc_minutes = 0; rtc_hours++;   }
    if (rtc_hours   >= 24) { rtc_hours   = 0;                }
}

static void rtc_get_time_str(char *buf, size_t len)
{
    if (!rtc_synced)
        snprintf(buf, len, "--:--:--");
    else
        snprintf(buf, len, "%02d:%02d:%02d",
                 rtc_hours, rtc_minutes, rtc_seconds);
}
// Formata como "2025-06-10 14:32:05" — buf deve ter ao menos 32 bytes
static void ds3231_format(const ds3231_time_t *t, char *buf, size_t len)
{
    snprintf(buf, len, "20%02d-%02d-%02d %02d:%02d:%02d",
             t->year, t->month, t->date,
             t->hours, t->minutes, t->seconds);
}

// Formata apenas hora como "14:32:05" — buf deve ter ao menos 12 bytes
static void ds3231_format_time(const ds3231_time_t *t, char *buf, size_t len)
{
    snprintf(buf, len, "%02d:%02d:%02d",
             t->hours, t->minutes, t->seconds);
}

// ═════════════════════════════════════════════════════════════════════════════
// ── ADC / LM35 ───────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         adc_cali;

void lm35_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &chan_cfg);

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali);
}

float lm35_read_celsius(void)
{
    int raw = 0, mv = 0;
    for (int i = 0; i < 64; i++) {
        int r;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &r);
        raw += r;
    }
    raw /= 64;
    ESP_LOGI(TAG, "Raw:%d", raw);
    adc_cali_raw_to_voltage(adc_cali, raw, &mv);
    ESP_LOGI(TAG, "MV:%d", mv);
    return mv / 10.0f;
}

// ═════════════════════════════════════════════════════════════════════════════
// ── SSD1306 ──────────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
            ESP_LOGI(TAG, "  Dispositivo encontrado: 0x%02X", addr);
    }
}


// ═════════════════════════════════════════════════════════════════════════════
// ── SPI helpers ──────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════
static uint8_t spi_read(uint8_t reg)
{
    spi_transaction_t t = {
        .length  = 16,
        .tx_data = { reg & 0x7F, 0x00 },
        .flags   = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_transmit(spi, &t);
    return t.rx_data[1];
}

static void spi_write(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length  = 16,
        .tx_data = { reg | 0x80, val },
        .flags   = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_transmit(spi, &t);
}

static void spi_write_buf(uint8_t reg, const uint8_t *buf, size_t len)
{
    uint8_t tx[len + 1];
    tx[0] = reg | 0x80;
    memcpy(tx + 1, buf, len);
    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
    };
    spi_device_transmit(spi, &t);
}

static void lora_spi_init(void)
{
    spi_bus_config_t bus = {
        .miso_io_num   = PIN_MISO,
        .mosi_io_num   = PIN_MOSI,
        .sclk_io_num   = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = PIN_NSS,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));
}
static void lora_receive_time(void)
{
    // Coloca em modo RX
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    // Aguarda até 3 segundos por uma resposta
    for (int ms = 0; ms < 3000; ms += 10) {
        uint8_t irq = spi_read(REG_IRQ_FLAGS);
        if (irq & 0x40) {
            spi_write(REG_IRQ_FLAGS, 0xFF);

            if (irq & 0x20) {   // erro CRC
                ESP_LOGW(TAG, "CRC error no pacote de hora");
                return;
            }

            uint8_t nb  = spi_read(REG_RX_NB_BYTES);
            uint8_t ptr = spi_read(REG_FIFO_RX_CURRENT_ADDR);
            spi_write(REG_FIFO_ADDR_PTR, ptr);

            if (nb == sizeof(time_payload_t)) {
                time_payload_t tp;
                uint8_t *raw = (uint8_t *)&tp;
                for (size_t i = 0; i < sizeof(tp); i++)
                    raw[i] = spi_read(REG_FIFO);

                if (tp.type == 0xAA) {
                    rtc_hours   = tp.hours;
                    rtc_minutes = tp.minutes;
                    rtc_seconds = tp.seconds;
                    rtc_date    = tp.date;
                    rtc_month   = tp.month;
                    rtc_year    = tp.year;
                    rtc_synced  = true;

                    ESP_LOGI(TAG, "Hora sincronizada: %02d:%02d:%02d %02d/%02d/%04d",
                             rtc_hours, rtc_minutes, rtc_seconds,
                             rtc_date, rtc_month, rtc_year);
                }
            }
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "Timeout — sem resposta de hora do receptor");
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RST, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1); vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_frequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    spi_write(REG_FRF_MSB, (frf >> 16) & 0xFF);
    spi_write(REG_FRF_MID, (frf >>  8) & 0xFF);
    spi_write(REG_FRF_LSB, (frf      ) & 0xFF);
}

static bool lora_init(void)
{
    lora_reset();
    uint8_t ver = spi_read(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 version: 0x%02X (esperado: 0x12)", ver);
    if (ver != 0x12) { ESP_LOGE(TAG, "SX1276 não encontrado!"); return false; }

    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    lora_set_frequency(LORA_FREQUENCY);
    spi_write(REG_FIFO_TX_BASE_ADDR, 0x00);
    spi_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    spi_write(REG_LNA, spi_read(REG_LNA) | 0x03);
    spi_write(REG_MODEM_CONFIG_1, 0x72);
    spi_write(REG_MODEM_CONFIG_2, 0x74);
    spi_write(REG_MODEM_CONFIG_3, 0x04);
    spi_write(REG_PA_CONFIG, 0x8F);
    spi_write(REG_PA_DAC,    0x87);
    spi_write(REG_PREAMBLE_MSB, 0x00);
    spi_write(REG_PREAMBLE_LSB, 0x08);
    spi_write(REG_SYNC_WORD, LORA_SYNC_WORD);
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    ESP_LOGI(TAG, "LoRa inicializado OK");
    return true;
}

static void lora_send(const uint8_t *data, size_t len)
{
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    spi_write(REG_FIFO_ADDR_PTR, 0x00);
    spi_write_buf(REG_FIFO, data, len);
    spi_write(REG_PAYLOAD_LENGTH, len);
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    while ((spi_read(REG_IRQ_FLAGS) & 0x08) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    spi_write(REG_IRQ_FLAGS, 0xFF);
    ESP_LOGI(TAG, "Pacote enviado (%d bytes)", len);
}

static void led_blink(int times)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(PIN_LED, 1); vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PIN_LED, 0); vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void oled_show_sensor(const sensor_payload_t *p, const char *ts_time)
{
    char line[24];

    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "LoRa Transmissor", 17, false);

    float temp   = p->temperature_x10 / 10.0f;
    int   t_int  = (int)temp;
    int   t_frac = (int)((temp - t_int) * 10);
    if (t_frac < 0) t_frac = -t_frac;
    snprintf(line, sizeof(line), "Temp: %d.%d C", t_int, t_frac);
    ssd1306_display_text(&dev, 2, line, strlen(line), false);

    snprintf(line, sizeof(line), "Pkt: #%lu", (unsigned long)p->counter);
    ssd1306_display_text(&dev, 4, line, strlen(line), false);

    snprintf(line, sizeof(line), "%s", ts_time);
    ssd1306_display_text(&dev, 6, line, strlen(line), false);
}
// ═════════════════════════════════════════════════════════════════════════════
// ── app_main ─────────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════
void app_main(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    lora_spi_init();
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    #if CONFIG_SSD1306_128x64
        ssd1306_init(&dev, 128, 64);
    #endif
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    // ── (Opcional) Acerto inicial do RTC — descomente se necessário ──────────
    // ds3231_time_t t = { .seconds=0, .minutes=30, .hours=14,
    //                     .day=2, .date=10, .month=6, .year=25 };
    // ds3231_set_time(&t);
    ESP_LOGI(TAG, "=== SCAN I2C ===");
    i2c_scan();
    ESP_LOGI(TAG, "=== FIM SCAN ===");

    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "LoRa Transmissor", 17, false);
    ssd1306_display_text(&dev, 2, "Inicializando...", 17, false);

    if (!lora_init()) {
        ESP_LOGE(TAG, "Falha ao inicializar LoRa. Travando.");
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, "ERRO LoRa!", 10, false);
        ssd1306_display_text(&dev, 2, "Verifique o", 11, false);
        ssd1306_display_text(&dev, 3, "hardware.", 9, false);
        while (1) { led_blink(10); vTaskDelay(pdMS_TO_TICKS(500)); }
    }

    ssd1306_clear_screen(&dev, false);
    ssd1306_display_text(&dev, 0, "LoRa OK!", 8, false);

    led_blink(3);
    lm35_init();

    sensor_payload_t payload = { .node_id = 0x01 };

    while (1) {
        float temp = lm35_read_celsius();
        payload.temperature_x10 = (int16_t)(temp * 10);
        payload.counter++;

        // Usa hora local (sincronizada pelo rádio)
        char ts_time[12];
        rtc_get_time_str(ts_time, sizeof(ts_time));

        // Preenche payload com hora atual
        payload.hours   = rtc_hours;
        payload.minutes = rtc_minutes;
        payload.seconds = rtc_seconds;

        ESP_LOGI(TAG, "Enviando pacote #%lu | temp=%.1f°C | %s",
                 (unsigned long)payload.counter,
                 payload.temperature_x10 / 10.0f,
                 ts_time);

        oled_show_sensor(&payload, ts_time);

        gpio_set_level(PIN_LED, 1);
        lora_send((uint8_t *)&payload, sizeof(payload));
        gpio_set_level(PIN_LED, 0);

        // Aguarda resposta com hora do receptor
        lora_receive_time();

        // Incrementa segundos localmente até próxima sincronização
        for (int i = 0; i < TX_INTERVAL_MS / 1000; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            rtc_tick();
        }
    }
}
