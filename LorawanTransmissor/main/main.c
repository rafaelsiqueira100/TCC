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
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_ADDR       0x3C   // endereço I2C do SSD1306
#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     400000
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

static const char *TAG = "LORA_TX";
static spi_device_handle_t spi;

// ── Payload de sensores ───────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    int16_t  temperature_x10;
    uint32_t counter;
} sensor_payload_t;

// ═════════════════════════════════════════════════════════════════════════════
// ── DS3231 ───────────────────────────────────────────────────────────────────
// ═════════════════════════════════════════════════════════════════════════════

// NOTA: i2c_driver_install() é chamado apenas em oled_init().
// O DS3231 compartilha o mesmo barramento — não inicializa I2C novamente.

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
static uint8_t oled_buf[OLED_WIDTH * OLED_PAGES];

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 33 '!'
    {0x00,0x07,0x00,0x07,0x00}, // 34 '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 '$'
    {0x23,0x13,0x08,0x64,0x62}, // 37 '%'
    {0x36,0x49,0x55,0x22,0x50}, // 38 '&'
    {0x00,0x05,0x03,0x00,0x00}, // 39 '''
    {0x00,0x1C,0x22,0x41,0x00}, // 40 '('
    {0x00,0x41,0x22,0x1C,0x00}, // 41 ')'
    {0x14,0x08,0x3E,0x08,0x14}, // 42 '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 43 '+'
    {0x00,0x50,0x30,0x00,0x00}, // 44 ','
    {0x08,0x08,0x08,0x08,0x08}, // 45 '-'
    {0x00,0x60,0x60,0x00,0x00}, // 46 '.'
    {0x20,0x10,0x08,0x04,0x02}, // 47 '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 49 '1'
    {0x42,0x61,0x51,0x49,0x46}, // 50 '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 51 '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 52 '4'
    {0x27,0x45,0x45,0x45,0x39}, // 53 '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 '6'
    {0x01,0x71,0x09,0x05,0x03}, // 55 '7'
    {0x36,0x49,0x49,0x49,0x36}, // 56 '8'
    {0x06,0x49,0x49,0x29,0x1E}, // 57 '9'
    {0x00,0x36,0x36,0x00,0x00}, // 58 ':'
    {0x00,0x56,0x36,0x00,0x00}, // 59 ';'
    {0x08,0x14,0x22,0x41,0x00}, // 60 '<'
    {0x14,0x14,0x14,0x14,0x14}, // 61 '='
    {0x00,0x41,0x22,0x14,0x08}, // 62 '>'
    {0x02,0x01,0x51,0x09,0x06}, // 63 '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 64 '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 66 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 67 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 69 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 70 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 71 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 73 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 74 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 75 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 76 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 77 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 80 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 82 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 83 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 84 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 87 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 88 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 89 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 90 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // 91 '['
    {0x02,0x04,0x08,0x10,0x20}, // 92 '\'
    {0x00,0x41,0x41,0x7F,0x00}, // 93 ']'
    {0x04,0x02,0x01,0x02,0x04}, // 94 '^'
    {0x40,0x40,0x40,0x40,0x40}, // 95 '_'
    {0x00,0x01,0x02,0x04,0x00}, // 96 '`'
    {0x20,0x54,0x54,0x54,0x78}, // 97 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 98 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 99 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 100 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 101 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 102 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 103 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 104 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 105 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 106 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 107 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 108 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 109 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 110 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 111 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 112 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 113 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 114 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 115 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 116 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 117 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 118 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 119 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 120 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 121 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 122 'z'
    {0x00,0x08,0x36,0x41,0x00}, // 123 '{'
    {0x00,0x00,0x7F,0x00,0x00}, // 124 '|'
    {0x00,0x41,0x36,0x08,0x00}, // 125 '}'
    {0x10,0x08,0x08,0x10,0x08}, // 126 '~'
    {0x00,0x00,0x00,0x00,0x00}, // 127 DEL
};

static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd };
    i2c_master_write_to_device(I2C_PORT, OLED_ADDR, buf, 2, pdMS_TO_TICKS(10));
}

static void oled_data(const uint8_t *data, size_t len)
{
    uint8_t buf[len + 1];
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    i2c_master_write_to_device(I2C_PORT, OLED_ADDR, buf, len + 1, pdMS_TO_TICKS(50));
}

static void oled_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = OLED_SDA,
        .scl_io_num       = OLED_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    vTaskDelay(pdMS_TO_TICKS(100));

    const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
    };
    for (size_t i = 0; i < sizeof(init_seq); i++) oled_cmd(init_seq[i]);

    memset(oled_buf, 0, sizeof(oled_buf));
    ESP_LOGI(TAG, "OLED SSD1306 inicializado");

    // Verifica presença do DS3231 no mesmo barramento
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
        ESP_LOGI(TAG, "DS3231 detectado no barramento I2C");
    else
        ESP_LOGW(TAG, "DS3231 não encontrado — timestamps serão \"--:--:--\"");
}

static void oled_flush(void)
{
    oled_cmd(0x21); oled_cmd(0); oled_cmd(127);
    oled_cmd(0x22); oled_cmd(0); oled_cmd(7);
    oled_data(oled_buf, sizeof(oled_buf));
}

static void oled_clear(void) { memset(oled_buf, 0, sizeof(oled_buf)); }

static void oled_draw_char(uint8_t page, uint8_t col, char c)
{
    if (c < 32 || c > 127) c = ' ';
    const uint8_t *glyph = font5x7[c - 32];
    for (int i = 0; i < 5; i++)
        if (col + i < OLED_WIDTH)
            oled_buf[page * OLED_WIDTH + col + i] = glyph[i];
    if (col + 5 < OLED_WIDTH)
        oled_buf[page * OLED_WIDTH + col + 5] = 0x00;
}

static void oled_draw_str(uint8_t page, uint8_t col, const char *str)
{
    while (*str && col < OLED_WIDTH) {
        oled_draw_char(page, col, *str++);
        col += 6;
    }
}

// ── Atualiza display — agora com timestamp na última linha ────────────────────
static void oled_show_sensor(const sensor_payload_t *p, const char *ts_time)
{
    char line[32];
    oled_clear();

    // Página 0: título
    oled_draw_str(0, 0, "LoRa Transmissor");

    // Página 1: separador
    for (int col = 0; col < OLED_WIDTH; col++)
        oled_buf[1 * OLED_WIDTH + col] = 0x08;

    // Página 3: temperatura
    float temp   = p->temperature_x10 / 10.0f;
    int   t_int  = (int)temp;
    int   t_frac = (int)((temp - t_int) * 10);
    if (t_frac < 0) t_frac = -t_frac;
    snprintf(line, sizeof(line), "Temp: %d.%d C", t_int, t_frac);
    oled_draw_str(3, 0, line);

    // Página 5: contador
    snprintf(line, sizeof(line), "Pkt: #%lu", (unsigned long)p->counter);
    oled_draw_str(5, 0, line);

    // Página 7: hora do RTC  ← novo
    snprintf(line, sizeof(line), "%s", ts_time);
    oled_draw_str(7, 0, line);

    oled_flush();
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

static void spi_init(void)
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

    spi_init();
    oled_init();   // ← inicializa I2C + OLED + verifica DS3231

    // ── (Opcional) Acerto inicial do RTC — descomente se necessário ──────────
    // ds3231_time_t t = { .seconds=0, .minutes=30, .hours=14,
    //                     .day=2, .date=10, .month=6, .year=25 };
    // ds3231_set_time(&t);

    oled_clear();
    oled_draw_str(0, 0, "LoRa Transmissor");
    oled_draw_str(2, 0, "Inicializando...");
    oled_flush();

    if (!lora_init()) {
        ESP_LOGE(TAG, "Falha ao inicializar LoRa. Travando.");
        oled_clear();
        oled_draw_str(0, 0, "ERRO LoRa!");
        oled_draw_str(2, 0, "Verifique o");
        oled_draw_str(3, 0, "hardware.");
        oled_flush();
        while (1) { led_blink(10); vTaskDelay(pdMS_TO_TICKS(500)); }
    }

    oled_clear();
    oled_draw_str(0, 0, "LoRa OK!");
    oled_flush();

    led_blink(3);
    lm35_init();

    sensor_payload_t payload = { .node_id = 0x01 };

    while (1) {
        float temp = lm35_read_celsius();
        payload.temperature_x10 = (int16_t)(temp * 10);
        payload.counter++;

        // Lê RTC
        ds3231_time_t rtc;
        char ts_full[32] = "----.--.-- --:--:--";
        char ts_time[12] = "--:--:--";
        if (ds3231_get_time(&rtc) == ESP_OK) {
            ds3231_format(&rtc, ts_full, sizeof(ts_full));
            ds3231_format_time(&rtc, ts_time, sizeof(ts_time));
        }

        ESP_LOGI(TAG, "Enviando pacote #%lu | temp=%.1f°C | %s",
                 (unsigned long)payload.counter,
                 payload.temperature_x10 / 10.0f,
                 ts_full);

        // Atualiza display com hora na última linha
        oled_show_sensor(&payload, ts_time);

        gpio_set_level(PIN_LED, 1);
        lora_send((uint8_t *)&payload, sizeof(payload));
        gpio_set_level(PIN_LED, 0);

        led_blink(2);
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}
