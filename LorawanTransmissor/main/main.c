// ── Transmissor LoRa P2P - TTGO LoRa32 V2.0 ──────────────────────────────────
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define LM35_ADC_CHANNEL    ADC1_CHANNEL_6  // GPIO34
#define ADC_ATTEN           ADC_ATTEN_DB_11 // range até ~3.1V
#define ADC_WIDTH           ADC_WIDTH_BIT_12

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         adc_cali;

void lm35_init(void) {
	adc_oneshot_unit_init_cfg_t unit_cfg = {
		.unit_id = ADC_UNIT_1,
	};
	adc_oneshot_new_unit(&unit_cfg, &adc_handle);

	adc_oneshot_chan_cfg_t chan_cfg = {
		.atten    = ADC_ATTEN_DB_6,
		.bitwidth = ADC_BITWIDTH_12,
	};
	adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &chan_cfg);

	// line_fitting em vez de curve_fitting
	adc_cali_line_fitting_config_t cali_cfg = {
		.unit_id  = ADC_UNIT_1,
		.atten    = ADC_ATTEN_DB_6,
		.bitwidth = ADC_BITWIDTH_12,
	};
	adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali);
}

static const char *TAG = "LORA_TX";
float lm35_read_celsius(void) {
	 int raw = 0, mv = 0;
	// média de 64 leituras
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


// ── Pinos TTGO LoRa32 V2.0 ───────────────────────────────────────────────────
#define PIN_SCLK   5
#define PIN_MOSI   27
#define PIN_MISO   19
#define PIN_NSS    18
#define PIN_RST    14
#define PIN_DIO0   26
#define PIN_LED    25

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
#define LORA_FREQUENCY           915000000   // 915 MHz (AU915 para Brasil)
#define LORA_SYNC_WORD           0x12        // sync word privado P2P
#define TX_INTERVAL_MS           10000       // envia a cada 10 segundos

static spi_device_handle_t spi;

// ── Payload de sensores ───────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
	uint8_t  node_id;          // ID do nó transmissor
	int16_t  temperature_x10;  // ex: 253 = 25.3 °C
	uint32_t counter;          // contador de pacotes
} sensor_payload_t;

// ── SPI helpers ───────────────────────────────────────────────────────────────
static uint8_t spi_read(uint8_t reg)
{
    spi_transaction_t t = {
        .length    = 16,
        .tx_data   = { reg & 0x7F, 0x00 },
        .flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_transmit(spi, &t);
    return t.rx_data[1];
}

static void spi_write(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .length    = 16,
        .tx_data   = { reg | 0x80, val },
        .flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
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

// ── Inicialização SPI ─────────────────────────────────────────────────────────
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

// ── Reset do SX1276 ───────────────────────────────────────────────────────────
static void lora_reset(void)
{
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// ── Configura frequência ──────────────────────────────────────────────────────
static void lora_set_frequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    spi_write(REG_FRF_MSB, (frf >> 16) & 0xFF);
    spi_write(REG_FRF_MID, (frf >>  8) & 0xFF);
    spi_write(REG_FRF_LSB, (frf      ) & 0xFF);
}

// ── Inicialização LoRa ────────────────────────────────────────────────────────
static bool lora_init(void)
{
    lora_reset();

    uint8_t ver = spi_read(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 version: 0x%02X (esperado: 0x12)", ver);
    if (ver != 0x12) {
        ESP_LOGE(TAG, "SX1276 não encontrado!");
        return false;
    }

    // Sleep mode para configurar
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Frequência
    lora_set_frequency(LORA_FREQUENCY);

    // Endereços FIFO
    spi_write(REG_FIFO_TX_BASE_ADDR, 0x00);
    spi_write(REG_FIFO_RX_BASE_ADDR, 0x00);

    // LNA boost
    spi_write(REG_LNA, spi_read(REG_LNA) | 0x03);

    // Modem config:
    // BW=125kHz, CR=4/5, implicit header off
    spi_write(REG_MODEM_CONFIG_1, 0x72);
    // SF=7, CRC on
    spi_write(REG_MODEM_CONFIG_2, 0x74);
    // LNA AGC on
    spi_write(REG_MODEM_CONFIG_3, 0x04);

    // Potência TX: 17 dBm
    spi_write(REG_PA_CONFIG, 0x8F);
    spi_write(REG_PA_DAC,    0x87);

    // Preâmbulo
    spi_write(REG_PREAMBLE_MSB, 0x00);
    spi_write(REG_PREAMBLE_LSB, 0x08);

    // Sync word P2P (diferente do LoRaWAN = 0x34)
    spi_write(REG_SYNC_WORD, LORA_SYNC_WORD);

    // Standby
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    ESP_LOGI(TAG, "LoRa inicializado OK");
    return true;
}

// ── Transmite pacote ──────────────────────────────────────────────────────────
static void lora_send(const uint8_t *data, size_t len)
{
    // Standby
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

    // Aponta FIFO para TX
    spi_write(REG_FIFO_ADDR_PTR, 0x00);

    // Escreve payload no FIFO
    spi_write_buf(REG_FIFO, data, len);
    spi_write(REG_PAYLOAD_LENGTH, len);

    // Inicia TX
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    // Aguarda TX completo (flag TxDone no IRQ)
    while ((spi_read(REG_IRQ_FLAGS) & 0x08) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Limpa flags
    spi_write(REG_IRQ_FLAGS, 0xFF);
    ESP_LOGI(TAG, "Pacote enviado (%d bytes)", len);
}

// ── LED helper ────────────────────────────────────────────────────────────────
static void led_blink(int times)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(PIN_LED, 1); vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PIN_LED, 0); vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ── Task principal ────────────────────────────────────────────────────────────
void app_main(void)
{
    // LED
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    spi_init();

    if (!lora_init()) {
        ESP_LOGE(TAG, "Falha ao inicializar LoRa. Travando.");
        while (1) { led_blink(10); vTaskDelay(pdMS_TO_TICKS(500)); }
    }

    led_blink(3);

    lm35_init();

    sensor_payload_t payload = {
        .node_id = 0x01,
    };


    while (1) {
    	float temp = lm35_read_celsius();
		payload.temperature_x10 = (int16_t)(temp * 10);
		payload.counter++;

		ESP_LOGI(TAG, "Enviando pacote #%lu | temp=%.1f°C",
				 payload.counter,
				 payload.temperature_x10 / 10.0f);

		gpio_set_level(PIN_LED, 1);
		lora_send((uint8_t *)&payload, sizeof(payload));
		gpio_set_level(PIN_LED, 0);

		led_blink(2);
		vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}
