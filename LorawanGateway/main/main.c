// ── Receptor LoRa P2P + MQTT - TTGO LoRa32 V2.0 - Multi-nó ──────────────────
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_timer.h"

static const char *TAG = "LORA_RX";

// ── WiFi / MQTT ───────────────────────────────────────────────────────────────
//#define WIFI_SSID              "HUAWEI SQ"
//#define WIFI_PASS              "Siqueira@10"
#define WIFI_CONNECTED_BIT     BIT0
//#define MQTT_URI               "mqtt://192.168.3.82:1883"
//#define MQTT_USER              "admin"
//#define MQTT_PASS              "hivemq"

#define WIFI_SSID      "IoT-local"
#define WIFI_PASS      "ApenasCoisas!@Local"

#define MQTT_URI    "mqtt://143.106.12.206:1883"
#define MQTT_USER      "FEEC-broker"
#define MQTT_PASS      "Iotfeecgo"

#define MQTT_TOPIC_BASE        "rafael/dados"

static EventGroupHandle_t wifi_event_group;

// ── Configuração de nós permitidos ────────────────────────────────────────────
// Adicione ou remova IDs conforme necessário.
// Se ALLOW_UNKNOWN_NODES for 1, qualquer nó é aceito (útil para testes).
#define ALLOW_UNKNOWN_NODES    0

static const uint8_t ALLOWED_NODES[] = { 0x01, 0x02 };
#define NUM_ALLOWED_NODES (sizeof(ALLOWED_NODES) / sizeof(ALLOWED_NODES[0]))

// ── Estatísticas por nó ───────────────────────────────────────────────────────
#define NODE_TIMEOUT_MS        60000   // marca nó como inativo após 60 s sem pacote

typedef struct {
    uint8_t  node_id;
    bool     active;
    uint32_t packets_received;
    uint32_t packets_missed;    // inferido a partir do counter do transmissor
    uint32_t last_counter;
    int8_t   last_snr;
    int16_t  last_rssi;
    int64_t  last_seen_us;      // esp_timer_get_time()
} node_stats_t;

static node_stats_t node_stats[8];  // suporta até 8 nós
static int node_count = 0;
static SemaphoreHandle_t stats_mutex;

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
#define REG_RX_NB_BYTES          0x13
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_SYNC_WORD            0x39
#define REG_PA_DAC               0x4D
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_VERSION              0x42

// ── Modos SX1276 ─────────────────────────────────────────────────────────────
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_RX_CONTINUOUS       0x05

// ── Configurações LoRa ────────────────────────────────────────────────────────
#define LORA_FREQUENCY           915000000
#define LORA_SYNC_WORD           0x12

static spi_device_handle_t spi;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// ── Payload ───────────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  node_id;
    int16_t  temperature_x10;
    uint32_t counter;
} sensor_payload_t;

// ── Validação de nó ───────────────────────────────────────────────────────────
static bool node_is_allowed(uint8_t id)
{
#if ALLOW_UNKNOWN_NODES
    return true;
#else
    for (size_t i = 0; i < NUM_ALLOWED_NODES; i++) {
        if (ALLOWED_NODES[i] == id) return true;
    }
    return false;
#endif
}

// ── Localiza ou cria entrada de estatísticas para um nó ───────────────────────
static node_stats_t *get_or_create_node(uint8_t id)
{
    // Busca nó existente
    for (int i = 0; i < node_count; i++) {
        if (node_stats[i].node_id == id) return &node_stats[i];
    }
    // Cria novo (se houver espaço)
    if (node_count < (int)(sizeof(node_stats) / sizeof(node_stats[0]))) {
        node_stats_t *n = &node_stats[node_count++];
        memset(n, 0, sizeof(*n));
        n->node_id      = id;
        n->last_counter = UINT32_MAX; // sentinela: primeiro pacote nunca tem "missed"
        ESP_LOGI(TAG, "Novo nó registrado: 0x%02X (total: %d)", id, node_count);
        return n;
    }
    ESP_LOGE(TAG, "Tabela de nós cheia — ignorando nó 0x%02X", id);
    return NULL;
}

// ── Atualiza estatísticas do nó ───────────────────────────────────────────────
static void update_node_stats(node_stats_t *n, uint32_t counter,
                               int8_t snr, int16_t rssi)
{
    xSemaphoreTake(stats_mutex, portMAX_DELAY);

    if (n->last_counter != UINT32_MAX && counter > n->last_counter + 1) {
        uint32_t missed = counter - n->last_counter - 1;
        n->packets_missed += missed;
        ESP_LOGW(TAG, "Nó 0x%02X: %lu pacote(s) perdido(s) (esperado %lu, recebido %lu)",
                 n->node_id, missed, n->last_counter + 1, counter);
    }

    n->active           = true;
    n->packets_received++;
    n->last_counter     = counter;
    n->last_snr         = snr;
    n->last_rssi        = rssi;
    n->last_seen_us     = esp_timer_get_time();

    xSemaphoreGive(stats_mutex);
}

// ── Publica status de todos os nós (heartbeat) ────────────────────────────────
static void publish_nodes_status(void)
{
    if (!mqtt_connected || node_count == 0) return;

    int64_t now = esp_timer_get_time();
    char json[512];
    int  pos = 0;

    pos += snprintf(json + pos, sizeof(json) - pos, "[");
    for (int i = 0; i < node_count; i++) {
        node_stats_t *n = &node_stats[i];
        bool online = (now - n->last_seen_us) < ((int64_t)NODE_TIMEOUT_MS * 1000);

        // Atualiza flag de atividade
        if (n->active && !online) {
            n->active = false;
            ESP_LOGW(TAG, "Nó 0x%02X marcado como OFFLINE (sem pacote há >%d s)",
                     n->node_id, NODE_TIMEOUT_MS / 1000);
        }

        pos += snprintf(json + pos, sizeof(json) - pos,
            "%s{\"node_id\":%u,\"online\":%s,"
            "\"pkts_ok\":%lu,\"pkts_lost\":%lu,"
            "\"last_rssi\":%d,\"last_snr\":%d}",
            (i > 0 ? "," : ""),
            n->node_id,
            online ? "true" : "false",
            n->packets_received,
            n->packets_missed,
            n->last_rssi,
            n->last_snr
        );
    }
    snprintf(json + pos, sizeof(json) - pos, "]");

    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_BASE "/nodes/status",
                            json, 0, 1, 1); // retain=1
}

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

static void spi_read_buf(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg & 0x7F;
    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi, &t);
    memcpy(buf, rx + 1, len);
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

// ── Reset / init LoRa ─────────────────────────────────────────────────────────
static void lora_reset(void)
{
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
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
    if (ver != 0x12) {
        ESP_LOGE(TAG, "SX1276 não encontrado!");
        return false;
    }

    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    lora_set_frequency(LORA_FREQUENCY);

    spi_write(REG_FIFO_TX_BASE_ADDR, 0x00);
    spi_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    spi_write(REG_LNA, spi_read(REG_LNA) | 0x03);
    spi_write(REG_MODEM_CONFIG_1, 0x72);
    spi_write(REG_MODEM_CONFIG_2, 0x74);
    spi_write(REG_MODEM_CONFIG_3, 0x04);
    spi_write(REG_PA_CONFIG,      0x8F);
    spi_write(REG_PA_DAC,         0x87);
    spi_write(REG_PREAMBLE_MSB,   0x00);
    spi_write(REG_PREAMBLE_LSB,   0x08);
    spi_write(REG_SYNC_WORD,      LORA_SYNC_WORD);
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    ESP_LOGI(TAG, "LoRa inicializado — aguardando pacotes...");
    return true;
}

// ── Lê pacote recebido ────────────────────────────────────────────────────────
static int lora_receive(uint8_t *buf, size_t max_len)
{
    uint8_t irq = spi_read(REG_IRQ_FLAGS);
    if (!(irq & 0x40)) return 0;
    spi_write(REG_IRQ_FLAGS, 0xFF);
    if (irq & 0x20) {
        ESP_LOGW(TAG, "Erro de CRC no pacote recebido");
        return -1;
    }
    uint8_t nb  = spi_read(REG_RX_NB_BYTES);
    uint8_t ptr = spi_read(REG_FIFO_RX_CURRENT_ADDR);
    if (nb > max_len) nb = max_len;
    spi_write(REG_FIFO_ADDR_PTR, ptr);
    spi_read_buf(REG_FIFO, buf, nb);
    return nb;
}

// ── LED helper ────────────────────────────────────────────────────────────────
static void led_blink(int times)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(PIN_LED, 1); vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PIN_LED, 0); vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ── WiFi ──────────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, reconectando...");
        mqtt_connected = false;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi conectado, IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h1, h2;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &h1));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &h2));

    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Aguardando conexão WiFi...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT))
        ESP_LOGE(TAG, "Timeout DHCP — continuando mesmo assim");
}

// ── MQTT ──────────────────────────────────────────────────────────────────────
static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT desconectado");
            mqtt_connected = false;
            esp_mqtt_client_reconnect(ev->client);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT erro (TLS: 0x%x)",
                     ev->error_handle->esp_tls_last_esp_err);
            break;
        default:
            break;
    }
}

static void mqtt_init(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri              = MQTT_URI,
        .credentials.client_id           = "lora_rx_001",
        .credentials.username            = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,
        .session.keepalive               = 60,
        .network.timeout_ms              = 10000,
        .network.reconnect_timeout_ms    = 5000,
    };
    mqtt_client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// ── Publica dados do sensor via MQTT ─────────────────────────────────────────
static void mqtt_publish_sensor(const sensor_payload_t *p,
                                 int8_t snr, int16_t rssi)
{
    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT não conectado, descartando pacote do nó 0x%02X", p->node_id);
        return;
    }

    char json[256];
    snprintf(json, sizeof(json),
        "{"
        "\"node_id\":%u,"
        "\"counter\":%lu,"
        "\"temperature\":%.1f,"
        "\"rssi\":%d,"
        "\"snr\":%d"
        "}",
        p->node_id,
        p->counter,
        p->temperature_x10 / 10.0f,
        rssi,
        snr
    );

    char topic[64];
    snprintf(topic, sizeof(topic), "%s/0x%02X", MQTT_TOPIC_BASE, p->node_id);

    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, json, 0, 1, 0);
    if (msg_id >= 0)
        ESP_LOGI(TAG, "MQTT → %s (msg_id=%d)", topic, msg_id);
    else
        ESP_LOGE(TAG, "Falha ao publicar MQTT para nó 0x%02X", p->node_id);
}

// ── Task: heartbeat de status dos nós a cada 30 s ────────────────────────────
static void status_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        publish_nodes_status();
    }
}
typedef struct {
    uint8_t  node_id;
    float    temperature;
    bool     valid;
} node_temp_t;

static node_temp_t node_temps[8];

static void update_node_temp(uint8_t node_id, float temp)
{
    for (int i = 0; i < node_count; i++) {
        if (node_temps[i].node_id == node_id) {
            node_temps[i].temperature = temp;
            node_temps[i].valid       = true;
            return;
        }
    }
    // Novo nó
    node_temps[node_count - 1].node_id     = node_id;
    node_temps[node_count - 1].temperature = temp;
    node_temps[node_count - 1].valid       = true;
}

static void mqtt_publish_average(void)
{
    if (!mqtt_connected) return;

    // Verifica se todos os nós permitidos têm leitura válida
    int   valid_count = 0;
    float sum         = 0.0f;

    for (size_t i = 0; i < NUM_ALLOWED_NODES; i++) {
        for (int j = 0; j < node_count; j++) {
            if (node_temps[j].node_id == ALLOWED_NODES[i] && node_temps[j].valid) {
                sum += node_temps[j].temperature;
                valid_count++;
                break;
            }
        }
    }

    if (valid_count < 2) {
        ESP_LOGW(TAG, "Média aguardando dados dos %d nós (%d/%d prontos)",
                 NUM_ALLOWED_NODES, valid_count, NUM_ALLOWED_NODES);
        return;
    }

    float avg = sum / valid_count;
    char json[64];
    snprintf(json, sizeof(json),
        "{\"temperature_avg\":%.1f,\"nodes\":%d}", avg, valid_count);

    esp_mqtt_client_publish(mqtt_client,
        MQTT_TOPIC_BASE "/average", json, 0, 1, 0);

    ESP_LOGI(TAG, "Média publicada: %.1f °C (%d nós)", avg, valid_count);
}
// ── app_main ──────────────────────────────────────────────────────────────────
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    stats_mutex = xSemaphoreCreateMutex();

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    wifi_init();
    mqtt_init();

    // Aguarda MQTT conectar (máx 10 s)
    for (int i = 0; i < 100 && !mqtt_connected; i++)
        vTaskDelay(pdMS_TO_TICKS(100));

    spi_init();

    if (!lora_init()) {
        ESP_LOGE(TAG, "Falha ao inicializar LoRa. Travando.");
        while (1) { led_blink(10); vTaskDelay(pdMS_TO_TICKS(500)); }
    }

    // Inicia task de heartbeat
    xTaskCreate(status_task, "node_status", 4096, NULL, 5, NULL);

    led_blink(3);

    sensor_payload_t payload;
    uint8_t buf[255];

    while (1) {
        int len = lora_receive(buf, sizeof(buf));

        if (len == sizeof(sensor_payload_t)) {
            memcpy(&payload, buf, sizeof(payload));

            // ── Validação de nó ───────────────────────────────────────────
            if (!node_is_allowed(payload.node_id)) {
                ESP_LOGW(TAG, "Pacote de nó desconhecido 0x%02X — ignorado",
                         payload.node_id);
                goto next;
            }

            int8_t  snr  = (int8_t)spi_read(REG_PKT_SNR_VALUE) / 4;
            int16_t rssi = spi_read(REG_PKT_RSSI_VALUE) - 157;

            // ── Estatísticas ──────────────────────────────────────────────
            node_stats_t *ns = get_or_create_node(payload.node_id);
            if (ns) update_node_stats(ns, payload.counter, snr, rssi);

            // ── Log ───────────────────────────────────────────────────────
            ESP_LOGI(TAG, "╔══════════════════════════════════╗");
            ESP_LOGI(TAG, "  Pacote #%lu do nó 0x%02X",
                     payload.counter, payload.node_id);
            ESP_LOGI(TAG, "  Temperatura : %.1f °C",
                     payload.temperature_x10 / 10.0f);
            ESP_LOGI(TAG, "  RSSI        : %d dBm", rssi);
            ESP_LOGI(TAG, "  SNR         : %d dB",  snr);
            if (ns)
                ESP_LOGI(TAG, "  Pkts ok/lost: %lu / %lu",
                         ns->packets_received, ns->packets_missed);
            ESP_LOGI(TAG, "╚══════════════════════════════════╝");

            mqtt_publish_sensor(&payload, snr, rssi);

			// ── Atualiza temperatura e publica média ──────────────────────
			update_node_temp(payload.node_id, payload.temperature_x10 / 10.0f);
			mqtt_publish_average();

			led_blink(2);

        } else if (len > 0) {
            ESP_LOGW(TAG, "Pacote inesperado: %d bytes, ignorando", len);
            ESP_LOG_BUFFER_HEX(TAG, buf, len);
        }

next:
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
