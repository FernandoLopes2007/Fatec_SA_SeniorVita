#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "nvs_flash.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "mqtt_client.h"


/* ============================================================
 * CONFIGURAÇÕES GERAIS
 * ============================================================ */

#define TAG "Sensor de Queda "

#define OLED_WIDTH              128
#define OLED_HEIGHT             64

#define COLISAO_LIMITE         2.5f
#define TEMPO_BLOQUEIO_MS      3000

#define INTERVALO_MQTT_MS      2000
#define INTERVALO_LEITURA_MS   500

/* ALARME DE TEMPERATURA */
#define TEMPERATURA_LIMITE     40.0f
#define LED_STATUS_GPIO        26
#define BUZZER_GPIO            25
#define INTERVALO_PISCA_MS     300

/* PWM para buzzer passivo no Wokwi */
#define BUZZER_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER      LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL    LEDC_CHANNEL_0
#define BUZZER_LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_PWM_FREQ_HZ     2000
#define BUZZER_PWM_DUTY        512


/* ============================================================
 * GPIO OLED
 * ============================================================ */

#define OLED_I2C_PORT           I2C_NUM_1
#define OLED_SDA_GPIO           19
#define OLED_SCL_GPIO           18
#define OLED_ADDR               0x3C


/* ============================================================
 * GPIO MPU6050
 * ============================================================ */

#define MPU_I2C_PORT            I2C_NUM_0
#define MPU_SDA_GPIO            21
#define MPU_SCL_GPIO            22
#define MPU_ADDR                0x68


/* ============================================================
 * GPIO CARTÃO SD
 * ============================================================ */

#define SD_MOSI_GPIO            13
#define SD_MISO_GPIO            12
#define SD_SCLK_GPIO            14
#define SD_CS_GPIO              15


/* ============================================================
 * WIFI
 * ============================================================ */

#define WIFI_SSID               "Wokwi-GUEST"
#define WIFI_PASSWORD           ""

#define WIFI_CONNECTED_BIT      BIT0


/* ============================================================
 * MQTT / ADAFRUIT IO
 *
 * ALTERE ESTAS DUAS CONFIGURAÇÕES
 * ============================================================ */

#define MQTT_USER               "SEU_USUARIO"
#define MQTT_PASSWORD           "SEU_NOVO_TOKEN"

#define MQTT_BROKER              "mqtt://io.adafruit.com"


/* ============================================================
 * MQTT FEEDS
 * ============================================================ */

#define TOPIC_TEMP              MQTT_USER "/feeds/temperatura"
#define TOPIC_ACX               MQTT_USER "/feeds/AcelerometroX"
#define TOPIC_ACY               MQTT_USER "/feeds/AcelerometroY"
#define TOPIC_ACZ               MQTT_USER "/feeds/AcelerometroZ"
#define TOPIC_COLISAO           MQTT_USER "/feeds/Colisao"
#define TOPIC_MAGNITUDE         MQTT_USER "/feeds/Magnitude"


/* ============================================================
 * SD
 * ============================================================ */

#define SD_MOUNT_POINT          "/sd"
#define ARQUIVO_SD              "/sd/dados_acelerometro.csv"


/* ============================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================ */

static bool oled_status = false;
static bool mpu_status = false;
static bool sd_status = false;
static bool wifi_status = false;
static volatile bool mqtt_status = false;

static sdmmc_card_t *sd_card = NULL;
static esp_mqtt_client_handle_t mqtt_client = NULL;

static EventGroupHandle_t wifi_event_group;

static int contador_colisoes = 0;
static int64_t tempo_ultima_colisao = 0;
static int64_t ultimo_envio_mqtt = 0;


/* ============================================================
 * ESTRUTURA MPU6050
 * ============================================================ */

typedef struct
{
    float temperatura;
    float acx;
    float acy;
    float acz;

} mpu_data_t;


/* ============================================================
 * I2C
 * ============================================================ */

static esp_err_t i2c_init_port(
    i2c_port_t port,
    int sda,
    int scl
)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,

        .sda_io_num = sda,
        .scl_io_num = scl,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = 400000,

        .clk_flags = 0
    };

    esp_err_t ret = i2c_param_config(
        port,
        &config
    );

    if (ret != ESP_OK)
    {
        return ret;
    }

    return i2c_driver_install(
        port,
        config.mode,
        0,
        0,
        0
    );
}


/* ============================================================
 * OLED SSD1306
 * ============================================================ */

static esp_err_t oled_cmd(uint8_t cmd)
{
    uint8_t data[2];

    data[0] = 0x00;
    data[1] = cmd;

    return i2c_master_write_to_device(
        OLED_I2C_PORT,
        OLED_ADDR,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );
}


static esp_err_t oled_data(
    const uint8_t *data,
    size_t len
)
{
    uint8_t buffer[17];

    buffer[0] = 0x40;

    while (len > 0)
    {
        size_t chunk = len;

        if (chunk > 16)
        {
            chunk = 16;
        }

        memcpy(
            &buffer[1],
            data,
            chunk
        );

        esp_err_t ret =
            i2c_master_write_to_device(
                OLED_I2C_PORT,
                OLED_ADDR,
                buffer,
                chunk + 1,
                pdMS_TO_TICKS(100)
            );

        if (ret != ESP_OK)
        {
            return ret;
        }

        data += chunk;
        len -= chunk;
    }

    return ESP_OK;
}


/* ============================================================
 * FONTE 5x7 OLED
 * ============================================================ */

static const uint8_t font5x7[][5] =
{
    [0] = {0,0,0,0,0},

    ['0'] = {0x3E,0x51,0x49,0x45,0x3E},
    ['1'] = {0x00,0x42,0x7F,0x40,0x00},
    ['2'] = {0x42,0x61,0x51,0x49,0x46},
    ['3'] = {0x21,0x41,0x45,0x4B,0x31},
    ['4'] = {0x18,0x14,0x12,0x7F,0x10},
    ['5'] = {0x27,0x45,0x45,0x45,0x39},
    ['6'] = {0x3C,0x4A,0x49,0x49,0x30},
    ['7'] = {0x01,0x71,0x09,0x05,0x03},
    ['8'] = {0x36,0x49,0x49,0x49,0x36},
    ['9'] = {0x06,0x49,0x49,0x29,0x1E},

    ['A'] = {0x7E,0x11,0x11,0x11,0x7E},
    ['B'] = {0x7F,0x49,0x49,0x49,0x36},
    ['C'] = {0x3E,0x41,0x41,0x41,0x22},
    ['D'] = {0x7F,0x41,0x41,0x22,0x1C},
    ['E'] = {0x7F,0x49,0x49,0x49,0x41},
    ['F'] = {0x7F,0x09,0x09,0x09,0x01},
    ['G'] = {0x3E,0x41,0x49,0x49,0x7A},
    ['H'] = {0x7F,0x08,0x08,0x08,0x7F},
    ['I'] = {0x00,0x41,0x7F,0x41,0x00},
    ['J'] = {0x20,0x40,0x41,0x3F,0x01},
    ['K'] = {0x7F,0x08,0x14,0x22,0x41},
    ['L'] = {0x7F,0x40,0x40,0x40,0x40},
    ['M'] = {0x7F,0x02,0x0C,0x02,0x7F},
    ['N'] = {0x7F,0x04,0x08,0x10,0x7F},
    ['O'] = {0x3E,0x41,0x41,0x41,0x3E},
    ['P'] = {0x7F,0x09,0x09,0x09,0x06},
    ['Q'] = {0x3E,0x41,0x51,0x21,0x5E},
    ['R'] = {0x7F,0x09,0x19,0x29,0x46},
    ['S'] = {0x46,0x49,0x49,0x49,0x31},
    ['T'] = {0x01,0x01,0x7F,0x01,0x01},
    ['U'] = {0x3F,0x40,0x40,0x40,0x3F},
    ['V'] = {0x1F,0x20,0x40,0x20,0x1F},
    ['W'] = {0x7F,0x20,0x18,0x20,0x7F},
    ['X'] = {0x63,0x14,0x08,0x14,0x63},
    ['Y'] = {0x07,0x08,0x70,0x08,0x07},
    ['Z'] = {0x61,0x51,0x49,0x45,0x43},

    ['a'] = {0x20,0x54,0x54,0x54,0x78},
    ['b'] = {0x7F,0x48,0x44,0x44,0x38},
    ['c'] = {0x38,0x44,0x44,0x44,0x20},
    ['d'] = {0x38,0x44,0x44,0x48,0x7F},
    ['e'] = {0x38,0x54,0x54,0x54,0x18},
    ['f'] = {0x08,0x7E,0x09,0x01,0x02},
    ['g'] = {0x0C,0x52,0x52,0x52,0x3E},
    ['h'] = {0x7F,0x08,0x04,0x04,0x78},
    ['i'] = {0x00,0x44,0x7D,0x40,0x00},
    ['j'] = {0x20,0x40,0x44,0x3D,0x00},
    ['k'] = {0x7F,0x10,0x28,0x44,0x00},
    ['l'] = {0x00,0x41,0x7F,0x40,0x00},
    ['m'] = {0x7C,0x04,0x18,0x04,0x78},
    ['n'] = {0x7C,0x08,0x04,0x04,0x78},
    ['o'] = {0x38,0x44,0x44,0x44,0x38},
    ['p'] = {0x7C,0x14,0x14,0x14,0x08},
    ['q'] = {0x08,0x14,0x14,0x18,0x7C},
    ['r'] = {0x7C,0x08,0x04,0x04,0x08},
    ['s'] = {0x48,0x54,0x54,0x54,0x20},
    ['t'] = {0x04,0x3F,0x44,0x40,0x20},
    ['u'] = {0x3C,0x40,0x40,0x20,0x7C},
    ['v'] = {0x1C,0x20,0x40,0x20,0x1C},
    ['w'] = {0x3C,0x40,0x30,0x40,0x3C},
    ['x'] = {0x44,0x28,0x10,0x28,0x44},
    ['y'] = {0x0C,0x50,0x50,0x50,0x3C},
    ['z'] = {0x44,0x64,0x54,0x4C,0x44},

    [':'] = {0x00,0x36,0x36,0x00,0x00},
    ['.'] = {0x00,0x60,0x60,0x00,0x00},
    ['-'] = {0x08,0x08,0x08,0x08,0x08},
    ['+'] = {0x08,0x08,0x3E,0x08,0x08},
    ['!'] = {0x00,0x00,0x5F,0x00,0x00},
    ['?'] = {0x02,0x01,0x51,0x09,0x06}
};


/* Bitmap original 128x120 (1920 bytes), extraido do sketch OLED. */
static const uint8_t logo_bitmap_128x120[1920] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xf0, 0x01, 0xff, 0x00, 0x01, 0xff, 0x80, 0x1f, 0xff, 0xf0, 0x03, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xc0, 0x00, 0x7f, 0x00, 0x01, 0xfe, 0x00, 0x07, 0xff, 0xc0, 0x00, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0x80, 0x00, 0x3f, 0x00, 0x01, 0xfc, 0x00, 0x03, 0xff, 0x80, 0x00, 0x7f,
    0xf8, 0x3f, 0xff, 0xff, 0x00, 0x00, 0x3f, 0x00, 0x01, 0xf8, 0x00, 0x01, 0xff, 0x00, 0x00, 0x3f,
    0xf8, 0x1f, 0xff, 0xfe, 0x07, 0xf8, 0x1f, 0xe0, 0x7f, 0xf0, 0x1f, 0x80, 0xfe, 0x03, 0xf0, 0x3f,
    0xf8, 0x00, 0x00, 0x7e, 0x0f, 0xfc, 0x1f, 0xe0, 0x7f, 0xf0, 0x7f, 0xe0, 0xfe, 0x0f, 0xf8, 0x1f,
    0xf8, 0x00, 0x00, 0x7e, 0x0f, 0xfc, 0x1f, 0xe0, 0x7f, 0xf0, 0x7f, 0xe0, 0xfc, 0x0f, 0xfc, 0x1f,
    0xf8, 0x00, 0x00, 0x7e, 0x0f, 0xfe, 0x0f, 0xe0, 0x7f, 0xe0, 0xff, 0xf0, 0xfc, 0x1f, 0xfc, 0x1f,
    0xf8, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x0f, 0xe0, 0x7f, 0xe0, 0xff, 0xf0, 0x7c, 0x1f, 0xfc, 0x1f,
    0xf8, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x0f, 0xe0, 0x7f, 0xe0, 0xff, 0xf0, 0x7c, 0x1f, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x0f, 0xe0, 0x7f, 0xe0, 0x00, 0x00, 0x7c, 0x1f, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xc0, 0x00, 0x0f, 0xe0, 0x7f, 0xe0, 0x00, 0x00, 0x7c, 0x1f, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0x00, 0x00, 0x0f, 0xe0, 0x7f, 0xe0, 0x00, 0x00, 0x7c, 0x1f, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x0f, 0xe0, 0x7f, 0xe0, 0x00, 0x00, 0x7c, 0x1f, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xfe, 0x0f, 0xfe, 0x0f, 0xe0, 0x7f, 0xe0, 0xff, 0xff, 0xfc, 0x1f, 0xff, 0xff,
    0xf8, 0x3f, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xe0, 0x7f, 0xe0, 0xff, 0xff, 0xfc, 0x1f, 0xfc, 0x1f,
    0xf8, 0x3f, 0xff, 0xfc, 0x1f, 0xfe, 0x0f, 0xe0, 0x7f, 0xe0, 0xff, 0xff, 0xfc, 0x1f, 0xfc, 0x1f,
    0xf8, 0x3f, 0xff, 0xfc, 0x1f, 0xfc, 0x0f, 0xe0, 0x7f, 0xf0, 0x7f, 0xf0, 0x7c, 0x0f, 0xfc, 0x1f,
    0xf8, 0x3f, 0xff, 0xfc, 0x1f, 0xf0, 0x0f, 0xf0, 0x7f, 0xf0, 0x7f, 0xe0, 0xfe, 0x0f, 0xf8, 0x1f,
    0xf8, 0x3f, 0xff, 0xfc, 0x0f, 0xc0, 0x0f, 0xf0, 0x0f, 0xf0, 0x3f, 0xc0, 0xfe, 0x03, 0xf0, 0x3f,
    0xf8, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0xf8, 0x00, 0x01, 0xff, 0x00, 0x00, 0x3f,
    0xf8, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0xfc, 0x00, 0x01, 0xff, 0x80, 0x00, 0x7f,
    0xf8, 0x3f, 0xff, 0xff, 0x00, 0x0e, 0x0f, 0xfe, 0x00, 0xfe, 0x00, 0x07, 0xff, 0xc0, 0x00, 0xff,
    0xf8, 0x3f, 0xff, 0xff, 0xc0, 0x3e, 0x0f, 0xff, 0x80, 0xff, 0x80, 0x1f, 0xff, 0xf0, 0x03, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xfb, 0xff, 0x3f,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0x9f, 0xff, 0xfd, 0xff, 0xff, 0x8f, 0xff, 0xfb, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf9, 0xff, 0xff, 0x2f, 0xff, 0xfb, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xfc, 0x38, 0x10, 0x61, 0xff, 0x6f, 0x03, 0x83, 0x0c, 0x1f,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3b, 0xd9, 0xdd, 0xce, 0xfe, 0x67, 0x3b, 0x73, 0x39, 0xdf,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xd9, 0xdd, 0xde, 0xfe, 0xf7, 0x7b, 0x7b, 0x39, 0xdf,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdc, 0x19, 0xdd, 0xde, 0xfe, 0x67, 0x7b, 0x7b, 0x39, 0xdf,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xdb, 0xdb, 0xdd, 0xde, 0xfc, 0x03, 0x7b, 0x7b, 0x39, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x19, 0x19, 0xdc, 0xe4, 0xfc, 0xfb, 0x7b, 0x03, 0x3c, 0x9f,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7c, 0xdf, 0xde, 0x73, 0xfd, 0xfb, 0x7b, 0xdb, 0xfe, 0x3f,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};


/* ============================================================
 * ICONE WIFI - BITMAPS DO EXEMPLO ORIGINAL
 * O indicador representa a conexao MQTT, nao somente o Wi-Fi.
 * ============================================================ */

static const uint8_t icon_wifi[32] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF0,
    0x3C, 0x3C, 0x60, 0x06, 0x07, 0xE0, 0x1C, 0x38,
    0x10, 0x08, 0x01, 0x80, 0x07, 0xE0, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t icon_wifi_off[32] = {
    0x80, 0x00, 0xC0, 0x00, 0x60, 0x00, 0x39, 0xF0,
    0x38, 0x3C, 0x6C, 0x06, 0x07, 0x00, 0x1F, 0x18,
    0x11, 0xC8, 0x01, 0xE0, 0x07, 0xF0, 0x00, 0x38,
    0x00, 0x0C, 0x01, 0x86, 0x00, 0x03, 0x00, 0x01
};


/* ============================================================
 * BUFFER OLED
 * ============================================================ */

static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];


static void oled_clear_buffer(void)
{
    memset(
        oled_buffer,
        0,
        sizeof(oled_buffer)
    );
}


static void oled_pixel(
    int x,
    int y,
    bool color
)
{
    if (x < 0 || x >= OLED_WIDTH ||
        y < 0 || y >= OLED_HEIGHT)
    {
        return;
    }

    int index =
        x + (y / 8) * OLED_WIDTH;

    if (color)
    {
        oled_buffer[index] |=
            (1 << (y & 7));
    }
    else
    {
        oled_buffer[index] &=
            ~(1 << (y & 7));
    }
}


/* Desenha um bitmap monocromatico de 16x16 pixels no buffer OLED.
 * Os bits seguem a mesma ordem usada por Adafruit_GFX drawBitmap:
 * bit mais significativo primeiro; bit 1 acende o pixel.
 */
static void oled_draw_icon_16x16(int x, int y, const uint8_t *bitmap)
{
    for (int row = 0; row < 16; row++)
    {
        for (int col = 0; col < 16; col++)
        {
            uint8_t byte = bitmap[row * 2 + (col / 8)];
            bool pixel_aceso = (byte & (0x80U >> (col & 7))) != 0;

            if (pixel_aceso)
            {
                oled_pixel(x + col, y + row, true);
            }
        }
    }
}


static void oled_char(
    int x,
    int y,
    char c
)
{
    if ((unsigned char)c >= 128)
    {
        c = '?';
    }

    const uint8_t *glyph =
        font5x7[(unsigned char)c];

    for (int col = 0; col < 5; col++)
    {
        uint8_t bits = glyph[col];

        for (int row = 0; row < 7; row++)
        {
            if (bits & (1 << row))
            {
                oled_pixel(
                    x + col,
                    y + row,
                    true
                );
            }
        }
    }
}


static void oled_text(
    int x,
    int y,
    const char *text
)
{
    while (*text)
    {
        oled_char(
            x,
            y,
            *text
        );

        x += 6;

        text++;
    }
}


/* Fonte 5x7 ampliada para uma matriz de 8x10 pixels por caractere.
 * O espacamento horizontal e 9 pixels; descricoes e valores usam
 * exatamente a mesma escala, conforme solicitado. */
static void oled_char_medio(int x, int y, char c)
{
    if ((unsigned char)c >= 128) c = '?';
    const uint8_t *glyph = font5x7[(unsigned char)c];

    for (int dy = 0; dy < 10; dy++)
    {
        int sy = (dy * 7) / 10;
        for (int dx = 0; dx < 8; dx++)
        {
            int sx = (dx * 5) / 8;
            if (glyph[sx] & (1U << sy))
            {
                oled_pixel(x + dx, y + dy, true);
            }
        }
    }
}

static void oled_text_medio_centralizado(int y, const char *texto)
{
    size_t n = strlen(texto);
    int largura = n ? ((int)n * 9 - 1) : 0;
    int x = (OLED_WIDTH - largura) / 2;
    if (x < 0) x = 0;

    for (size_t i = 0; i < n; i++)
    {
        oled_char_medio(x, y, texto[i]);
        x += 9;
    }
}

/* Fonte condensada para Inicializando: 6x10, com altura equivalente a descricao/valor. */
static void oled_char_pequeno_ampliado(int x, int y, char c)
{
    if ((unsigned char)c >= 128) c = '?';
    const uint8_t *glyph = font5x7[(unsigned char)c];

    for (int dy = 0; dy < 10; dy++)
    {
        int sy = (dy * 7) / 10;
        for (int dx = 0; dx < 6; dx++)
        {
            int sx = (dx * 5) / 6;
            if (glyph[sx] & (1U << sy))
            {
                oled_pixel(x + dx, y + dy, true);
            }
        }
    }
}

static void oled_text_inicializacao_centralizado(int y, const char *texto)
{
    size_t n = strlen(texto);
    int largura = n ? ((int)n * 7 - 1) : 0;
    int x = (OLED_WIDTH - largura) / 2;
    if (x < 0) x = 0;

    for (size_t i = 0; i < n; i++)
    {
        oled_char_pequeno_ampliado(x, y, texto[i]);
        x += 7;
    }
}


static esp_err_t oled_refresh(void)
{
    for (int page = 0; page < 8; page++)
    {
        esp_err_t ret;

        ret = oled_cmd(
            0xB0 + page
        );

        if (ret != ESP_OK)
        {
            return ret;
        }

        ret = oled_cmd(0x00);

        if (ret != ESP_OK)
        {
            return ret;
        }

        ret = oled_cmd(0x10);

        if (ret != ESP_OK)
        {
            return ret;
        }

        ret = oled_data(
            &oled_buffer[
                page * OLED_WIDTH
            ],
            OLED_WIDTH
        );

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    return ESP_OK;
}


static bool oled_init(void)
{
    if (i2c_init_port(
            OLED_I2C_PORT,
            OLED_SDA_GPIO,
            OLED_SCL_GPIO) != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Falha ao inicializar I2C OLED"
        );

        return false;
    }


    const uint8_t init_commands[] =
    {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x02,  /* Enderecamento por pagina, usado em oled_refresh(). */
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF
    };


    for (size_t i = 0;
         i < sizeof(init_commands);
         i++)
    {
        if (oled_cmd(
                init_commands[i]) != ESP_OK)
        {
            return false;
        }
    }


    oled_clear_buffer();

    oled_refresh();

    return true;
}


/* ============================================================
 * BITMAP E SEQUENCIA DE INICIALIZACAO OLED
 * ============================================================ */

static void oled_draw_bitmap(int x, int y, const uint8_t *bitmap,
                             int width, int height)
{
    const int bytes_por_linha = (width + 7) / 8;

    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            uint8_t byte = bitmap[row * bytes_por_linha + (col / 8)];
            bool bit_ativo = (byte & (0x80U >> (col & 7))) != 0;

            /* Mesma polaridade do sketch Arduino:
             * bit 1 = preto; bit 0 = branco. */
            oled_pixel(x + col, y + row, !bit_ativo);
        }
    }
}

static void oled_exibir_logo(void)
{
    if (!oled_status) return;

    oled_clear_buffer();
    /* Bitmap permanece 128x120. O painel 128x64 mostra a janela
     * centralizada, com deslocamento vertical -28, sem alterar o array. */
    oled_draw_bitmap(0, -28, logo_bitmap_128x120, 128, 120);

    if (oled_refresh() != ESP_OK)
    {
        ESP_LOGW(TAG, "Erro ao exibir logotipo no OLED");
    }
}

static void oled_exibir_inicializando(void)
{
    if (!oled_status) return;

    oled_clear_buffer();
    oled_text_inicializacao_centralizado(28, "Inicializando...");

    if (oled_refresh() != ESP_OK)
    {
        ESP_LOGW(TAG, "Erro ao exibir inicializacao no OLED");
    }
}


/* ============================================================
 * ATUALIZAR OLED - UMA INFORMACAO POR TELA
 * Cada pagina permanece visivel por 1 segundo.
 * ============================================================ */

static void atualizar_oled(
    float temperatura,
    float acx,
    float acy,
    float acz,
    bool colisao,
    float magnitude,
    int contador,
    bool leitura_valida
)
{
    if (!oled_status) return;

    static const int TOTAL_PAGINAS = 7;
    static int pagina = 0;
    static bool pagina_iniciada = false;
    static int64_t ultima_troca_ms = 0;

    int64_t agora_ms = esp_timer_get_time() / 1000;

    if (!pagina_iniciada)
    {
        ultima_troca_ms = agora_ms;
        pagina_iniciada = true;
    }
    else if ((agora_ms - ultima_troca_ms) >= 1000)
    {
        int64_t intervalos = (agora_ms - ultima_troca_ms) / 1000;
        pagina = (pagina + (int)(intervalos % TOTAL_PAGINAS)) % TOTAL_PAGINAS;
        ultima_troca_ms += intervalos * 1000;
    }

    oled_clear_buffer();

    if (!leitura_valida)
    {
        oled_text_medio_centralizado(8, "MPU6050");
        for (int x = 12; x < (OLED_WIDTH - 12); x++)
        {
            oled_pixel(x, 27, true);
        }
        oled_text_medio_centralizado(36, "SEM LEITURA");
    }
    else
    {
        char descricao[24];
        char valor[32];

        switch (pagina)
        {
            case 0:
                /* Temperatura interna do proprio MPU6050, nao ambiente. */
                snprintf(descricao, sizeof(descricao), "TEMP MPU");
                snprintf(valor, sizeof(valor), "%.1f C", temperatura);
                break;

            case 1:
                snprintf(descricao, sizeof(descricao), "ACELERACAO X");
                snprintf(valor, sizeof(valor), "%+.2f g", acx);
                break;

            case 2:
                snprintf(descricao, sizeof(descricao), "ACELERACAO Y");
                snprintf(valor, sizeof(valor), "%+.2f g", acy);
                break;

            case 3:
                snprintf(descricao, sizeof(descricao), "ACELERACAO Z");
                snprintf(valor, sizeof(valor), "%+.2f g", acz);
                break;

            case 4:
                snprintf(descricao, sizeof(descricao), "MAGNITUDE");
                snprintf(valor, sizeof(valor), "%.2f g", magnitude);
                break;

            case 5:
                snprintf(descricao, sizeof(descricao), "STATUS SISTEMA");
                snprintf(valor, sizeof(valor), "%s", colisao ? "COLISAO!" : "NORMAL");
                break;

            case 6:
            default:
                snprintf(descricao, sizeof(descricao), "TOTAL COLISOES");
                snprintf(valor, sizeof(valor), "%d", contador);
                break;
        }

        /* Descricao e valor usam a mesma fonte ampliada. */
        oled_text_medio_centralizado(7, descricao);

        /* Reta de separacao mantida. */
        for (int x = 12; x < (OLED_WIDTH - 12); x++)
        {
            oled_pixel(x, 26, true);
        }

        oled_text_medio_centralizado(39, valor);
    }

    /* Indicador fixo no canto inferior direito.
     * A conexao exibida e a do broker MQTT (Adafruit IO),
     * portanto usa mqtt_status em vez de wifi_status.
     * Esta funcao so e chamada no loop de dados, apos a inicializacao.
     */
    oled_draw_icon_16x16(
        112,
        48,
        mqtt_status ? icon_wifi : icon_wifi_off
    );

    if (oled_refresh() != ESP_OK)
    {
        ESP_LOGW(TAG, "Erro ao atualizar OLED");
    }
}


/* ============================================================
 * LED DE STATUS E ALARME DE TEMPERATURA
 * LED fixo = leitura normal; LED piscando + buzzer = acima de 40 C.
 * O buzzer usa PWM (LEDC), para funcionar tambem com buzzer passivo.
 * ============================================================ */

static void buzzer_pwm_ligar(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_PWM_DUTY);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static void buzzer_pwm_desligar(void)
{
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static bool alarme_gpio_init(void)
{
    /* O LED e GPIO comum; o pino do buzzer sera controlado pelo LEDC/PWM. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_STATUS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar o LED de status");
        return false;
    }

    ledc_timer_config_t timer_conf = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = BUZZER_LEDC_RESOLUTION,
        .timer_num = BUZZER_LEDC_TIMER,
        .freq_hz = BUZZER_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar timer PWM do buzzer: %s", esp_err_to_name(err));
        return false;
    }

    ledc_channel_config_t channel_conf = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar PWM no GPIO %d: %s", BUZZER_GPIO, esp_err_to_name(err));
        return false;
    }

    gpio_set_level(LED_STATUS_GPIO, 0);
    buzzer_pwm_desligar();
    ESP_LOGI(TAG, "LED no GPIO %d e buzzer PWM no GPIO %d inicializados", LED_STATUS_GPIO, BUZZER_GPIO);
    return true;
}

/* Chamar periodicamente no loop principal; nao usa delay(). */
static void atualizar_alarme_temperatura(float temperatura,
                                         bool leitura_valida)
{
    static bool estado_led = false;
    static bool estado_alarme_anterior = false;
    static int64_t ultima_troca_led_ms = 0;

    int64_t agora_ms = esp_timer_get_time() / 1000;

    /* Sem leitura confiavel do MPU: nao sinaliza estado normal. */
    if (!leitura_valida) {
        gpio_set_level(LED_STATUS_GPIO, 0);
        buzzer_pwm_desligar();
        estado_led = false;
        estado_alarme_anterior = false;
        return;
    }

    bool temperatura_alta = (temperatura > TEMPERATURA_LIMITE);

    if (temperatura_alta) {
        /* PWM de 2 kHz enquanto a temperatura permanecer acima de 40 C. */
        buzzer_pwm_ligar();

        /* Pisca o LED sem bloquear a leitura do sensor, SD ou MQTT. */
        if (!estado_alarme_anterior ||
            (agora_ms - ultima_troca_led_ms) >= INTERVALO_PISCA_MS) {
            estado_led = !estado_led;
            gpio_set_level(LED_STATUS_GPIO, estado_led ? 1 : 0);
            ultima_troca_led_ms = agora_ms;
        }

        if (!estado_alarme_anterior) {
            ESP_LOGW(TAG, "ALERTA: temperatura do MPU acima de %.1f C", TEMPERATURA_LIMITE);
        }
    } else {
        /* Estado normal: LED permanentemente aceso e buzzer desligado. */
        gpio_set_level(LED_STATUS_GPIO, 1);
        buzzer_pwm_desligar();
        estado_led = true;
        ultima_troca_led_ms = agora_ms;

        if (estado_alarme_anterior) {
            ESP_LOGI(TAG, "Temperatura normalizada. Alarme desligado.");
        }
    }

    estado_alarme_anterior = temperatura_alta;
}


/* ============================================================
 * MPU6050
 * ============================================================ */

static esp_err_t mpu_write(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t data[2] =
    {
        reg,
        value
    };

    return i2c_master_write_to_device(
        MPU_I2C_PORT,
        MPU_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );
}


static esp_err_t mpu_read(
    uint8_t reg,
    uint8_t *data,
    size_t len
)
{
    return i2c_master_write_read_device(
        MPU_I2C_PORT,
        MPU_ADDR,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(100)
    );
}


static bool mpu_init(void)
{
    if (i2c_init_port(
            MPU_I2C_PORT,
            MPU_SDA_GPIO,
            MPU_SCL_GPIO) != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Falha ao inicializar I2C MPU"
        );

        return false;
    }


    uint8_t whoami = 0;


    if (mpu_read(
            0x75,
            &whoami,
            1) != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "MPU6050 nao encontrado"
        );

        return false;
    }


    ESP_LOGI(
        TAG,
        "MPU6050 WHO_AM_I = 0x%02X",
        whoami
    );


    if (mpu_write(
            0x6B,
            0x00) != ESP_OK)
    {
        return false;
    }


    /* Acelerometro +/-4g (AFS_SEL=01). */
    if (mpu_write(0x1C, 0x08) != ESP_OK)
    {
        return false;
    }


    /* Giroscópio ±250 °/s */
    mpu_write(
        0x1B,
        0x00
    );


    return true;
}


/* ============================================================
 * LEITURA MPU6050
 * ============================================================ */

static bool mpu_read_data(
    mpu_data_t *data
)
{
    uint8_t buffer[14];


    if (mpu_read(
            0x3B,
            buffer,
            sizeof(buffer)) != ESP_OK)
    {
        return false;
    }


    int16_t raw_ax =
        (int16_t)(
            (buffer[0] << 8) |
            buffer[1]
        );


    int16_t raw_ay =
        (int16_t)(
            (buffer[2] << 8) |
            buffer[3]
        );


    int16_t raw_az =
        (int16_t)(
            (buffer[4] << 8) |
            buffer[5]
        );


    int16_t raw_temp =
        (int16_t)(
            (buffer[6] << 8) |
            buffer[7]
        );


    data->acx =
        raw_ax / 8192.0f;


    data->acy =
        raw_ay / 8192.0f;


    data->acz =
        raw_az / 8192.0f;


    data->temperatura =
        (raw_temp / 340.0f) +
        36.53f;


    return true;
}


/* ============================================================
 * DETECÇÃO DE COLISÃO
 * ============================================================ */

static float calcular_magnitude(
    float x,
    float y,
    float z
)
{
    return sqrtf(
        x * x +
        y * y +
        z * z
    );
}


static bool detectar_colisao(
    float magnitude
)
{
    return magnitude >
           COLISAO_LIMITE;
}


/* ============================================================
 * CARTÃO SD
 * ============================================================ */

static bool sd_init(void)
{
    esp_err_t ret;


    spi_bus_config_t bus_cfg =
    {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };


    ret = spi_bus_initialize(
        SPI2_HOST,
        &bus_cfg,
        SDSPI_DEFAULT_DMA
    );


    if (ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(
            TAG,
            "Falha SPI SD: %s",
            esp_err_to_name(ret)
        );

        return false;
    }


    sdmmc_host_t host =
        SDSPI_HOST_DEFAULT();


    host.slot = SPI2_HOST;


    sdspi_device_config_t slot_config =
        SDSPI_DEVICE_CONFIG_DEFAULT();


    slot_config.gpio_cs =
        SD_CS_GPIO;

    slot_config.host_id =
        SPI2_HOST;


    esp_vfs_fat_sdmmc_mount_config_t mount_config =
    {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };


    ret =
        esp_vfs_fat_sdspi_mount(
            SD_MOUNT_POINT,
            &host,
            &slot_config,
            &mount_config,
            &sd_card
        );


    if (ret != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Falha ao montar SD: %s",
            esp_err_to_name(ret)
        );

        return false;
    }


    ESP_LOGI(
        TAG,
        "Cartao SD inicializado"
    );


    return true;
}


/* ============================================================
 * CSV
 * ============================================================ */

static void criar_arquivo_csv(void)
{
    if (!sd_status)
    {
        return;
    }


    FILE *file =
        fopen(
            ARQUIVO_SD,
            "r"
        );


    if (file)
    {
        fclose(file);

        ESP_LOGI(
            TAG,
            "Arquivo CSV ja existe"
        );

        return;
    }


    file =
        fopen(
            ARQUIVO_SD,
            "w"
        );


    if (!file)
    {
        ESP_LOGE(
            TAG,
            "Erro ao criar CSV"
        );

        return;
    }


    fprintf(
        file,
        "Timestamp,"
        "Temperatura(C),"
        "AcX(g),"
        "AcY(g),"
        "AcZ(g),"
        "Magnitude(g),"
        "Colisao\n"
    );


    fclose(file);


    ESP_LOGI(
        TAG,
        "Arquivo CSV criado"
    );
}


/* ============================================================
 * SALVAR SD
 * ============================================================ */

static void salvar_sd(
    int64_t timestamp,
    float temperatura,
    float acx,
    float acy,
    float acz,
    float magnitude,
    bool colisao
)
{
    if (!sd_status)
    {
        return;
    }


    FILE *file =
        fopen(
            ARQUIVO_SD,
            "a"
        );


    if (!file)
    {
        ESP_LOGE(
            TAG,
            "Erro ao abrir CSV"
        );

        return;
    }


    fprintf(
        file,
        "%lld,"
        "%.2f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%.3f,"
        "%d\n",

        (long long)timestamp,
        temperatura,
        acx,
        acy,
        acz,
        magnitude,
        colisao ? 1 : 0
    );


    fclose(file);
}


/* ============================================================
 * WIFI
 * ============================================================ */

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }


    else if (
        event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_status = false;

        xEventGroupClearBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );


        ESP_LOGW(
            TAG,
            "Wi-Fi desconectado"
        );


        esp_wifi_connect();
    }


    else if (
        event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;


        ESP_LOGI(
            TAG,
            "Wi-Fi conectado - IP: "
            IPSTR,
            IP2STR(
                &event->ip_info.ip
            )
        );


        wifi_status = true;


        xEventGroupSetBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );
    }
}


static bool conectar_wifi(void)
{
    esp_err_t ret;


    wifi_event_group =
        xEventGroupCreate();


    ret =
        esp_netif_init();


    if (ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(
            TAG,
            "Erro esp_netif_init"
        );

        return false;
    }


    ret =
        esp_event_loop_create_default();


    if (ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(
            TAG,
            "Erro event loop"
        );

        return false;
    }


    esp_netif_create_default_wifi_sta();


    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );


    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL
        )
    );


    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL
        )
    );


    wifi_config_t wifi_config =
    {
        .sta =
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,

            .threshold.authmode =
                WIFI_AUTH_OPEN,

            .pmf_cfg =
            {
                .capable = true,
                .required = false
            }
        }
    };


    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_start()
    );


    ESP_LOGI(
        TAG,
        "Conectando ao Wi-Fi..."
    );


    EventBits_t bits =
        xEventGroupWaitBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(15000)
        );


    if (bits & WIFI_CONNECTED_BIT)
    {
        return true;
    }


    ESP_LOGW(
        TAG,
        "Nao foi possivel conectar ao Wi-Fi"
    );


    return false;
}


/* ============================================================
 * MQTT EVENTO
 * ============================================================ */

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data
)
{
    esp_mqtt_event_handle_t event =
        event_data;


    switch (event_id)
    {
        case MQTT_EVENT_CONNECTED:

            mqtt_status = true;

            ESP_LOGI(
                TAG,
                "MQTT conectado!"
            );

            break;


        case MQTT_EVENT_DISCONNECTED:

            mqtt_status = false;

            ESP_LOGW(
                TAG,
                "MQTT desconectado"
            );

            break;


        case MQTT_EVENT_ERROR:

            mqtt_status = false;

            ESP_LOGW(
                TAG,
                "Erro MQTT"
            );

            break;


        default:
            break;
    }


    (void)event;
}


/* ============================================================
 * CONECTAR MQTT
 * ============================================================ */

static bool conectar_mqtt(void)
{
    if (!wifi_status)
    {
        ESP_LOGW(
            TAG,
            "MQTT: Wi-Fi indisponivel"
        );

        return false;
    }


    if (mqtt_client)
    {
        esp_mqtt_client_stop(
            mqtt_client
        );

        esp_mqtt_client_destroy(
            mqtt_client
        );

        mqtt_client = NULL;
    }


    esp_mqtt_client_config_t mqtt_cfg =
    {
        .broker.address.uri =
            MQTT_BROKER,

        .credentials.username =
            MQTT_USER,

        .credentials.authentication.password =
            MQTT_PASSWORD,

        .credentials.client_id =
            "esp32_colisao_12322"
    };


    mqtt_client =
        esp_mqtt_client_init(
            &mqtt_cfg
        );


    if (!mqtt_client)
    {
        ESP_LOGE(
            TAG,
            "Nao foi possivel criar cliente MQTT"
        );

        return false;
    }


    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );


    esp_err_t ret =
        esp_mqtt_client_start(
            mqtt_client
        );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Erro ao iniciar MQTT"
        );

        return false;
    }


    return true;
}


/* ============================================================
 * PUBLICAR MQTT
 * ============================================================ */

static void mqtt_publish_value(
    const char *topic,
    const char *value
)
{
    if (!mqtt_status ||
        !mqtt_client)
    {
        return;
    }


    int msg_id =
        esp_mqtt_client_publish(
            mqtt_client,
            topic,
            value,
            0,
            0,
            0
        );


    if (msg_id < 0)
    {
        mqtt_status = false;

        ESP_LOGW(
            TAG,
            "Falha MQTT"
        );
    }
}


static void publicar_mqtt(
    float temperatura,
    float acx,
    float acy,
    float acz,
    float magnitude,
    bool colisao
)
{
    if (!mqtt_status)
    {
        return;
    }


    char valor[32];


    snprintf(
        valor,
        sizeof(valor),
        "%.2f",
        temperatura
    );

    mqtt_publish_value(
        TOPIC_TEMP,
        valor
    );


    snprintf(
        valor,
        sizeof(valor),
        "%.3f",
        acx
    );

    mqtt_publish_value(
        TOPIC_ACX,
        valor
    );


    snprintf(
        valor,
        sizeof(valor),
        "%.3f",
        acy
    );

    mqtt_publish_value(
        TOPIC_ACY,
        valor
    );


    snprintf(
        valor,
        sizeof(valor),
        "%.3f",
        acz
    );

    mqtt_publish_value(
        TOPIC_ACZ,
        valor
    );


    snprintf(
        valor,
        sizeof(valor),
        "%.3f",
        magnitude
    );

    mqtt_publish_value(
        TOPIC_MAGNITUDE,
        valor
    );


    mqtt_publish_value(
        TOPIC_COLISAO,
        colisao ? "1" : "0"
    );


    ESP_LOGI(
        TAG,
        "Dados enviados ao MQTT"
    );
}


/* ============================================================
 * MAIN
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "===================================="
    );

    ESP_LOGI(
        TAG,
        " SISTEMA DE DETECCAO DE COLISAO"
    );

    ESP_LOGI(
        TAG,
        " ESP32 + MPU6050"
    );

    ESP_LOGI(
        TAG,
        "===================================="
    );


    /* --------------------------------------------------------
     * NVS
     * -------------------------------------------------------- */

    esp_err_t ret =
        nvs_flash_init();


    if (
        ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND
    )
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ret =
            nvs_flash_init();
    }


    ESP_ERROR_CHECK(ret);

    /* --------------------------------------------------------
     * LED E BUZZER
     * -------------------------------------------------------- */
    if (!alarme_gpio_init()) {
        ESP_LOGE(TAG, "Nao foi possivel inicializar LED/buzzer");
    }

    /* --------------------------------------------------------
     * OLED
     * -------------------------------------------------------- */

    oled_status =
        oled_init();


    if (oled_status)
    {
        ESP_LOGI(
            TAG,
            "OLED inicializado com sucesso"
        );

        /* Sequencia visual de partida: logo por 2s e inicializacao por 1s. */
        oled_exibir_logo();
        vTaskDelay(pdMS_TO_TICKS(2000));

        oled_exibir_inicializando();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else
    {
        ESP_LOGW(
            TAG,
            "OLED nao encontrado"
        );
    }


    /* --------------------------------------------------------
     * MPU6050
     * -------------------------------------------------------- */

    mpu_status =
        mpu_init();


    if (mpu_status)
    {
        ESP_LOGI(
            TAG,
            "MPU6050 inicializado com sucesso"
        );
    }
    else
    {
        ESP_LOGW(
            TAG,
            "MPU6050 nao encontrado"
        );
    }


    /* --------------------------------------------------------
     * SD
     * -------------------------------------------------------- */

    sd_status =
        sd_init();


    if (sd_status)
    {
        criar_arquivo_csv();
    }


    /* --------------------------------------------------------
     * WIFI
     * -------------------------------------------------------- */

    wifi_status =
        conectar_wifi();


    /* --------------------------------------------------------
     * MQTT
     * -------------------------------------------------------- */

    if (wifi_status)
    {
        conectar_mqtt();
    }


    /* --------------------------------------------------------
     * LOOP PRINCIPAL
     * -------------------------------------------------------- */

    while (true)
    {
        int64_t agora =
            esp_timer_get_time() / 1000;


        mpu_data_t sensor =
        {
            .temperatura = 0,
            .acx = 0,
            .acy = 0,
            .acz = 0
        };


        /* ----------------------------------------------------
         * LEITURA MPU6050
         * ---------------------------------------------------- */

        bool leitura_mpu_ok = false;

        if (mpu_status)
        {
            leitura_mpu_ok = mpu_read_data(&sensor);
            if (!leitura_mpu_ok)
            {
                ESP_LOGW(TAG, "Erro ao ler MPU6050");
            }
        }

        /* ----------------------------------------------------
         * ALARME DE TEMPERATURA
         * ---------------------------------------------------- */
        atualizar_alarme_temperatura(sensor.temperatura, leitura_mpu_ok);


        /* ----------------------------------------------------
         * MAGNITUDE
         * ---------------------------------------------------- */

        float magnitude =
            calcular_magnitude(
                sensor.acx,
                sensor.acy,
                sensor.acz
            );


        bool colisao =
            leitura_mpu_ok && detectar_colisao(magnitude);


        /* ----------------------------------------------------
         * BLOQUEIO DE COLISÃO
         * ---------------------------------------------------- */

        bool colisao_real = false;


        if (colisao)
        {
            if (
                tempo_ultima_colisao == 0 ||
                agora -
                tempo_ultima_colisao
                >
                TEMPO_BLOQUEIO_MS
            )
            {
                colisao_real = true;

                contador_colisoes++;

                tempo_ultima_colisao =
                    agora;


                ESP_LOGW(
                    TAG,
                    "!!! COLISAO DETECTADA !!!"
                );


                ESP_LOGW(
                    TAG,
                    "Magnitude: %.2f g",
                    magnitude
                );


                ESP_LOGW(
                    TAG,
                    "Total: %d",
                    contador_colisoes
                );
            }
        }


        /* ----------------------------------------------------
         * OLED
         * ---------------------------------------------------- */

        atualizar_oled(
            sensor.temperatura,
            sensor.acx,
            sensor.acy,
            sensor.acz,
            colisao,
            magnitude,
            contador_colisoes,
            leitura_mpu_ok
        );


        /* ----------------------------------------------------
         * SD
         * ---------------------------------------------------- */

        if (leitura_mpu_ok)
        {
            salvar_sd(
                agora,
                sensor.temperatura,
                sensor.acx,
                sensor.acy,
                sensor.acz,
                magnitude,
                colisao_real
            );
        }


        /* ----------------------------------------------------
         * MQTT
         * ---------------------------------------------------- */

        if (
            agora -
            ultimo_envio_mqtt
            >=
            INTERVALO_MQTT_MS
        )
        {
            if (!wifi_status)
            {
                ESP_LOGW(
                    TAG,
                    "Wi-Fi indisponivel"
                );
            }


            if (!mqtt_status &&
                wifi_status)
            {
                conectar_mqtt();
            }


            if (leitura_mpu_ok)
            {
                publicar_mqtt(
                    sensor.temperatura,
                    sensor.acx,
                    sensor.acy,
                    sensor.acz,
                    magnitude,
                    colisao_real
                );
            }


            ultimo_envio_mqtt =
                agora;
        }


        /* ----------------------------------------------------
         * SERIAL
         * ---------------------------------------------------- */

        ESP_LOGI(
            TAG,
            "Temp: %.2f C | "
            "X: %.2f | "
            "Y: %.2f | "
            "Z: %.2f | "
            "Mag: %.2f g | "
            "Colisao: %s | "
            "Total: %d",

            sensor.temperatura,
            sensor.acx,
            sensor.acy,
            sensor.acz,
            magnitude,

            colisao_real
                ? "SIM"
                : "NAO",

            contador_colisoes
        );


        /* ----------------------------------------------------
         * DELAY
         * ---------------------------------------------------- */

        vTaskDelay(
            pdMS_TO_TICKS(
                INTERVALO_LEITURA_MS
            )
        );
    }
}
