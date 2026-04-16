/*
 * Custom Watch Face - ESP32-S3-EYE
 * 240x240 LCD, LVGL 9, Real-time analog clock
 * Adds fluid simulation and IMU tilt control without breaking watch face rendering
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "bsp/esp-bsp.h"
#include "button_gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_private/esp_cache_private.h"
#include "esp_wifi.h"
#include "iot_button.h"
#include "linux/videodev2.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "app_video.h"
#include "bmi270.h"
#include "qma6100p.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_types.h"

LV_FONT_DECLARE(watch_zh_patch_14);

#define CENTER_X                120
#define CENTER_Y                120
#define FACE_R                  84
#define WATCH_SCREEN_BG_COLOR   0x020202
#define WATCH_SECOND_HAND_COLOR 0x4caf50
#define TAG                     "watch_face"
#define VALID_TIME_UNIX         1704067200
#define STATUS_TEXT_LEN         48
#define BUTTON_LONG_MS          1000
#define BUTTON_SHORT_MS         180
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1
#define WIFI_CONNECT_WAIT_MS    20000
#define NETWORK_RETRY_DELAY_MS  10000
#define NTP_START_DELAY_MS      2000
#define NTP_SYNC_WAIT_MS        1000
#define NTP_SYNC_MAX_WAIT_MS    10000
#define HTTP_TIME_TIMEOUT_MS    3000
#define NTP_SERVER_BACKUP_1     "ntp.tencent.com"
#define NTP_SERVER_BACKUP_2     "pool.ntp.org"
#define HTTP_TIME_URL_1         "http://neverssl.com/"
#define HTTP_TIME_URL_2         "http://example.com/"
#define HTTP_TIME_URL_3         "http://connect.rom.miui.com/generate_204"
#define HTTP_TIME_URL_4         "http://captive.apple.com/hotspot-detect.html"
#define HTTP_TIME_URL_5         "http://api.m.taobao.com/rest/api3.do?api=mtop.common.getTimestamp"
#define HTTP_TIME_URL_6         "http://quan.suning.com/getSysTime.do"

#define FLUID_TIMER_MS          33
#define FLUID_VIEW_W            210
#define FLUID_VIEW_H            156
#define FLUID_CANVAS_BUF_BYTES  LV_DRAW_BUF_SIZE(FLUID_VIEW_W, FLUID_VIEW_H, LV_COLOR_FORMAT_RGB565)
#define FLUID_PARTICLE_COUNT    84
#define FLUID_RADIUS            3
#define FLUID_GRAVITY_SCALE     180.0f
#define FLUID_DEFAULT_GY        120.0f
#define FLUID_DT                (1.0f / 30.0f)
#define FLUID_SUBSTEPS          2
#define FLUID_DRAG              0.995f
#define FLUID_BOUND_DAMP        0.35f
#define FLUID_REST_DIST         5.5f
#define FLUID_INTERACT_DIST     10.5f

#define TILT_BALL_COUNT         7
#define TILT_BALL_GRAVITY_SCALE 220.0f
#define TILT_BALL_DT            (1.0f / 30.0f)
#define TILT_BALL_SUBSTEPS      2
#define TILT_BALL_DRAG          0.994f
#define TILT_BALL_RESTITUTION   0.72f
#define TILT_BALL_WALL_FRICTION 0.985f
#define TILT_BALL_MAX_SPEED     260.0f

/* Camera preview defines (240x240 RGB565 via esp_driver_cam DVP) */
#define CAM_H_RES               BSP_LCD_H_RES
#define CAM_V_RES               BSP_LCD_V_RES
#define CAM_PIXEL_SIZE          2
#define CAM_NUM_BUFS            2
#define CAM_FRAME_SIZE          (CAM_H_RES * CAM_V_RES * CAM_PIXEL_SIZE)
#define CAM_ALIGN_UP(n, a)      (((n) + ((a) - 1)) & ~((a) - 1))

#define IMU_I2C_SPEED_HZ        400000
#define IMU_I2C_TIMEOUT_MS      50
#define ICM42670_ADDR_LOW       0x68
#define ICM42670_ADDR_HIGH      0x69
#define ICM42670_WHOAMI_REG     0x75
#define ICM42670_PWR_MGMT0_REG  0x1F
#define ICM42670_GYRO_CFG0_REG  0x20
#define ICM42670_ACCEL_DATA_REG 0x0B
#define ICM42670_DEVICE_ID      0x67
#define ICM42607_DEVICE_ID      0x60
#define ICM42670_ACCEL_SENS_2G  16384.0f
#define IMU_FILTER_ALPHA        0.12f
#define IMU_DEADZONE_G          0.08f
#define IMU_MIN_VALID_MAG_G     0.30f
#define IMU_MAX_VALID_MAG_G     2.50f
#define GRAVITY_BASE_ROTATE_DEG 90
#define GRAVITY_ROTATE_STEP_DEG 10

typedef enum {
    WATCH_STYLE_CLASSIC = 0,
    WATCH_STYLE_EMERALD,
    WATCH_STYLE_BLUE,
    WATCH_STYLE_CHAMPAGNE,
    WATCH_STYLE_BURGUNDY,
    WATCH_STYLE_COUNT,
} watch_style_t;

typedef enum {
    APP_MODE_WATCH = 0,
    APP_MODE_TILT_BALL,
    APP_MODE_FLUID,
    APP_MODE_CAMERA,
    APP_MODE_COUNT,
} app_mode_t;

typedef struct {
    uint32_t bg_color;
    uint32_t ring_color;
    uint32_t tick_major_color;
    uint32_t tick_minor_color;
    uint32_t hour_color;
    uint32_t minute_color;
    uint32_t second_color;
    uint32_t time_color;
    uint32_t status_color;
    uint32_t panel_bg_color;
} watch_theme_t;

typedef enum {
    WATCH_BUTTON_BOOT = 0,
    WATCH_BUTTON_MENU,
    WATCH_BUTTON_PLAY,
    WATCH_BUTTON_UP,
    WATCH_BUTTON_DOWN,
    WATCH_BUTTON_COUNT,
} watch_button_id_t;

typedef struct {
    watch_button_id_t id;
    bsp_button_t bsp_id;
    const char *name;
    button_handle_t handle;
} watch_button_t;

typedef enum {
    IMU_DRIVER_NONE = 0,
    IMU_DRIVER_QMA6100P,
    IMU_DRIVER_BMI270,
    IMU_DRIVER_ICM42670,
} imu_driver_t;

typedef struct {
    float x;
    float y;
    float z;
} imu_sample_t;

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
} fluid_particle_t;

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
    float radius;
    float mass;
} tilt_ball_t;

typedef enum {
    TIME_SOURCE_NONE = 0,
    TIME_SOURCE_SERIAL,
    TIME_SOURCE_NTP,
    TIME_SOURCE_HTTP,
} time_source_t;

static int g_hour   = 12;
static int g_minute = 0;
static int g_second = 0;

static lv_obj_t *scr;
static lv_obj_t *hour_hand;
static lv_obj_t *min_hand;
static lv_obj_t *sec_hand;
static lv_obj_t *time_label;
static lv_obj_t *date_label;
static lv_obj_t *sync_label;
static lv_obj_t *status_label;
static lv_obj_t *fluid_canvas;
static lv_obj_t *fluid_info_label;
static lv_obj_t *camera_canvas;
static lv_timer_t *s_tick_timer;
static lv_timer_t *s_fluid_timer;
static lv_timer_t *s_camera_timer;   /* Camera refresh timer */
static TaskHandle_t s_console_task;
static TaskHandle_t s_network_task;
static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif;
static watch_style_t s_watch_style = WATCH_STYLE_CLASSIC;
static app_mode_t s_app_mode = APP_MODE_WATCH;
static time_source_t s_time_source = TIME_SOURCE_NONE;
static char s_status_text[STATUS_TEXT_LEN] = "BLACK GOLD";
static char s_sync_text[STATUS_TEXT_LEN] = u8"等待校时";
static char s_last_time_text[16];
static uint8_t *s_fluid_canvas_buf;
static size_t s_fluid_canvas_buf_size;
static bool s_fluid_canvas_buf_from_heap;
static LV_ATTRIBUTE_MEM_ALIGN uint8_t s_fluid_canvas_fallback[FLUID_CANVAS_BUF_BYTES];
static fluid_particle_t s_particles[FLUID_PARTICLE_COUNT];
static tilt_ball_t s_tilt_balls[TILT_BALL_COUNT];
static bool s_fluid_initialized;
static bool s_tilt_ball_initialized;
static bool s_ntp_configured;
static bool s_ntp_syncing;
static bool s_ntp_synced;
static bool s_wifi_connected;
static bool s_wifi_failed;
static int s_wifi_retry_count;
static imu_driver_t s_imu_driver = IMU_DRIVER_NONE;
static qma6100p_handle_t s_qma6100p;
static bmi270_handle_t *s_bmi270;
static i2c_master_dev_handle_t s_icm42670;
static imu_sample_t s_imu_filtered = {0.0f, -(FLUID_DEFAULT_GY / FLUID_GRAVITY_SCALE), 0.0f};
static bool s_imu_has_valid_sample;
static int s_gravity_rotation;

/* Camera preview state */
static int s_cam_fd = -1;
static uint8_t *s_cam_canvas_buf;      /* PSRAM canvas backing buffer */
static size_t s_cam_canvas_buf_size;   /* Aligned size for canvas buffer */
static bool s_camera_initialized;
static bool s_camera_streaming;
static volatile bool s_new_frame_ready; /* Camera frame received flag */

static const watch_theme_t s_themes[WATCH_STYLE_COUNT] = {
    [WATCH_STYLE_CLASSIC] = {
        .bg_color = 0x11100c,
        .ring_color = 0xc8b27a,
        .tick_major_color = 0xf5f1e6,
        .tick_minor_color = 0x675b43,
        .hour_color = 0xf5f1e6,
        .minute_color = 0xd8c18a,
        .second_color = WATCH_SECOND_HAND_COLOR,
        .time_color = 0xf0deb1,
        .status_color = 0xd8c69c,
        .panel_bg_color = 0x17140d,
    },
    [WATCH_STYLE_EMERALD] = {
        .bg_color = 0x102016,
        .ring_color = 0xcca64e,
        .tick_major_color = 0xf6f2df,
        .tick_minor_color = 0x52634b,
        .hour_color = 0xf6f2df,
        .minute_color = 0xe1c06d,
        .second_color = WATCH_SECOND_HAND_COLOR,
        .time_color = 0xe9deab,
        .status_color = 0xc7c99b,
        .panel_bg_color = 0x17241a,
    },
    [WATCH_STYLE_BLUE] = {
        .bg_color = 0x101b29,
        .ring_color = 0xd9dde4,
        .tick_major_color = 0xf7f8fb,
        .tick_minor_color = 0x58677a,
        .hour_color = 0xf7f8fb,
        .minute_color = 0xd9dde4,
        .second_color = WATCH_SECOND_HAND_COLOR,
        .time_color = 0xd7e2ef,
        .status_color = 0xbfcadd,
        .panel_bg_color = 0x16202d,
    },
    [WATCH_STYLE_CHAMPAGNE] = {
        .bg_color = 0x352715,
        .ring_color = 0xe0c072,
        .tick_major_color = 0xfff5db,
        .tick_minor_color = 0x806342,
        .hour_color = 0xfff5db,
        .minute_color = 0xf0d17a,
        .second_color = WATCH_SECOND_HAND_COLOR,
        .time_color = 0xf4e1b7,
        .status_color = 0xe6cf9f,
        .panel_bg_color = 0x3d2b18,
    },
    [WATCH_STYLE_BURGUNDY] = {
        .bg_color = 0x251118,
        .ring_color = 0xd5a06a,
        .tick_major_color = 0xfaece2,
        .tick_minor_color = 0x734852,
        .hour_color = 0xfaece2,
        .minute_color = 0xe6bf95,
        .second_color = WATCH_SECOND_HAND_COLOR,
        .time_color = 0xf1d0b7,
        .status_color = 0xdcbca9,
        .panel_bg_color = 0x2d161d,
    },
};

static watch_button_t s_buttons[WATCH_BUTTON_COUNT] = {
    { .id = WATCH_BUTTON_BOOT, .bsp_id = BSP_BUTTON_5, .name = u8"启动", .handle = NULL },
    { .id = WATCH_BUTTON_MENU, .bsp_id = BSP_BUTTON_1, .name = u8"菜单", .handle = NULL },
    { .id = WATCH_BUTTON_PLAY, .bsp_id = BSP_BUTTON_2, .name = u8"播放", .handle = NULL },
    { .id = WATCH_BUTTON_UP,   .bsp_id = BSP_BUTTON_3, .name = u8"上键", .handle = NULL },
    { .id = WATCH_BUTTON_DOWN, .bsp_id = BSP_BUTTON_4, .name = u8"下键", .handle = NULL },
};

static void build_watch_face_ui(void);
static void build_tilt_ball_ui(void);
static void build_fluid_ui(void);
static void build_camera_ui(void);
static void camera_stop_stream(void);
static bool set_system_time_from_tm(const struct tm *timeinfo);

static const watch_theme_t *get_theme(void)
{
    return &s_themes[s_watch_style];
}

static const char *get_watch_style_name(void)
{
    switch (s_watch_style) {
    case WATCH_STYLE_EMERALD:
        return "EMERALD";
    case WATCH_STYLE_BLUE:
        return "ICE BLUE";
    case WATCH_STYLE_CHAMPAGNE:
        return "CHAMPAGNE";
    case WATCH_STYLE_BURGUNDY:
        return "BURGUNDY";
    case WATCH_STYLE_CLASSIC:
    default:
        return "BLACK GOLD";
    }
}

static const lv_font_t *get_cjk_font(void)
{
#if CONFIG_LV_FONT_SOURCE_HAN_SANS_SC_14_CJK
    return &watch_zh_patch_14;
#else
    return &lv_font_montserrat_14;
#endif
}

static const char *get_imu_name(void)
{
    switch (s_imu_driver) {
    case IMU_DRIVER_QMA6100P:
        return "qma6100p";
    case IMU_DRIVER_BMI270:
        return "bmi270";
    case IMU_DRIVER_ICM42670:
        return "icm42670";
    default:
        return "none";
    }
}

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float imu_sample_magnitude(const imu_sample_t *sample)
{
    return sqrtf(sample->x * sample->x + sample->y * sample->y + sample->z * sample->z);
}

static void normalize_imu_sample(imu_sample_t *sample)
{
    float magnitude;

    if (sample == NULL) {
        return;
    }

    magnitude = imu_sample_magnitude(sample);
    if (magnitude < 0.0001f) {
        return;
    }

    sample->x /= magnitude;
    sample->y /= magnitude;
    sample->z /= magnitude;
}

static int get_total_gravity_rotation(void)
{
    return (GRAVITY_BASE_ROTATE_DEG + s_gravity_rotation) % 360;
}

static void rotate_gravity_vector(float *gx, float *gy)
{
    float x;
    float y;
    float radians;
    float cos_a;
    float sin_a;

    if (gx == NULL || gy == NULL) {
        return;
    }

    x = *gx;
    y = *gy;
    radians = (float)get_total_gravity_rotation() * 3.14159265f / 180.0f;
    cos_a = cosf(radians);
    sin_a = sinf(radians);

    *gx = x * cos_a - y * sin_a;
    *gy = x * sin_a + y * cos_a;
}

static void update_fluid_info_label(void)
{
    const char *imu_text;

    if (fluid_info_label == NULL) {
        return;
    }

    imu_text = (s_imu_driver == IMU_DRIVER_NONE) ? "no imu" : get_imu_name();
    if (s_app_mode == APP_MODE_TILT_BALL) {
        lv_label_set_text_fmt(fluid_info_label, "BALL x%d · %s · %d°",
                              TILT_BALL_COUNT, imu_text, get_total_gravity_rotation());
    } else {
        lv_label_set_text_fmt(fluid_info_label, "%s · %d°", imu_text, get_total_gravity_rotation());
    }
}

static void reset_ui_refs(void)
{
    hour_hand = NULL;
    min_hand = NULL;
    sec_hand = NULL;
    time_label = NULL;
    date_label = NULL;
    sync_label = NULL;
    status_label = NULL;
    fluid_canvas = NULL;
    fluid_info_label = NULL;
    camera_canvas = NULL;
    s_last_time_text[0] = '\0';
}

static void update_status_label(void)
{
    if (status_label == NULL) {
        return;
    }

    lv_label_set_text(status_label, s_status_text);
}

static void set_status_text(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vsnprintf(s_status_text, sizeof(s_status_text), fmt, args);
    va_end(args);

    update_status_label();
}

static void set_mode_status(void)
{
    const char *text;

    if (s_app_mode == APP_MODE_TILT_BALL) {
        text = "TILT BALLS";
    } else if (s_app_mode == APP_MODE_FLUID) {
        text = "FLUID MODE";
    } else if (s_app_mode == APP_MODE_CAMERA) {
        text = "CAMERA";
    } else {
        text = get_watch_style_name();
    }

    snprintf(s_status_text, sizeof(s_status_text), "%s", text);
    update_status_label();
}

static void build_active_ui(void)
{
    if (s_app_mode == APP_MODE_TILT_BALL) {
        build_tilt_ball_ui();
    } else if (s_app_mode == APP_MODE_FLUID) {
        build_fluid_ui();
    } else if (s_app_mode == APP_MODE_CAMERA) {
        build_camera_ui();
    } else {
        build_watch_face_ui();
    }
}

static void toggle_watch_style(void)
{
    s_watch_style = (watch_style_t)((s_watch_style + 1) % WATCH_STYLE_COUNT);
    set_mode_status();
    build_active_ui();
}

static void toggle_app_mode(void)
{
    if (s_app_mode == APP_MODE_CAMERA) {
        camera_stop_stream();
    }

    switch (s_app_mode) {
    case APP_MODE_WATCH:
        s_app_mode = APP_MODE_TILT_BALL;
        break;
    case APP_MODE_TILT_BALL:
        s_app_mode = APP_MODE_FLUID;
        break;
    case APP_MODE_CAMERA:
    case APP_MODE_FLUID:
    default:
        s_app_mode = APP_MODE_WATCH;
        break;
    }

    set_mode_status();
    build_active_ui();
}

static esp_err_t imu_i2c_add_device(uint8_t address, i2c_master_dev_handle_t *handle)
{
    const i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = IMU_I2C_SPEED_HZ,
    };

    return i2c_master_bus_add_device(bsp_i2c_get_handle(), &i2c_dev_cfg, handle);
}

static esp_err_t imu_i2c_read(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(handle, &reg, sizeof(reg), data, len, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_i2c_write(i2c_master_dev_handle_t handle, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buffer[3];

    if (len > sizeof(buffer) - 1) {
        return ESP_ERR_INVALID_ARG;
    }

    buffer[0] = reg;
    if (len > 0 && data != NULL) {
        memcpy(&buffer[1], data, len);
    }

    return i2c_master_transmit(handle, buffer, len + 1, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t try_init_qma6100p(uint8_t address)
{
    qma6100p_handle_t sensor = NULL;
    uint8_t device_id = 0;

    if (qma6100p_create(bsp_i2c_get_handle(), address, &sensor) != ESP_OK || sensor == NULL) {
        return ESP_FAIL;
    }

    if (qma6100p_get_deviceid(sensor, &device_id) != ESP_OK || device_id != QMA6100P_WHO_AM_I_VAL) {
        qma6100p_delete(sensor);
        return ESP_FAIL;
    }

    if (qma6100p_wake_up(sensor) != ESP_OK || qma6100p_config(sensor, ACCE_FS_2G) != ESP_OK) {
        qma6100p_delete(sensor);
        return ESP_FAIL;
    }

    s_qma6100p = sensor;
    s_imu_driver = IMU_DRIVER_QMA6100P;
    return ESP_OK;
}

static esp_err_t try_init_bmi270(uint8_t address)
{
    bmi270_handle_t *sensor = NULL;
    const bmi270_driver_config_t driver_config = {
        .addr = address,
        .interface = BMI270_USE_I2C,
        .i2c_bus = bsp_i2c_get_handle(),
    };
    const bmi270_config_t cfg = {
        .acce_odr = BMI270_ACC_ODR_100_HZ,
        .acce_range = BMI270_ACC_RANGE_2_G,
        .gyro_odr = BMI270_GYR_ODR_100_HZ,
        .gyro_range = BMI270_GYR_RANGE_2000_DPS,
    };

    if (bmi270_create(&driver_config, &sensor) != ESP_OK || sensor == NULL) {
        return ESP_FAIL;
    }

    if (bmi270_start(sensor, &cfg) != ESP_OK) {
        bmi270_delete(sensor);
        return ESP_FAIL;
    }

    s_bmi270 = sensor;
    s_imu_driver = IMU_DRIVER_BMI270;
    return ESP_OK;
}

static esp_err_t try_init_icm42670(uint8_t address)
{
    i2c_master_dev_handle_t sensor = NULL;
    uint8_t device_id = 0;
    uint8_t cfg_data[2] = {0x07, 0x67};
    uint8_t power_data = 0;

    if (imu_i2c_add_device(address, &sensor) != ESP_OK || sensor == NULL) {
        return ESP_FAIL;
    }

    if (imu_i2c_read(sensor, ICM42670_WHOAMI_REG, &device_id, 1) != ESP_OK ||
        (device_id != ICM42670_DEVICE_ID && device_id != ICM42607_DEVICE_ID)) {
        i2c_master_bus_rm_device(sensor);
        return ESP_FAIL;
    }

    if (imu_i2c_write(sensor, ICM42670_GYRO_CFG0_REG, cfg_data, sizeof(cfg_data)) != ESP_OK ||
        imu_i2c_read(sensor, ICM42670_PWR_MGMT0_REG, &power_data, 1) != ESP_OK) {
        i2c_master_bus_rm_device(sensor);
        return ESP_FAIL;
    }

    power_data |= 0x0F;
    if (imu_i2c_write(sensor, ICM42670_PWR_MGMT0_REG, &power_data, 1) != ESP_OK) {
        i2c_master_bus_rm_device(sensor);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    s_icm42670 = sensor;
    s_imu_driver = IMU_DRIVER_ICM42670;
    return ESP_OK;
}

static void init_imu(void)
{
    if (s_imu_driver != IMU_DRIVER_NONE) {
        return;
    }

    if (try_init_qma6100p(QMA6100P_I2C_ADDRESS) == ESP_OK ||
        try_init_qma6100p(QMA6100P_I2C_ADDRESS_1) == ESP_OK ||
        try_init_bmi270(BMI270_I2C_ADDRESS_L) == ESP_OK ||
        try_init_bmi270(BMI270_I2C_ADDRESS_H) == ESP_OK ||
        try_init_icm42670(ICM42670_ADDR_LOW) == ESP_OK ||
        try_init_icm42670(ICM42670_ADDR_HIGH) == ESP_OK) {
        ESP_LOGI(TAG, "IMU detected: %s", get_imu_name());
        return;
    }

    ESP_LOGW(TAG, "No supported IMU detected on BSP I2C bus; fluid uses fallback gravity");
}

static bool read_imu_sample(imu_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    memset(sample, 0, sizeof(*sample));

    switch (s_imu_driver) {
    case IMU_DRIVER_QMA6100P: {
        qma6100p_acce_value_t acce = {0};
        if (qma6100p_get_acce(s_qma6100p, &acce) != ESP_OK) {
            return false;
        }
        sample->x = acce.acce_x;
        sample->y = acce.acce_y;
        sample->z = acce.acce_z;
        return true;
    }
    case IMU_DRIVER_BMI270:
        return bmi270_get_acce_data(s_bmi270, &sample->x, &sample->y, &sample->z) == ESP_OK;
    case IMU_DRIVER_ICM42670: {
        uint8_t raw[6] = {0};
        if (imu_i2c_read(s_icm42670, ICM42670_ACCEL_DATA_REG, raw, sizeof(raw)) != ESP_OK) {
            return false;
        }
        sample->x = (float)(int16_t)((raw[0] << 8) | raw[1]) / ICM42670_ACCEL_SENS_2G;
        sample->y = (float)(int16_t)((raw[2] << 8) | raw[3]) / ICM42670_ACCEL_SENS_2G;
        sample->z = (float)(int16_t)((raw[4] << 8) | raw[5]) / ICM42670_ACCEL_SENS_2G;
        return true;
    }
    default:
        sample->x = 0.0f;
        sample->y = -(FLUID_DEFAULT_GY / FLUID_GRAVITY_SCALE);
        sample->z = 0.0f;
        return false;
    }
}

static void update_filtered_imu(void)
{
    imu_sample_t raw = {0.0f, -(FLUID_DEFAULT_GY / FLUID_GRAVITY_SCALE), 0.0f};
    float magnitude;

    if (s_imu_driver == IMU_DRIVER_NONE) {
        s_imu_filtered = raw;
        s_imu_has_valid_sample = false;
        return;
    }

    if (!read_imu_sample(&raw)) {
        return;
    }

    magnitude = imu_sample_magnitude(&raw);
    if (magnitude < IMU_MIN_VALID_MAG_G || magnitude > IMU_MAX_VALID_MAG_G) {
        return;
    }

    normalize_imu_sample(&raw);

    if (!s_imu_has_valid_sample) {
        s_imu_filtered = raw;
        s_imu_has_valid_sample = true;
        return;
    }

    s_imu_filtered.x += (raw.x - s_imu_filtered.x) * IMU_FILTER_ALPHA;
    s_imu_filtered.y += (raw.y - s_imu_filtered.y) * IMU_FILTER_ALPHA;
    s_imu_filtered.z += (raw.z - s_imu_filtered.z) * IMU_FILTER_ALPHA;
    normalize_imu_sample(&s_imu_filtered);
}

static void get_screen_gravity(float *gx, float *gy)
{
    float gravity_x;
    float gravity_y;
    float gravity_xy_mag;
    float active_scale;

    if (gx == NULL || gy == NULL) {
        return;
    }

    gravity_x = -s_imu_filtered.x;
    gravity_y = -s_imu_filtered.y;
    gravity_xy_mag = sqrtf(gravity_x * gravity_x + gravity_y * gravity_y);

    if (gravity_xy_mag < IMU_DEADZONE_G) {
        gravity_x = 0.0f;
        gravity_y = 0.0f;
    } else {
        active_scale = (gravity_xy_mag - IMU_DEADZONE_G) / gravity_xy_mag;
        gravity_x *= active_scale;
        gravity_y *= active_scale;
    }

    gravity_x *= FLUID_GRAVITY_SCALE;
    gravity_y *= FLUID_GRAVITY_SCALE;
    rotate_gravity_vector(&gravity_x, &gravity_y);

    *gx = clampf(gravity_x, -FLUID_GRAVITY_SCALE, FLUID_GRAVITY_SCALE);
    *gy = clampf(gravity_y, -FLUID_GRAVITY_SCALE, FLUID_GRAVITY_SCALE);
}

static void ensure_fluid_canvas_buffer(void)
{
    if (s_fluid_canvas_buf != NULL) {
        return;
    }

    s_fluid_canvas_buf_size = FLUID_CANVAS_BUF_BYTES;
    s_fluid_canvas_buf_from_heap = false;
    s_fluid_canvas_buf = heap_caps_aligned_calloc(LV_DRAW_BUF_ALIGN, 1, s_fluid_canvas_buf_size,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fluid_canvas_buf == NULL) {
        s_fluid_canvas_buf = heap_caps_aligned_calloc(LV_DRAW_BUF_ALIGN, 1, s_fluid_canvas_buf_size,
                                                      MALLOC_CAP_8BIT);
    }

    if (s_fluid_canvas_buf == NULL) {
        memset(s_fluid_canvas_fallback, 0, sizeof(s_fluid_canvas_fallback));
        s_fluid_canvas_buf = s_fluid_canvas_fallback;
        s_fluid_canvas_buf_size = sizeof(s_fluid_canvas_fallback);
        ESP_LOGW(TAG, "Use static aligned fluid canvas buffer: %p size=%u", s_fluid_canvas_buf,
                 (unsigned)s_fluid_canvas_buf_size);
    } else {
        s_fluid_canvas_buf_from_heap = true;
        ESP_LOGI(TAG, "Fluid canvas buffer ready: %p size=%u", s_fluid_canvas_buf,
                 (unsigned)s_fluid_canvas_buf_size);
    }
}

static void release_fluid_canvas_buffer(void)
{
    if (s_fluid_canvas_buf == NULL) {
        return;
    }

    if (s_fluid_canvas_buf_from_heap) {
        void *buf = s_fluid_canvas_buf;
        s_fluid_canvas_buf = NULL;
        s_fluid_canvas_buf_size = 0;
        s_fluid_canvas_buf_from_heap = false;
        heap_caps_free(buf);
        ESP_LOGI(TAG, "Fluid canvas buffer released");
        return;
    }

    s_fluid_canvas_buf = NULL;
    s_fluid_canvas_buf_size = 0;
}

static uint32_t get_tilt_ball_color(int index)
{
    const watch_theme_t *theme = get_theme();

    switch (index % TILT_BALL_COUNT) {
    case 0:
        return theme->ring_color;
    case 1:
        return theme->minute_color;
    case 2:
        return theme->tick_major_color;
    case 3:
        return theme->status_color;
    case 4:
        return WATCH_SECOND_HAND_COLOR;
    case 5:
        return theme->hour_color;
    default:
        return theme->time_color;
    }
}

static void tilt_ball_reset_particles(void)
{
    static const float start_x[TILT_BALL_COUNT] = {78.0f, 104.0f, 130.0f, 91.0f, 117.0f, 78.0f, 104.0f};
    static const float start_y[TILT_BALL_COUNT] = {36.0f, 36.0f, 36.0f, 62.0f, 62.0f, 88.0f, 88.0f};
    static const float radii[TILT_BALL_COUNT] = {11.0f, 10.0f, 9.0f, 10.0f, 9.0f, 8.0f, 8.0f};

    for (int i = 0; i < TILT_BALL_COUNT; i++) {
        tilt_ball_t *ball = &s_tilt_balls[i];

        ball->x = start_x[i];
        ball->y = start_y[i];
        ball->vx = 0.0f;
        ball->vy = 0.0f;
        ball->radius = radii[i];
        ball->mass = radii[i] * radii[i];
    }

    s_tilt_ball_initialized = true;
}

static void tilt_ball_limit_speed(tilt_ball_t *ball)
{
    float speed_sq;
    const float limit_sq = TILT_BALL_MAX_SPEED * TILT_BALL_MAX_SPEED;

    if (ball == NULL) {
        return;
    }

    speed_sq = ball->vx * ball->vx + ball->vy * ball->vy;
    if (speed_sq > limit_sq) {
        float scale = TILT_BALL_MAX_SPEED / sqrtf(speed_sq);
        ball->vx *= scale;
        ball->vy *= scale;
    }
}

static void tilt_ball_resolve_bounds(void)
{
    const float padding = 6.0f;

    for (int i = 0; i < TILT_BALL_COUNT; i++) {
        tilt_ball_t *ball = &s_tilt_balls[i];
        float min_x = ball->radius + padding;
        float max_x = FLUID_VIEW_W - ball->radius - padding;
        float min_y = ball->radius + padding;
        float max_y = FLUID_VIEW_H - ball->radius - padding;

        if (ball->x < min_x) {
            ball->x = min_x;
            if (ball->vx < 0.0f) {
                ball->vx = -ball->vx * TILT_BALL_RESTITUTION;
            }
            ball->vy *= TILT_BALL_WALL_FRICTION;
        } else if (ball->x > max_x) {
            ball->x = max_x;
            if (ball->vx > 0.0f) {
                ball->vx = -ball->vx * TILT_BALL_RESTITUTION;
            }
            ball->vy *= TILT_BALL_WALL_FRICTION;
        }

        if (ball->y < min_y) {
            ball->y = min_y;
            if (ball->vy < 0.0f) {
                ball->vy = -ball->vy * TILT_BALL_RESTITUTION;
            }
            ball->vx *= TILT_BALL_WALL_FRICTION;
        } else if (ball->y > max_y) {
            ball->y = max_y;
            if (ball->vy > 0.0f) {
                ball->vy = -ball->vy * TILT_BALL_RESTITUTION;
            }
            ball->vx *= TILT_BALL_WALL_FRICTION;
        }
    }
}

static void tilt_ball_solve_collisions(void)
{
    for (int iter = 0; iter < 2; iter++) {
        for (int i = 0; i < TILT_BALL_COUNT - 1; i++) {
            for (int j = i + 1; j < TILT_BALL_COUNT; j++) {
                tilt_ball_t *a = &s_tilt_balls[i];
                tilt_ball_t *b = &s_tilt_balls[j];
                float dx = b->x - a->x;
                float dy = b->y - a->y;
                float min_dist = a->radius + b->radius;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq >= min_dist * min_dist) {
                    continue;
                }

                float dist = sqrtf(dist_sq);
                float nx;
                float ny;
                float inv_mass_a = 1.0f / a->mass;
                float inv_mass_b = 1.0f / b->mass;
                float inv_mass_sum = inv_mass_a + inv_mass_b;
                float penetration;
                float correction;
                float rvx;
                float rvy;
                float rel_n;

                if (dist < 0.001f) {
                    nx = 1.0f;
                    ny = 0.0f;
                    dist = min_dist;
                } else {
                    nx = dx / dist;
                    ny = dy / dist;
                }

                penetration = min_dist - dist;
                correction = penetration * 0.55f / inv_mass_sum;
                a->x -= nx * correction * inv_mass_a;
                a->y -= ny * correction * inv_mass_a;
                b->x += nx * correction * inv_mass_b;
                b->y += ny * correction * inv_mass_b;

                rvx = b->vx - a->vx;
                rvy = b->vy - a->vy;
                rel_n = rvx * nx + rvy * ny;
                if (rel_n < 0.0f) {
                    float impulse = -(1.0f + TILT_BALL_RESTITUTION) * rel_n / inv_mass_sum;
                    float impulse_x = nx * impulse;
                    float impulse_y = ny * impulse;

                    a->vx -= impulse_x * inv_mass_a;
                    a->vy -= impulse_y * inv_mass_a;
                    b->vx += impulse_x * inv_mass_b;
                    b->vy += impulse_y * inv_mass_b;
                }
            }
        }

        tilt_ball_resolve_bounds();
    }
}

static void tilt_ball_sim_step(void)
{
    float gx = 0.0f;
    float gy = FLUID_DEFAULT_GY;
    float step_dt = TILT_BALL_DT / TILT_BALL_SUBSTEPS;
    float gravity_scale = TILT_BALL_GRAVITY_SCALE / FLUID_GRAVITY_SCALE;

    if (!s_tilt_ball_initialized) {
        tilt_ball_reset_particles();
    }

    update_filtered_imu();
    get_screen_gravity(&gx, &gy);
    gx *= gravity_scale;
    gy *= gravity_scale;

    for (int step = 0; step < TILT_BALL_SUBSTEPS; step++) {
        for (int i = 0; i < TILT_BALL_COUNT; i++) {
            tilt_ball_t *ball = &s_tilt_balls[i];

            ball->vx += gx * step_dt;
            ball->vy += gy * step_dt;
            ball->vx *= TILT_BALL_DRAG;
            ball->vy *= TILT_BALL_DRAG;
            ball->x += ball->vx * step_dt;
            ball->y += ball->vy * step_dt;
            tilt_ball_limit_speed(ball);
        }

        tilt_ball_solve_collisions();
        tilt_ball_resolve_bounds();
    }
}

static void render_tilt_ball_canvas(void)
{
    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_fill_dsc_t highlight_dsc;
    lv_draw_rect_dsc_t tank_dsc;
    lv_draw_rect_dsc_t outline_dsc;
    lv_layer_t layer;
    const watch_theme_t *theme = get_theme();

    if (fluid_canvas == NULL || s_fluid_canvas_buf == NULL) {
        return;
    }

    lv_canvas_fill_bg(fluid_canvas, lv_color_hex(theme->panel_bg_color), LV_OPA_COVER);
    lv_canvas_init_layer(fluid_canvas, &layer);

    lv_draw_rect_dsc_init(&tank_dsc);
    tank_dsc.bg_opa = LV_OPA_TRANSP;
    tank_dsc.border_width = 2;
    tank_dsc.border_color = lv_color_hex(theme->ring_color);
    tank_dsc.radius = 18;

    lv_area_t tank_area = {0, 0, FLUID_VIEW_W - 1, FLUID_VIEW_H - 1};
    lv_draw_rect(&layer, &tank_dsc, &tank_area);

    lv_draw_rect_dsc_init(&outline_dsc);
    outline_dsc.bg_opa = LV_OPA_TRANSP;
    outline_dsc.border_width = 1;
    outline_dsc.border_color = lv_color_hex(theme->tick_major_color);
    outline_dsc.radius = LV_RADIUS_CIRCLE;

    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.opa = LV_OPA_COVER;
    fill_dsc.radius = LV_RADIUS_CIRCLE;

    lv_draw_fill_dsc_init(&highlight_dsc);
    highlight_dsc.color = lv_color_hex(0xffffff);
    highlight_dsc.opa = LV_OPA_40;
    highlight_dsc.radius = LV_RADIUS_CIRCLE;

    for (int i = 0; i < TILT_BALL_COUNT; i++) {
        const tilt_ball_t *ball = &s_tilt_balls[i];
        int32_t x = (int32_t)lroundf(ball->x);
        int32_t y = (int32_t)lroundf(ball->y);
        int32_t radius = (int32_t)lroundf(ball->radius);
        int32_t highlight_r = (int32_t)lroundf(ball->radius * 0.45f);
        lv_area_t ball_area;
        lv_area_t highlight_area;

        if (highlight_r < 2) {
            highlight_r = 2;
        }

        fill_dsc.color = lv_color_hex(get_tilt_ball_color(i));
        ball_area.x1 = x - radius;
        ball_area.y1 = y - radius;
        ball_area.x2 = x + radius;
        ball_area.y2 = y + radius;
        lv_draw_fill(&layer, &fill_dsc, &ball_area);
        lv_draw_rect(&layer, &outline_dsc, &ball_area);

        highlight_area.x1 = x - radius / 2 - highlight_r;
        highlight_area.y1 = y - radius / 2 - highlight_r;
        highlight_area.x2 = highlight_area.x1 + highlight_r * 2;
        highlight_area.y2 = highlight_area.y1 + highlight_r * 2;
        lv_draw_fill(&layer, &highlight_dsc, &highlight_area);
    }

    lv_canvas_finish_layer(fluid_canvas, &layer);
    lv_obj_invalidate(fluid_canvas);
}

static void fluid_reset_particles(void)
{
    const int cols = 12;
    const float spacing = 8.0f;
    const float start_x = (FLUID_VIEW_W - ((cols - 1) * spacing)) * 0.5f;
    const float start_y = 18.0f;

    for (int i = 0; i < FLUID_PARTICLE_COUNT; i++) {
        int row = i / cols;
        int col = i % cols;

        s_particles[i].x = start_x + col * spacing;
        s_particles[i].y = start_y + row * spacing;
        s_particles[i].vx = 0.0f;
        s_particles[i].vy = 0.0f;
    }

    s_fluid_initialized = true;
}

static void fluid_resolve_bounds(void)
{
    const float min_x = FLUID_RADIUS + 3.0f;
    const float max_x = FLUID_VIEW_W - FLUID_RADIUS - 4.0f;
    const float min_y = FLUID_RADIUS + 3.0f;
    const float max_y = FLUID_VIEW_H - FLUID_RADIUS - 4.0f;

    for (int i = 0; i < FLUID_PARTICLE_COUNT; i++) {
        fluid_particle_t *p = &s_particles[i];

        if (p->x < min_x) {
            p->x = min_x;
            p->vx = fabsf(p->vx) * FLUID_BOUND_DAMP;
        } else if (p->x > max_x) {
            p->x = max_x;
            p->vx = -fabsf(p->vx) * FLUID_BOUND_DAMP;
        }

        if (p->y < min_y) {
            p->y = min_y;
            p->vy = fabsf(p->vy) * FLUID_BOUND_DAMP;
        } else if (p->y > max_y) {
            p->y = max_y;
            p->vy = -fabsf(p->vy) * FLUID_BOUND_DAMP;
            p->vx *= 0.96f;
        }
    }
}

static void fluid_solve_collisions(void)
{
    const float interact_dist_sq = FLUID_INTERACT_DIST * FLUID_INTERACT_DIST;

    for (int iter = 0; iter < 2; iter++) {
        for (int i = 0; i < FLUID_PARTICLE_COUNT - 1; i++) {
            for (int j = i + 1; j < FLUID_PARTICLE_COUNT; j++) {
                fluid_particle_t *a = &s_particles[i];
                fluid_particle_t *b = &s_particles[j];
                float dx = b->x - a->x;
                float dy = b->y - a->y;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq <= 0.0001f || dist_sq > interact_dist_sq) {
                    continue;
                }

                float dist = sqrtf(dist_sq);
                float nx = dx / dist;
                float ny = dy / dist;

                if (dist < FLUID_REST_DIST) {
                    float push = (FLUID_REST_DIST - dist) * 0.5f;
                    float px = nx * push;
                    float py = ny * push;

                    a->x -= px;
                    a->y -= py;
                    b->x += px;
                    b->y += py;

                    a->vx -= px * 4.0f;
                    a->vy -= py * 4.0f;
                    b->vx += px * 4.0f;
                    b->vy += py * 4.0f;
                }

                float rel_vx = b->vx - a->vx;
                float rel_vy = b->vy - a->vy;
                float rel_n = rel_vx * nx + rel_vy * ny;
                if (rel_n < 0.0f) {
                    float impulse = -rel_n * 0.04f;
                    a->vx -= nx * impulse;
                    a->vy -= ny * impulse;
                    b->vx += nx * impulse;
                    b->vy += ny * impulse;
                }
            }
        }

        fluid_resolve_bounds();
    }
}

static void fluid_sim_step(void)
{
    float gx = 0.0f;
    float gy = FLUID_DEFAULT_GY;
    float step_dt = FLUID_DT / FLUID_SUBSTEPS;

    if (!s_fluid_initialized) {
        fluid_reset_particles();
    }

    update_filtered_imu();
    get_screen_gravity(&gx, &gy);

    for (int step = 0; step < FLUID_SUBSTEPS; step++) {
        for (int i = 0; i < FLUID_PARTICLE_COUNT; i++) {
            fluid_particle_t *p = &s_particles[i];

            p->vx += gx * step_dt;
            p->vy += gy * step_dt;
            p->vx *= FLUID_DRAG;
            p->vy *= FLUID_DRAG;
            p->x += p->vx * step_dt;
            p->y += p->vy * step_dt;
        }

        fluid_solve_collisions();
        fluid_resolve_bounds();
    }
}

static void render_fluid_canvas(void)
{
    if (fluid_canvas == NULL || s_fluid_canvas_buf == NULL) {
        return;
    }

    const watch_theme_t *theme = get_theme();

    lv_canvas_fill_bg(fluid_canvas, lv_color_hex(theme->panel_bg_color), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(fluid_canvas, &layer);

    lv_draw_rect_dsc_t tank_dsc;
    lv_draw_rect_dsc_init(&tank_dsc);
    tank_dsc.bg_opa = LV_OPA_TRANSP;
    tank_dsc.border_width = 2;
    tank_dsc.border_color = lv_color_hex(theme->ring_color);
    tank_dsc.radius = 18;

    lv_area_t tank_area = {0, 0, FLUID_VIEW_W - 1, FLUID_VIEW_H - 1};
    lv_draw_rect(&layer, &tank_dsc, &tank_area);

    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.color = lv_color_hex(theme->minute_color);
    fill_dsc.opa = LV_OPA_COVER;
    fill_dsc.radius = LV_RADIUS_CIRCLE;

    for (int i = 0; i < FLUID_PARTICLE_COUNT; i++) {
        int32_t x = (int32_t)lroundf(s_particles[i].x);
        int32_t y = (int32_t)lroundf(s_particles[i].y);
        lv_area_t particle_area = {
            .x1 = x - FLUID_RADIUS,
            .y1 = y - FLUID_RADIUS,
            .x2 = x + FLUID_RADIUS,
            .y2 = y + FLUID_RADIUS,
        };
        lv_draw_fill(&layer, &fill_dsc, &particle_area);
    }

    lv_canvas_finish_layer(fluid_canvas, &layer);
    lv_obj_invalidate(fluid_canvas);
}

static bool has_wifi_credentials_configured(void)
{
    return CONFIG_WATCH_WIFI_SSID[0] != '\0';
}

static const char *weekday_text(int wday)
{
    static const char *const s_weekday_names[] = {
        u8"周日", u8"周一", u8"周二", u8"周三", u8"周四", u8"周五", u8"周六"
    };

    if (wday < 0 || wday > 6) {
        return "--";
    }

    return s_weekday_names[wday];
}

static void refresh_sync_state_text(void)
{
    time_t now = 0;
    const char *text;

    time(&now);

    if (!s_ntp_configured) {
        if (s_time_source == TIME_SOURCE_SERIAL) {
            text = u8"串口校时";
        } else if (now >= VALID_TIME_UNIX) {
            text = u8"系统时间";
        } else {
            text = u8"未配置 Wi-Fi";
        }
    } else if (s_ntp_synced) {
        text = (s_time_source == TIME_SOURCE_HTTP) ? u8"HTTP 已同步" : u8"NTP 已同步";
    } else if (s_ntp_syncing) {
        text = s_wifi_connected ? u8"NTP 同步中" : u8"连接 Wi-Fi";
    } else if (s_wifi_failed) {
        text = u8"Wi-Fi 失败";
    } else if (s_wifi_connected) {
        text = u8"等待 NTP";
    } else {
        text = u8"等待联网";
    }

    snprintf(s_sync_text, sizeof(s_sync_text), "%s", text);
}

static void log_ntp_server_status(void)
{
    unsigned int reachability = 0;

    if (esp_netif_sntp_reachability(0, &reachability) == ESP_OK) {
        ESP_LOGI(TAG, "NTP server[0]=%s reachability=0x%02x", CONFIG_WATCH_NTP_SERVER, reachability);
    }
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 1
    if (esp_netif_sntp_reachability(1, &reachability) == ESP_OK) {
        ESP_LOGI(TAG, "NTP server[1]=%s reachability=0x%02x", NTP_SERVER_BACKUP_1, reachability);
    }
#endif
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 2
    if (esp_netif_sntp_reachability(2, &reachability) == ESP_OK) {
        ESP_LOGI(TAG, "NTP server[2]=%s reachability=0x%02x", NTP_SERVER_BACKUP_2, reachability);
    }
#endif
}

static void log_dns_server(esp_netif_dns_type_t dns_type, const char *label)
{
    esp_netif_dns_info_t dns = {0};

    if (s_sta_netif == NULL || label == NULL) {
        return;
    }

    if (esp_netif_get_dns_info(s_sta_netif, dns_type, &dns) != ESP_OK) {
        return;
    }

    if (dns.ip.type == ESP_IPADDR_TYPE_V4 && dns.ip.u_addr.ip4.addr != 0) {
        ESP_LOGI(TAG, "%s DNS: " IPSTR, label, IP2STR(&dns.ip.u_addr.ip4));
    }
}

static void override_dns_server(esp_netif_dns_type_t dns_type, const char *dns_ip)
{
    esp_netif_dns_info_t dns = {0};

    if (s_sta_netif == NULL || dns_ip == NULL) {
        return;
    }

    dns.ip.type = ESP_IPADDR_TYPE_V4;
    if (esp_netif_str_to_ip4(dns_ip, &dns.ip.u_addr.ip4) != ESP_OK) {
        ESP_LOGW(TAG, "Invalid DNS server: %s", dns_ip);
        return;
    }

    if (esp_netif_set_dns_info(s_sta_netif, dns_type, &dns) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set DNS server: %s", dns_ip);
        return;
    }

    ESP_LOGI(TAG, "Override DNS server: %s", dns_ip);
}

static void apply_public_dns_servers(void)
{
    override_dns_server(ESP_NETIF_DNS_MAIN, "223.5.5.5");
    override_dns_server(ESP_NETIF_DNS_BACKUP, "114.114.114.114");
    log_dns_server(ESP_NETIF_DNS_MAIN, "Main");
    log_dns_server(ESP_NETIF_DNS_BACKUP, "Backup");
}

typedef struct {
    char date_header[48];
    char body[256];
    size_t body_len;
} http_time_context_t;

static int http_date_month_to_int(const char *month)
{
    static const char *const months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    if (month == NULL) {
        return -1;
    }

    for (int i = 0; i < 12; i++) {
        if (strcmp(month, months[i]) == 0) {
            return i;
        }
    }

    return -1;
}

static esp_err_t http_time_event_handler(esp_http_client_event_t *evt)
{
    http_time_context_t *ctx = evt ? (http_time_context_t *)evt->user_data : NULL;

    if (evt != NULL && evt->event_id == HTTP_EVENT_ON_HEADER && ctx != NULL &&
        evt->header_key != NULL && evt->header_value != NULL &&
        strcasecmp(evt->header_key, "Date") == 0) {
        strlcpy(ctx->date_header, evt->header_value, sizeof(ctx->date_header));
    }

    if (evt != NULL && evt->event_id == HTTP_EVENT_ON_DATA && ctx != NULL &&
        evt->data != NULL && evt->data_len > 0 && ctx->body_len < sizeof(ctx->body) - 1) {
        size_t copy_len = sizeof(ctx->body) - 1 - ctx->body_len;
        if ((size_t)evt->data_len < copy_len) {
            copy_len = (size_t)evt->data_len;
        }
        memcpy(ctx->body + ctx->body_len, evt->data, copy_len);
        ctx->body_len += copy_len;
        ctx->body[ctx->body_len] = '\0';
    }

    return ESP_OK;
}

static bool parse_http_date_header(const char *date_header, struct tm *timeinfo)
{
    char weekday[4] = {0};
    char month[4] = {0};
    int day;
    int year;
    int hour;
    int minute;
    int second;
    int month_index;

    if (date_header == NULL || timeinfo == NULL) {
        return false;
    }

    if (sscanf(date_header, "%3[^,], %d %3s %d %d:%d:%d GMT",
               weekday, &day, month, &year, &hour, &minute, &second) != 7) {
        return false;
    }

    month_index = http_date_month_to_int(month);
    if (month_index < 0 || day < 1 || day > 31 || hour < 0 || hour > 23 ||
        minute < 0 || minute > 59 || second < 0 || second > 59 || year < 2024) {
        return false;
    }

    *timeinfo = (struct tm) {
        .tm_year = year - 1900,
        .tm_mon = month_index,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second,
        .tm_isdst = 0,
    };

    return true;
}

static bool set_system_time_from_epoch(time_t epoch)
{
    struct timeval tv;

    if (epoch < VALID_TIME_UNIX) {
        return false;
    }

    tv.tv_sec = epoch;
    tv.tv_usec = 0;
    return settimeofday(&tv, NULL) == 0;
}

static bool set_system_time_from_utc_tm(const struct tm *timeinfo)
{
    struct tm time_copy;
    time_t epoch;

    if (timeinfo == NULL) {
        return false;
    }

    time_copy = *timeinfo;
    setenv("TZ", "UTC0", 1);
    tzset();
    epoch = mktime(&time_copy);
    setenv("TZ", CONFIG_WATCH_TIMEZONE, 1);
    tzset();

    if (epoch == (time_t)-1) {
        return false;
    }

    return set_system_time_from_epoch(epoch);
}

static bool parse_http_epoch_body(const char *body, time_t *epoch)
{
    const char *marker;
    long long timestamp_ms;

    if (body == NULL || epoch == NULL) {
        return false;
    }

    marker = strstr(body, "\"t\":\"");
    if (marker == NULL) {
        return false;
    }

    timestamp_ms = strtoll(marker + 5, NULL, 10);
    if (timestamp_ms < 1704067200000LL) {
        return false;
    }

    *epoch = (time_t)(timestamp_ms / 1000LL);
    return true;
}

static bool parse_http_local_time_body(const char *body, struct tm *timeinfo)
{
    const char *marker;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;

    if (body == NULL || timeinfo == NULL) {
        return false;
    }

    marker = strstr(body, "\"sysTime1\":\"");
    if (marker == NULL) {
        return false;
    }

    if (sscanf(marker + 12, "%4d%2d%2d%2d%2d%2d", &year, &month, &day, &hour, &minute, &second) != 6) {
        return false;
    }

    if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
        return false;
    }

    *timeinfo = (struct tm) {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second,
        .tm_isdst = -1,
    };
    return true;
}

static bool sync_time_via_http_body(const char *url, const char *body)
{
    time_t epoch = 0;
    struct tm local_time = {0};

    if (parse_http_epoch_body(body, &epoch) && set_system_time_from_epoch(epoch)) {
        s_time_source = TIME_SOURCE_HTTP;
        s_ntp_synced = true;
        s_ntp_syncing = false;
        s_wifi_failed = false;
        refresh_sync_state_text();
        ESP_LOGI(TAG, "HTTP body time sync ok: %s -> epoch=%lld", url, (long long)epoch);
        return true;
    }

    if (parse_http_local_time_body(body, &local_time) && set_system_time_from_tm(&local_time)) {
        s_time_source = TIME_SOURCE_HTTP;
        s_ntp_synced = true;
        s_ntp_syncing = false;
        s_wifi_failed = false;
        refresh_sync_state_text();
        ESP_LOGI(TAG, "HTTP body time sync ok: %s -> %04d-%02d-%02d %02d:%02d:%02d", url,
                 local_time.tm_year + 1900, local_time.tm_mon + 1, local_time.tm_mday,
                 local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
        return true;
    }

    return false;
}

static bool sync_time_via_http_url(const char *url)
{
    http_time_context_t ctx = {0};
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = HTTP_TIME_TIMEOUT_MS,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .event_handler = http_time_event_handler,
        .user_data = &ctx,
    };
    esp_http_client_handle_t client;
    esp_err_t ret;
    int status_code = -1;
    struct tm timeinfo = {0};

    client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGW(TAG, "HTTP time client init failed: %s", url);
        return false;
    }

    ret = esp_http_client_perform(client);
    if (ret == ESP_OK) {
        status_code = esp_http_client_get_status_code(client);
    }
    esp_http_client_cleanup(client);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HTTP time request failed: %s (%s)", url, esp_err_to_name(ret));
        return false;
    }

    if (ctx.date_header[0] != '\0') {
        if (!parse_http_date_header(ctx.date_header, &timeinfo)) {
            ESP_LOGW(TAG, "HTTP time Date header parse failed: %s -> %s", url, ctx.date_header);
        } else if (!set_system_time_from_utc_tm(&timeinfo)) {
            ESP_LOGW(TAG, "HTTP time apply failed: %s", url);
        } else {
            s_time_source = TIME_SOURCE_HTTP;
            s_ntp_synced = true;
            s_ntp_syncing = false;
            s_wifi_failed = false;
            refresh_sync_state_text();
            ESP_LOGI(TAG, "HTTP time sync ok: %s -> %s", url, ctx.date_header);
            return true;
        }
    } else {
        ESP_LOGW(TAG, "HTTP time Date header missing: %s status=%d", url, status_code);
    }

    if (sync_time_via_http_body(url, ctx.body)) {
        return true;
    }

    ESP_LOGW(TAG, "HTTP time body parse failed: %s body=%s", url, ctx.body[0] ? ctx.body : "<empty>");
    return false;
}

static bool sync_time_via_http_fallback(void)
{
    return sync_time_via_http_url(HTTP_TIME_URL_1) ||
           sync_time_via_http_url(HTTP_TIME_URL_2) ||
           sync_time_via_http_url(HTTP_TIME_URL_3) ||
           sync_time_via_http_url(HTTP_TIME_URL_4) ||
           sync_time_via_http_url(HTTP_TIME_URL_5) ||
           sync_time_via_http_url(HTTP_TIME_URL_6);
}

static void update_date_label(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    char buf[32];

    if (date_label == NULL) {
        return;
    }

    time(&now);
    if (now >= VALID_TIME_UNIX) {
        localtime_r(&now, &timeinfo);
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %s",
                 timeinfo.tm_year + 1900,
                 timeinfo.tm_mon + 1,
                 timeinfo.tm_mday,
                 weekday_text(timeinfo.tm_wday));
    } else if (s_ntp_configured) {
        snprintf(buf, sizeof(buf), "%s", u8"等待 NTP 对时");
    } else {
        snprintf(buf, sizeof(buf), "%s", u8"请配置 Wi-Fi");
    }

    lv_label_set_text(date_label, buf);
}

static void update_sync_label(void)
{
    if (sync_label == NULL) {
        return;
    }

    lv_label_set_text(sync_label, s_sync_text);
}

static void ntp_sync_notification_cb(struct timeval *tv)
{
    (void)tv;
    s_time_source = TIME_SOURCE_NTP;
    s_ntp_synced = true;
    s_ntp_syncing = false;
    s_wifi_failed = false;
    refresh_sync_state_text();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    const wifi_event_sta_disconnected_t *disconn = (const wifi_event_sta_disconnected_t *)event_data;

    LV_UNUSED(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_wifi_failed = false;
        s_ntp_syncing = true;
        refresh_sync_state_text();
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%d", disconn ? (int)disconn->reason : -1);
        if (s_wifi_retry_count < CONFIG_WATCH_WIFI_MAXIMUM_RETRY) {
            s_wifi_retry_count++;
            s_ntp_syncing = true;
            refresh_sync_state_text();
            esp_wifi_connect();
        } else {
            s_wifi_failed = true;
            s_ntp_syncing = false;
            if (s_wifi_event_group != NULL) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            refresh_sync_state_text();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got_ip = (const ip_event_got_ip_t *)event_data;
        esp_err_t ret;

        s_wifi_retry_count = 0;
        s_wifi_connected = true;
        s_wifi_failed = false;
        s_ntp_syncing = true;
        if (got_ip != NULL) {
            ESP_LOGI(TAG, "Got IP: " IPSTR ", gateway: " IPSTR ", netmask: " IPSTR,
                     IP2STR(&got_ip->ip_info.ip), IP2STR(&got_ip->ip_info.gw), IP2STR(&got_ip->ip_info.netmask));
        }
        apply_public_dns_servers();
        ret = esp_netif_sntp_start();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_netif_sntp_start returned: %s", esp_err_to_name(ret));
        }
        if (s_wifi_event_group != NULL) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        refresh_sync_state_text();
    }
}

static esp_err_t init_nvs_storage(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

static void network_time_task(void *arg)
{
    EventBits_t bits;
    esp_err_t ret;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {0};
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(3,
        ESP_SNTP_SERVER_LIST(CONFIG_WATCH_NTP_SERVER, NTP_SERVER_BACKUP_1, NTP_SERVER_BACKUP_2));

    LV_UNUSED(arg);

    s_ntp_configured = has_wifi_credentials_configured();
    refresh_sync_state_text();

    if (!s_ntp_configured) {
        ESP_LOGW(TAG, "Wi-Fi credentials not configured, skip NTP sync");
        s_network_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Delay NTP start for %d ms", NTP_START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(NTP_START_DELAY_MS));

    ESP_ERROR_CHECK(init_nvs_storage());

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        s_network_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop init failed: %s", esp_err_to_name(ret));
        s_network_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
    }

    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "failed to create wifi event group");
        s_network_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    s_sta_netif = esp_netif_create_default_wifi_sta();

    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        s_network_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    strlcpy((char *)wifi_config.sta.ssid, CONFIG_WATCH_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, CONFIG_WATCH_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    setenv("TZ", CONFIG_WATCH_TIMEZONE, 1);
    tzset();

    sntp_config.start = false;
    sntp_config.renew_servers_after_new_IP = true;
    sntp_config.index_of_first_server = 0;
    sntp_config.sync_cb = ntp_sync_notification_cb;
    ESP_LOGI(TAG, "NTP servers: %s, %s, %s", CONFIG_WATCH_NTP_SERVER, NTP_SERVER_BACKUP_1, NTP_SERVER_BACKUP_2);
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        s_network_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (!s_ntp_synced) {
        int ntp_wait_count = NTP_SYNC_MAX_WAIT_MS / NTP_SYNC_WAIT_MS;

        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        s_wifi_failed = false;
        s_ntp_syncing = true;
        refresh_sync_state_text();

        if (!s_wifi_connected) {
            s_wifi_retry_count = 0;
            ret = esp_wifi_connect();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "esp_wifi_connect returned: %s", esp_err_to_name(ret));
            }

            bits = xEventGroupWaitBits(s_wifi_event_group,
                                       WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                       pdFALSE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(WIFI_CONNECT_WAIT_MS));

            if ((bits & WIFI_CONNECTED_BIT) == 0) {
                s_wifi_connected = false;
                s_wifi_failed = true;
                s_ntp_syncing = false;
                refresh_sync_state_text();
                ESP_LOGW(TAG, "Wi-Fi not connected, retry in %d ms", NETWORK_RETRY_DELAY_MS);
                vTaskDelay(pdMS_TO_TICKS(NETWORK_RETRY_DELAY_MS));
                continue;
            }
        } else {
            ret = esp_netif_sntp_start();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "esp_netif_sntp_start returned: %s", esp_err_to_name(ret));
            }
        }

        while (!s_ntp_synced && s_wifi_connected && ntp_wait_count-- > 0) {
            ret = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(NTP_SYNC_WAIT_MS));
            if (ret == ESP_OK) {
                refresh_sync_state_text();
                break;
            }
            ESP_LOGI(TAG, "Waiting for NTP sync...");
        }

        if (!s_ntp_synced) {
            s_ntp_syncing = false;
            refresh_sync_state_text();
            log_ntp_server_status();
            if (sync_time_via_http_fallback()) {
                continue;
            }
            ESP_LOGW(TAG, "NTP/HTTP sync timeout, retry in %d ms", NETWORK_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(NETWORK_RETRY_DELAY_MS));
        }
    }

    s_network_task = NULL;
    vTaskDelete(NULL);
}

static bool has_valid_system_time(time_t now)
{
    return now >= VALID_TIME_UNIX;
}

static bool sync_from_system_time(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};

    time(&now);
    if (has_valid_system_time(now)) {
        localtime_r(&now, &timeinfo);
        g_hour   = timeinfo.tm_hour;
        g_minute = timeinfo.tm_min;
        g_second = timeinfo.tm_sec;
        return true;
    } else {
        ESP_LOGW(TAG, "System time not set! Showing 12:00. "
                   "Set time via: date MMDDHHMMYYYY e.g. date 040117352026");
        return false;
    }
}

static inline float deg2rad(float d)
{
    return d * 3.14159265f / 180.0f;
}

static void update_time_label(void)
{
    char buf[16];

    if (time_label == NULL) {
        return;
    }

    snprintf(buf, sizeof(buf), "%02d:%02d", g_hour, g_minute);
    if (strcmp(buf, s_last_time_text) == 0) {
        return;
    }

    lv_label_set_text(time_label, buf);
    snprintf(s_last_time_text, sizeof(s_last_time_text), "%s", buf);
}

static void init_hand(lv_obj_t *hand, int length, lv_color_t color, int width)
{
    lv_obj_set_size(hand, length, width);
    lv_obj_set_pos(hand, CENTER_X, CENTER_Y - width / 2);
    lv_obj_set_style_bg_color(hand, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(hand, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(hand, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(hand, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_x(hand, 0, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_y(hand, width / 2, LV_PART_MAIN);
}

static void set_hand_rotation(lv_obj_t *hand, float angle_deg)
{
    lv_obj_set_style_transform_rotation(hand, (int32_t)(angle_deg * 10.0f) - 900, LV_PART_MAIN);
}

static void create_marker(int radius, float angle_deg, int size, lv_color_t color)
{
    lv_obj_t *marker = lv_obj_create(scr);
    int x = CENTER_X + (int16_t)(radius * sin(deg2rad(angle_deg)));
    int y = CENTER_Y - (int16_t)(radius * cos(deg2rad(angle_deg)));

    lv_obj_remove_style_all(marker);
    lv_obj_set_size(marker, size, size);
    lv_obj_set_pos(marker, x - size / 2, y - size / 2);
    lv_obj_set_style_bg_color(marker, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(marker, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(marker, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(marker, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_remove_flag(marker, LV_OBJ_FLAG_CLICKABLE);
}

static void create_text_marker(int radius, float angle_deg, const char *text, lv_color_t color)
{
    lv_obj_t *label = lv_label_create(scr);
    int x = CENTER_X + (int16_t)(radius * sin(deg2rad(angle_deg)));
    int y = CENTER_Y - (int16_t)(radius * cos(deg2rad(angle_deg)));

    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(label, 1, LV_PART_MAIN);
    lv_obj_update_layout(label);
    lv_obj_set_pos(label, x - lv_obj_get_width(label) / 2, y - lv_obj_get_height(label) / 2);
    lv_obj_remove_flag(label, LV_OBJ_FLAG_CLICKABLE);
}

static void refresh_hands(void)
{
    float sec = (float)g_second;
    float mn  = g_minute * 6.0f + sec * 0.1f;
    float hr  = (g_hour % 12) * 30.0f + g_minute * 0.5f + sec * (0.5f / 60.0f);

    if (hour_hand == NULL || min_hand == NULL || sec_hand == NULL) {
        return;
    }

    set_hand_rotation(hour_hand, hr);
    set_hand_rotation(min_hand, mn);
    set_hand_rotation(sec_hand, sec * 6.0f);
}

static void build_watch_face_ui(void)
{
    const watch_theme_t *theme = get_theme();

    lv_obj_clean(scr);
    reset_ui_refs();
    release_fluid_canvas_buffer();
    lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_SCREEN_BG_COLOR), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *dial = lv_obj_create(scr);
    lv_obj_remove_style_all(dial);
    lv_obj_set_size(dial, FACE_R * 2 + 2, FACE_R * 2 + 2);
    lv_obj_align(dial, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(dial, lv_color_hex(theme->bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dial, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dial, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(dial, lv_color_hex(theme->tick_minor_color), LV_PART_MAIN);
    lv_obj_set_style_radius(dial, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(dial, 18, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(dial, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(dial, LV_OPA_20, LV_PART_MAIN);
    lv_obj_remove_flag(dial, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *outer_ring = lv_arc_create(scr);
    lv_obj_set_size(outer_ring, FACE_R * 2 + 16, FACE_R * 2 + 16);
    lv_obj_align(outer_ring, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(outer_ring, 0, 360);
    lv_arc_set_value(outer_ring, 360);
    lv_arc_set_bg_angles(outer_ring, 0, 360);
    lv_obj_set_style_arc_width(outer_ring, 1, LV_PART_MAIN);
    lv_obj_set_style_arc_color(outer_ring, lv_color_hex(theme->tick_minor_color), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(outer_ring, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_arc_width(outer_ring, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(outer_ring, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(outer_ring, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, FACE_R * 2 + 6, FACE_R * 2 + 6);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_range(arc, 0, 360);
    lv_arc_set_value(arc, 360);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    for (int h = 0; h < 12; h++) {
        int ang = h * 30;
        int big = (h % 3 == 0);
        create_marker(FACE_R - 7, ang, big ? 6 : 4,
                      lv_color_hex(big ? theme->tick_major_color : theme->ring_color));
    }

    for (int m = 0; m < 60; m++) {
        if (m % 5 == 0) {
            continue;
        }
        create_marker(FACE_R - 5, m * 6, 1, lv_color_hex(theme->tick_minor_color));
    }

    for (int h = 1; h <= 12; h++) {
        char hour_text[3];
        int ang = (h % 12) * 30;

        snprintf(hour_text, sizeof(hour_text), "%d", h);
        create_text_marker(FACE_R - 18, ang, hour_text, lv_color_hex(theme->tick_major_color));
    }

    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, s_status_text);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 6);
    lv_obj_set_style_text_color(status_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, LV_PART_MAIN);

    lv_obj_t *brand_label = lv_label_create(scr);
    lv_label_set_text(brand_label, "AUTOMATIC");
    lv_obj_align(brand_label, LV_ALIGN_CENTER, 0, -42);
    lv_obj_set_style_text_color(brand_label, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(brand_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(brand_label, 2, LV_PART_MAIN);

    lv_obj_t *sub_label = lv_label_create(scr);
    lv_label_set_text(sub_label, "MECHANICAL");
    lv_obj_align(sub_label, LV_ALIGN_CENTER, 0, 48);
    lv_obj_set_style_text_color(sub_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(sub_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(sub_label, 1, LV_PART_MAIN);

    hour_hand = lv_obj_create(scr);
    min_hand  = lv_obj_create(scr);
    sec_hand  = lv_obj_create(scr);
    lv_obj_remove_style_all(hour_hand);
    lv_obj_remove_style_all(min_hand);
    lv_obj_remove_style_all(sec_hand);
    lv_obj_remove_flag(hour_hand, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(min_hand, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(sec_hand, LV_OBJ_FLAG_CLICKABLE);
    init_hand(hour_hand, 34, lv_color_hex(theme->hour_color), 6);
    init_hand(min_hand, 50, lv_color_hex(theme->minute_color), 4);
    init_hand(sec_hand, 56, lv_color_hex(WATCH_SECOND_HAND_COLOR), 2);

    lv_obj_t *dot = lv_obj_create(scr);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 8, 8);
    lv_obj_align(dot, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dot, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(dot, lv_color_hex(theme->tick_major_color), LV_PART_MAIN);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *time_panel = lv_obj_create(scr);
    lv_obj_remove_style_all(time_panel);
    lv_obj_set_size(time_panel, 112, 36);
    lv_obj_align(time_panel, LV_ALIGN_CENTER, 0, 18);
    lv_obj_set_style_bg_color(time_panel, lv_color_hex(theme->panel_bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(time_panel, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_width(time_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(time_panel, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_radius(time_panel, 14, LV_PART_MAIN);
    lv_obj_remove_flag(time_panel, LV_OBJ_FLAG_CLICKABLE);

    time_label = lv_label_create(time_panel);
    lv_label_set_text(time_label, "--:--");
    lv_obj_center(time_label);
    lv_obj_set_style_text_color(time_label, lv_color_hex(theme->time_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_32, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(time_label, 1, LV_PART_MAIN);

    refresh_hands();
    update_time_label();
    update_status_label();
}

static void build_tilt_ball_ui(void)
{
    const watch_theme_t *theme = get_theme();

    lv_obj_clean(scr);
    reset_ui_refs();
    ensure_fluid_canvas_buffer();
    lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_SCREEN_BG_COLOR), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, s_status_text);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_color(status_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, LV_PART_MAIN);

    if (s_fluid_canvas_buf != NULL) {
        fluid_canvas = lv_canvas_create(scr);
        lv_canvas_set_buffer(fluid_canvas, s_fluid_canvas_buf, FLUID_VIEW_W, FLUID_VIEW_H, LV_COLOR_FORMAT_RGB565);
        lv_obj_set_size(fluid_canvas, FLUID_VIEW_W, FLUID_VIEW_H);
        lv_obj_align(fluid_canvas, LV_ALIGN_TOP_MID, 0, 34);
        lv_obj_remove_flag(fluid_canvas, LV_OBJ_FLAG_CLICKABLE);
    } else {
        lv_obj_t *wait_label = lv_label_create(scr);
        lv_label_set_text(wait_label, u8"模拟画布初始化失败");
        lv_obj_align(wait_label, LV_ALIGN_CENTER, 0, -8);
        lv_obj_set_style_text_color(wait_label, lv_color_hex(theme->time_color), LV_PART_MAIN);
        lv_obj_set_style_text_font(wait_label, get_cjk_font(), LV_PART_MAIN);
    }

    lv_obj_t *info_panel = lv_obj_create(scr);
    lv_obj_remove_style_all(info_panel);
    lv_obj_set_size(info_panel, 186, 28);
    lv_obj_align(info_panel, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_color(info_panel, lv_color_hex(theme->panel_bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(info_panel, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_width(info_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(info_panel, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_radius(info_panel, 14, LV_PART_MAIN);
    lv_obj_remove_flag(info_panel, LV_OBJ_FLAG_CLICKABLE);

    fluid_info_label = lv_label_create(info_panel);
    update_fluid_info_label();
    lv_obj_center(fluid_info_label);
    lv_obj_set_style_text_color(fluid_info_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(fluid_info_label, &lv_font_montserrat_14, LV_PART_MAIN);

    if (!s_tilt_ball_initialized) {
        tilt_ball_reset_particles();
    }

    render_tilt_ball_canvas();
    update_status_label();
}

static void build_fluid_ui(void)
{
    const watch_theme_t *theme = get_theme();

    lv_obj_clean(scr);
    reset_ui_refs();
    ensure_fluid_canvas_buffer();
    lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_SCREEN_BG_COLOR), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, s_status_text);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_color(status_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, LV_PART_MAIN);

    if (s_fluid_canvas_buf != NULL) {
        fluid_canvas = lv_canvas_create(scr);
        lv_canvas_set_buffer(fluid_canvas, s_fluid_canvas_buf, FLUID_VIEW_W, FLUID_VIEW_H, LV_COLOR_FORMAT_RGB565);
        lv_obj_set_size(fluid_canvas, FLUID_VIEW_W, FLUID_VIEW_H);
        lv_obj_align(fluid_canvas, LV_ALIGN_TOP_MID, 0, 34);
        lv_obj_remove_flag(fluid_canvas, LV_OBJ_FLAG_CLICKABLE);
    } else {
        lv_obj_t *wait_label = lv_label_create(scr);
        lv_label_set_text(wait_label, u8"模拟画布初始化失败");
        lv_obj_align(wait_label, LV_ALIGN_CENTER, 0, -8);
        lv_obj_set_style_text_color(wait_label, lv_color_hex(theme->time_color), LV_PART_MAIN);
        lv_obj_set_style_text_font(wait_label, get_cjk_font(), LV_PART_MAIN);
    }

    lv_obj_t *info_panel = lv_obj_create(scr);
    lv_obj_remove_style_all(info_panel);
    lv_obj_set_size(info_panel, 176, 28);
    lv_obj_align(info_panel, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_color(info_panel, lv_color_hex(theme->panel_bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(info_panel, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_width(info_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(info_panel, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_radius(info_panel, 14, LV_PART_MAIN);
    lv_obj_remove_flag(info_panel, LV_OBJ_FLAG_CLICKABLE);

    fluid_info_label = lv_label_create(info_panel);
    update_fluid_info_label();
    lv_obj_center(fluid_info_label);
    lv_obj_set_style_text_color(fluid_info_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(fluid_info_label, get_cjk_font(), LV_PART_MAIN);

    if (!s_fluid_initialized) {
        fluid_reset_particles();
    }

    render_fluid_canvas();
    update_status_label();
}

/* ============================================================
 *  Camera Preview Mode
 * ============================================================ */

/* Camera frame callback - copies frame to canvas buffer */
static void camera_frame_cb(uint8_t *camera_buf, uint8_t camera_buf_index,
                             uint32_t w, uint32_t h, size_t len)
{
    if (s_app_mode != APP_MODE_CAMERA) {
        return;
    }

    /* Copy camera frame to canvas buffer for next LVGL refresh.
     * s_cam_canvas_buf is aligned; copy min(len, canvas_buf_size) bytes */
    if (s_cam_canvas_buf != NULL && camera_buf != NULL) {
        size_t copy_len = (len < s_cam_canvas_buf_size) ? len : s_cam_canvas_buf_size;
        memcpy(s_cam_canvas_buf, camera_buf, copy_len);
        s_new_frame_ready = true;
    }
}

/* Camera refresh timer - updates canvas only when new frame arrived */
static void camera_refresh_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (s_app_mode != APP_MODE_CAMERA || camera_canvas == NULL) {
        return;
    }

    if (s_new_frame_ready) {
        s_new_frame_ready = false;
        lv_obj_invalidate(camera_canvas);
    }
}

/* Initialize camera hardware and canvas buffer */
static esp_err_t camera_init_hw(void)
{
    if (s_camera_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = bsp_camera_start(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bsp_camera_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Allocate PSRAM-aligned canvas backing buffer (240x240 RGB565) */
    size_t cache_line = 64;
    esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &cache_line);
    size_t raw_size = BSP_LCD_H_RES * BSP_LCD_V_RES * 2;
    s_cam_canvas_buf_size = CAM_ALIGN_UP(raw_size, cache_line);

    s_cam_canvas_buf = heap_caps_aligned_calloc(cache_line, 1, s_cam_canvas_buf_size,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_cam_canvas_buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc cam canvas buffer");
        return ESP_ERR_NO_MEM;
    }

    memset(s_cam_canvas_buf, 0, s_cam_canvas_buf_size);
    s_camera_initialized = true;
    ESP_LOGI(TAG, "Camera HW initialized, canvas_buf=%p size=%u", s_cam_canvas_buf,
             (unsigned)s_cam_canvas_buf_size);
    return ESP_OK;
}

/* Start camera video stream */
static esp_err_t camera_start_stream(void)
{
    if (s_camera_streaming) {
        return ESP_OK;
    }

    esp_err_t ret = camera_init_hw();
    if (ret != ESP_OK) {
        return ret;
    }

    s_new_frame_ready = false;

    s_cam_fd = app_video_open(BSP_CAMERA_DEVICE, APP_VIDEO_FMT_RGB565);
    if (s_cam_fd < 0) {
        ESP_LOGE(TAG, "app_video_open failed");
        return ESP_FAIL;
    }

    ret = app_video_set_bufs(s_cam_fd, CAM_NUM_BUFS, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "app_video_set_bufs failed");
        close(s_cam_fd);
        s_cam_fd = -1;
        return ret;
    }

    ret = app_video_register_frame_operation_cb(camera_frame_cb);
    if (ret != ESP_OK) {
        close(s_cam_fd);
        s_cam_fd = -1;
        return ret;
    }

    ret = app_video_stream_task_start(s_cam_fd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "app_video_stream_task_start failed");
        close(s_cam_fd);
        s_cam_fd = -1;
        return ret;
    }

    s_camera_streaming = true;
    ESP_LOGI(TAG, "Camera stream started");
    return ESP_OK;
}

/* Stop camera video stream */
static void camera_stop_stream(void)
{
    if (!s_camera_streaming) {
        return;
    }

    s_camera_streaming = false;
    s_new_frame_ready = false;

    app_video_stream_task_stop(s_cam_fd);
    app_video_wait_video_stop();
    app_video_close(s_cam_fd);
    s_cam_fd = -1;
    ESP_LOGI(TAG, "Camera stream stopped");
}

/* Build camera preview UI */
static void build_camera_ui(void)
{
    const watch_theme_t *theme = get_theme();

    lv_obj_clean(scr);
    reset_ui_refs();
    release_fluid_canvas_buffer();

    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* Status label at top */
    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, s_status_text);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_text_color(status_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, LV_PART_MAIN);

    /* Camera canvas - set buffer once (never call set_buffer again in callback) */
    if (s_cam_canvas_buf != NULL) {
        camera_canvas = lv_canvas_create(scr);
        lv_canvas_set_buffer(camera_canvas, s_cam_canvas_buf,
                             BSP_LCD_H_RES, BSP_LCD_V_RES, LV_COLOR_FORMAT_RGB565);
        lv_obj_center(camera_canvas);
    } else {
        /* Fallback: placeholder while buffer not ready */
        lv_obj_t *wait_label = lv_label_create(scr);
        lv_label_set_text(wait_label, u8"摄像头初始化中...");
        lv_obj_center(wait_label);
        lv_obj_set_style_text_color(wait_label, lv_color_hex(0x808080), LV_PART_MAIN);
        lv_obj_set_style_text_font(wait_label, get_cjk_font(), LV_PART_MAIN);
    }

    /* Hint label at bottom */
    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, u8"MENU: \u5207\u6362\u6a21\u5f0f");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_obj_set_style_text_color(hint, lv_color_hex(theme->time_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, LV_PART_MAIN);

    /* Start camera streaming (runs on core 1, doesn't block LVGL) */
    if (camera_start_stream() != ESP_OK) {
        set_status_text(u8"\u6444\u50cf\u5934\u542f\u52a8\u5931\u8d25");
    }
}

static bool parse_date_command(const char *line, struct tm *timeinfo)
{
    int month;
    int day;
    int hour;
    int minute;
    int year;
    char command[8] = {0};

    if (sscanf(line, "%7s %2d%2d%2d%2d%4d", command, &month, &day, &hour, &minute, &year) != 6) {
        return false;
    }

    if (strcmp(command, "date") != 0) {
        return false;
    }

    if (month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || year < 2000) {
        return false;
    }

    *timeinfo = (struct tm) {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = 0,
        .tm_isdst = -1,
    };

    return true;
}

static bool set_system_time_from_tm(const struct tm *timeinfo)
{
    struct tm time_copy = *timeinfo;
    time_t epoch = mktime(&time_copy);

    if (epoch == (time_t)-1) {
        return false;
    }

    struct timeval tv = {
        .tv_sec = epoch,
        .tv_usec = 0,
    };

    return settimeofday(&tv, NULL) == 0;
}

static void button_event_cb(void *button_handle, void *usr_data)
{
    watch_button_t *btn = (watch_button_t *)usr_data;
    button_event_t event = iot_button_get_event(button_handle);

    if (btn == NULL) {
        return;
    }

    ESP_LOGI(TAG, "%s %s", btn->name, iot_button_get_event_str(event));

    bsp_display_lock(0);

    switch (event) {
    case BUTTON_SINGLE_CLICK:
        if (btn->id == WATCH_BUTTON_MENU) {
            toggle_app_mode();
        } else if (btn->id == WATCH_BUTTON_PLAY) {
            toggle_watch_style();
        } else if (btn->id == WATCH_BUTTON_UP &&
                   (s_app_mode == APP_MODE_TILT_BALL || s_app_mode == APP_MODE_FLUID)) {
            s_gravity_rotation = (s_gravity_rotation + GRAVITY_ROTATE_STEP_DEG) % 360;
            update_fluid_info_label();
        } else if (btn->id == WATCH_BUTTON_DOWN &&
                   (s_app_mode == APP_MODE_TILT_BALL || s_app_mode == APP_MODE_FLUID)) {
            s_gravity_rotation = (s_gravity_rotation + 360 - GRAVITY_ROTATE_STEP_DEG) % 360;
            update_fluid_info_label();
        }
        break;
    case BUTTON_LONG_PRESS_START:
        if (btn->id == WATCH_BUTTON_PLAY && s_app_mode == APP_MODE_TILT_BALL) {
            s_imu_has_valid_sample = false;
            tilt_ball_reset_particles();
            render_tilt_ball_canvas();
            update_fluid_info_label();
        } else if (btn->id == WATCH_BUTTON_PLAY && s_app_mode == APP_MODE_FLUID) {
            s_imu_has_valid_sample = false;
            fluid_reset_particles();
            render_fluid_canvas();
            update_fluid_info_label();
        }
        break;
    default:
        break;
    }

    bsp_display_unlock();
}

static void init_buttons(void)
{
    button_handle_t board_buttons[BSP_BUTTON_NUM] = {NULL};
    int button_count = 0;

    if (s_buttons[0].handle != NULL) {
        return;
    }

    esp_err_t ret = bsp_iot_button_create(board_buttons, &button_count, BSP_BUTTON_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSP buttons: %s", esp_err_to_name(ret));
        return;
    }

    for (size_t i = 0; i < WATCH_BUTTON_COUNT; i++) {
        watch_button_t *btn = &s_buttons[i];

        if (btn->bsp_id >= button_count || board_buttons[btn->bsp_id] == NULL) {
            ESP_LOGW(TAG, "%s button is unavailable on BSP index %d", btn->name, (int)btn->bsp_id);
            btn->handle = NULL;
            continue;
        }

        btn->handle = board_buttons[btn->bsp_id];
        iot_button_register_cb(btn->handle, BUTTON_PRESS_DOWN, NULL, button_event_cb, btn);
        iot_button_register_cb(btn->handle, BUTTON_PRESS_UP, NULL, button_event_cb, btn);
        iot_button_register_cb(btn->handle, BUTTON_SINGLE_CLICK, NULL, button_event_cb, btn);
        iot_button_register_cb(btn->handle, BUTTON_DOUBLE_CLICK, NULL, button_event_cb, btn);
        iot_button_register_cb(btn->handle, BUTTON_LONG_PRESS_START, NULL, button_event_cb, btn);

        ESP_LOGI(TAG, "%s button ready on BSP index %d", btn->name, (int)btn->bsp_id);
    }
}

static void configure_console_input(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);

#if defined(CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG) || defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    usb_serial_jtag_driver_config_t usb_jtag_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_jtag_config));
    usb_serial_jtag_vfs_use_driver();
#else
    if (!uart_is_driver_installed((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM)) {
        ESP_ERROR_CHECK(uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    }
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
#endif
}

static void console_task(void *arg)
{
    char line[32];

    LV_UNUSED(arg);

    configure_console_input();

    ESP_LOGI(TAG, "Serial time command ready: date MMDDHHMMYYYY");

    while (true) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            continue;
        }

        struct tm parsed_time = {0};

        if (!parse_date_command(line, &parsed_time)) {
            ESP_LOGW(TAG, "Unsupported command: %s", line);
            continue;
        }

        if (!set_system_time_from_tm(&parsed_time)) {
            ESP_LOGE(TAG, "Failed to apply time command");
            continue;
        }

        s_time_source = TIME_SOURCE_SERIAL;
        refresh_sync_state_text();

        bsp_display_lock(0);
        sync_from_system_time();
        if (s_app_mode == APP_MODE_WATCH) {
            refresh_hands();
            update_time_label();
            update_date_label();
            update_sync_label();
        }
        bsp_display_unlock();

        ESP_LOGI(TAG, "System time updated to %04d-%02d-%02d %02d:%02d",
                 parsed_time.tm_year + 1900, parsed_time.tm_mon + 1, parsed_time.tm_mday,
                 parsed_time.tm_hour, parsed_time.tm_min);
    }
}

static void tick_cb(lv_timer_t *timer)
{
    time_t now = 0;
    struct tm timeinfo = {0};

    LV_UNUSED(timer);

    time(&now);
    if (has_valid_system_time(now)) {
        localtime_r(&now, &timeinfo);
        g_hour   = timeinfo.tm_hour;
        g_minute = timeinfo.tm_min;
        g_second = timeinfo.tm_sec;
    } else {
        g_second++;
        if (g_second >= 60) { g_second = 0; g_minute++; }
        if (g_minute >= 60) { g_minute = 0; g_hour = (g_hour + 1) % 24; }
    }

    if (s_app_mode == APP_MODE_WATCH) {
        refresh_hands();
        update_time_label();
        update_date_label();
        update_sync_label();
    }
}

static void fluid_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (fluid_canvas == NULL) {
        return;
    }

    if (s_app_mode == APP_MODE_TILT_BALL) {
        tilt_ball_sim_step();
        render_tilt_ball_canvas();
    } else if (s_app_mode == APP_MODE_FLUID) {
        fluid_sim_step();
        render_fluid_canvas();
    }
}

void watch_face_start(void)
{
    scr = lv_scr_act();

    s_ntp_configured = has_wifi_credentials_configured();
    refresh_sync_state_text();
    sync_from_system_time();
    init_imu();
    tilt_ball_reset_particles();
    fluid_reset_particles();
    set_mode_status();
    build_active_ui();
    init_buttons();

    if (s_tick_timer == NULL) {
        s_tick_timer = lv_timer_create(tick_cb, 1000, NULL);
    }

    if (s_fluid_timer == NULL) {
        s_fluid_timer = lv_timer_create(fluid_timer_cb, FLUID_TIMER_MS, NULL);
    }

    if (s_camera_timer == NULL) {
        s_camera_timer = lv_timer_create(camera_refresh_timer_cb, 33, NULL);
    }

    if (s_console_task == NULL) {
        xTaskCreate(console_task, "watch_console", 4096, NULL, 4, &s_console_task);
    }

    if (s_network_task == NULL) {
        xTaskCreate(network_time_task, "watch_ntp", 8192, NULL, 4, &s_network_task);
    }
}
