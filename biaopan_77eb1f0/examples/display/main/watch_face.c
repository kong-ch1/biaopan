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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "button_gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "iot_button.h"
#include "lvgl.h"
#include "sdkconfig.h"
#include "bmi270.h"
#include "qma6100p.h"

LV_FONT_DECLARE(watch_zh_patch_14);

#define CENTER_X                120
#define CENTER_Y                102
#define FACE_R                  74
#define TAG                     "watch_face"
#define VALID_TIME_UNIX         1704067200
#define STATUS_TEXT_LEN         48
#define BUTTON_LONG_MS          1000
#define BUTTON_SHORT_MS         180

#define FLUID_TIMER_MS          33
#define FLUID_VIEW_W            210
#define FLUID_VIEW_H            156
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
    WATCH_STYLE_WARM,
    WATCH_STYLE_COUNT,
} watch_style_t;

typedef enum {
    APP_MODE_WATCH = 0,
    APP_MODE_FLUID,
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

static int g_hour   = 12;
static int g_minute = 0;
static int g_second = 0;

static lv_obj_t *scr;
static lv_obj_t *hour_hand;
static lv_obj_t *min_hand;
static lv_obj_t *sec_hand;
static lv_obj_t *time_label;
static lv_obj_t *status_label;
static lv_obj_t *fluid_canvas;
static lv_obj_t *fluid_info_label;
static lv_timer_t *s_tick_timer;
static lv_timer_t *s_fluid_timer;
static TaskHandle_t s_console_task;
static watch_style_t s_watch_style = WATCH_STYLE_CLASSIC;
static app_mode_t s_app_mode = APP_MODE_WATCH;
static char s_status_text[STATUS_TEXT_LEN] = u8"表盘-就绪";
static uint8_t *s_fluid_canvas_buf;
static fluid_particle_t s_particles[FLUID_PARTICLE_COUNT];
static bool s_fluid_initialized;
static imu_driver_t s_imu_driver = IMU_DRIVER_NONE;
static qma6100p_handle_t s_qma6100p;
static bmi270_handle_t *s_bmi270;
static i2c_master_dev_handle_t s_icm42670;
static imu_sample_t s_imu_filtered = {0.0f, -(FLUID_DEFAULT_GY / FLUID_GRAVITY_SCALE), 0.0f};
static bool s_imu_has_valid_sample;
static int s_gravity_rotation;

static const watch_theme_t s_themes[WATCH_STYLE_COUNT] = {
    [WATCH_STYLE_CLASSIC] = {
        .bg_color = 0x080e1a,
        .ring_color = 0x1565c0,
        .tick_major_color = 0xffffff,
        .tick_minor_color = 0x1e3a5f,
        .hour_color = 0xffffff,
        .minute_color = 0x4fc3f7,
        .second_color = 0xff5252,
        .time_color = 0x4fc3f7,
        .status_color = 0x81d4fa,
        .panel_bg_color = 0x101a2e,
    },
    [WATCH_STYLE_WARM] = {
        .bg_color = 0x16100a,
        .ring_color = 0xffb74d,
        .tick_major_color = 0xfff3e0,
        .tick_minor_color = 0x5f4426,
        .hour_color = 0xfff3e0,
        .minute_color = 0xffcc80,
        .second_color = 0xff7043,
        .time_color = 0xffcc80,
        .status_color = 0xffe0b2,
        .panel_bg_color = 0x24180d,
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
static void build_fluid_ui(void);

static const watch_theme_t *get_theme(void)
{
    return &s_themes[s_watch_style];
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
    if (fluid_info_label == NULL) {
        return;
    }

    lv_label_set_text_fmt(fluid_info_label, "IMU:%s R:%d°", get_imu_name(), get_total_gravity_rotation());
}

static void reset_ui_refs(void)
{
    hour_hand = NULL;
    min_hand = NULL;
    sec_hand = NULL;
    time_label = NULL;
    status_label = NULL;
    fluid_canvas = NULL;
    fluid_info_label = NULL;
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
    const char *text = (s_app_mode == APP_MODE_FLUID)
        ? (s_imu_driver == IMU_DRIVER_NONE ? u8"流体-NO IMU" : u8"流体-IMU")
        : u8"表盘-就绪";

    snprintf(s_status_text, sizeof(s_status_text), "%s", text);
    update_status_label();
}

static void build_active_ui(void)
{
    if (s_app_mode == APP_MODE_FLUID) {
        build_fluid_ui();
    } else {
        build_watch_face_ui();
    }
}

static void toggle_watch_style(void)
{
    s_watch_style = (watch_style_t)((s_watch_style + 1) % WATCH_STYLE_COUNT);
    build_active_ui();
}

static void toggle_app_mode(void)
{
    s_app_mode = (s_app_mode == APP_MODE_WATCH) ? APP_MODE_FLUID : APP_MODE_WATCH;
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

    size_t buf_size = LV_CANVAS_BUF_SIZE(FLUID_VIEW_W, FLUID_VIEW_H, 16, LV_DRAW_BUF_STRIDE_ALIGN);

    s_fluid_canvas_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fluid_canvas_buf == NULL) {
        s_fluid_canvas_buf = heap_caps_malloc(buf_size, MALLOC_CAP_8BIT);
    }
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

    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", g_hour, g_minute, g_second);
    lv_label_set_text(time_label, buf);
}

static void set_hand(lv_obj_t *hand, int length, float angle_deg, lv_color_t color, int width)
{
    lv_obj_set_size(hand, length, width);
    lv_obj_set_pos(hand, CENTER_X, CENTER_Y - width / 2);
    lv_obj_set_style_bg_color(hand, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(hand, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(hand, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(hand, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_x(hand, 0, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_y(hand, width / 2, LV_PART_MAIN);
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
    lv_obj_update_layout(label);
    lv_obj_set_pos(label, x - lv_obj_get_width(label) / 2, y - lv_obj_get_height(label) / 2);
    lv_obj_remove_flag(label, LV_OBJ_FLAG_CLICKABLE);
}

static void refresh_hands(void)
{
    const watch_theme_t *theme = get_theme();
    float sec = g_second * 6.0f;
    float mn  = g_minute * 6.0f + g_second * 0.1f;
    float hr  = (g_hour % 12) * 30.0f + g_minute * 0.5f;

    if (hour_hand == NULL || min_hand == NULL || sec_hand == NULL) {
        return;
    }

    set_hand(hour_hand, 38, hr,  lv_color_hex(theme->hour_color), 4);
    set_hand(min_hand,  56, mn,  lv_color_hex(theme->minute_color), 3);
    set_hand(sec_hand,  62, sec, lv_color_hex(theme->second_color), 2);
}

static void build_watch_face_ui(void)
{
    const watch_theme_t *theme = get_theme();

    lv_obj_clean(scr);
    reset_ui_refs();
    lv_obj_set_style_bg_color(scr, lv_color_hex(theme->bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, FACE_R * 2, FACE_R * 2);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, -18);
    lv_arc_set_range(arc, 0, 360);
    lv_arc_set_value(arc, 360);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_set_style_arc_width(arc, 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    for (int h = 0; h < 12; h++) {
        int ang = h * 30;
        int big = (h % 3 == 0);
        create_marker(FACE_R - 6, ang, big ? 6 : 4,
                      lv_color_hex(big ? theme->tick_major_color : theme->tick_minor_color));
    }

    for (int m = 0; m < 60; m++) {
        if (m % 5 == 0) {
            continue;
        }
        create_marker(FACE_R - 5, m * 6, 2, lv_color_hex(theme->tick_minor_color));
    }

    create_text_marker(FACE_R - 20, 0, "12", lv_color_hex(theme->tick_major_color));
    create_text_marker(FACE_R - 20, 90, "3", lv_color_hex(theme->tick_major_color));
    create_text_marker(FACE_R - 20, 180, "6", lv_color_hex(theme->tick_major_color));
    create_text_marker(FACE_R - 20, 270, "9", lv_color_hex(theme->tick_major_color));

    hour_hand = lv_obj_create(scr);
    min_hand  = lv_obj_create(scr);
    sec_hand  = lv_obj_create(scr);
    lv_obj_remove_style_all(hour_hand);
    lv_obj_remove_style_all(min_hand);
    lv_obj_remove_style_all(sec_hand);
    lv_obj_remove_flag(hour_hand, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(min_hand, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(sec_hand, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *dot = lv_obj_create(scr);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 6, 6);
    lv_obj_align(dot, LV_ALIGN_CENTER, 0, -18);
    lv_obj_set_style_bg_color(dot, lv_color_hex(theme->minute_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dot, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(dot, 3, LV_PART_MAIN);
    lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE);

    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, s_status_text);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_color(status_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, get_cjk_font(), LV_PART_MAIN);

    lv_obj_t *time_panel = lv_obj_create(scr);
    lv_obj_remove_style_all(time_panel);
    lv_obj_set_size(time_panel, 146, 28);
    lv_obj_align(time_panel, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_set_style_bg_color(time_panel, lv_color_hex(theme->panel_bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(time_panel, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_border_width(time_panel, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(time_panel, lv_color_hex(theme->ring_color), LV_PART_MAIN);
    lv_obj_set_style_radius(time_panel, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_remove_flag(time_panel, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *time_title = lv_label_create(time_panel);
    lv_label_set_text(time_title, u8"时间");
    lv_obj_align(time_title, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_text_color(time_title, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(time_title, get_cjk_font(), LV_PART_MAIN);

    time_label = lv_label_create(time_panel);
    lv_label_set_text(time_label, "--:--:--");
    lv_obj_align(time_label, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_text_color(time_label, lv_color_hex(theme->time_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(time_label, 1, LV_PART_MAIN);

    refresh_hands();
    update_time_label();
    update_status_label();
}

static void build_fluid_ui(void)
{
    const watch_theme_t *theme = get_theme();

    lv_obj_clean(scr);
    reset_ui_refs();
    ensure_fluid_canvas_buffer();
    lv_obj_set_style_bg_color(scr, lv_color_hex(theme->bg_color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    status_label = lv_label_create(scr);
    lv_label_set_text(status_label, s_status_text);
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_color(status_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(status_label, get_cjk_font(), LV_PART_MAIN);

    if (s_fluid_canvas_buf != NULL) {
        fluid_canvas = lv_canvas_create(scr);
        lv_canvas_set_buffer(fluid_canvas, s_fluid_canvas_buf, FLUID_VIEW_W, FLUID_VIEW_H, LV_COLOR_FORMAT_RGB565);
        lv_obj_align(fluid_canvas, LV_ALIGN_TOP_MID, 0, 34);
    }

    lv_obj_t *hint_label = lv_label_create(scr);
    lv_label_set_text(hint_label, "Tilt / MENU mode / UP-DN 10deg");
    lv_obj_align(hint_label, LV_ALIGN_BOTTOM_MID, 0, -28);
    lv_obj_set_style_text_color(hint_label, lv_color_hex(theme->time_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(hint_label, &lv_font_montserrat_14, LV_PART_MAIN);

    fluid_info_label = lv_label_create(scr);
    update_fluid_info_label();
    lv_obj_align(fluid_info_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_color(fluid_info_label, lv_color_hex(theme->status_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(fluid_info_label, &lv_font_montserrat_14, LV_PART_MAIN);

    if (!s_fluid_initialized) {
        fluid_reset_particles();
    }

    render_fluid_canvas();
    update_status_label();
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
    case BUTTON_PRESS_DOWN:
        set_status_text(u8"%s-按下", btn->name);
        break;
    case BUTTON_PRESS_UP:
        set_status_text(u8"%s-松开", btn->name);
        break;
    case BUTTON_SINGLE_CLICK:
        if (btn->id == WATCH_BUTTON_MENU) {
            toggle_app_mode();
        } else {
            set_status_text(u8"%s-单击", btn->name);
            if (btn->id == WATCH_BUTTON_PLAY) {
                toggle_watch_style();
            } else if (btn->id == WATCH_BUTTON_UP && s_app_mode == APP_MODE_FLUID) {
                s_gravity_rotation = (s_gravity_rotation + GRAVITY_ROTATE_STEP_DEG) % 360;
                update_fluid_info_label();
                set_status_text(u8"重力旋转-%d°", get_total_gravity_rotation());
            } else if (btn->id == WATCH_BUTTON_DOWN && s_app_mode == APP_MODE_FLUID) {
                s_gravity_rotation = (s_gravity_rotation + 360 - GRAVITY_ROTATE_STEP_DEG) % 360;
                update_fluid_info_label();
                set_status_text(u8"重力旋转-%d°", get_total_gravity_rotation());
            }
        }
        break;
    case BUTTON_DOUBLE_CLICK:
        set_status_text(u8"%s-双击", btn->name);
        break;
    case BUTTON_LONG_PRESS_START:
        set_status_text(u8"%s-长按", btn->name);
        if (btn->id == WATCH_BUTTON_PLAY && s_app_mode == APP_MODE_FLUID) {
            s_imu_has_valid_sample = false;
            fluid_reset_particles();
            render_fluid_canvas();
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

        bsp_display_lock(0);
        sync_from_system_time();
        if (s_app_mode == APP_MODE_WATCH) {
            refresh_hands();
            update_time_label();
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
    }
}

static void fluid_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    if (s_app_mode != APP_MODE_FLUID || fluid_canvas == NULL) {
        return;
    }

    fluid_sim_step();
    render_fluid_canvas();
}

void watch_face_start(void)
{
    scr = lv_scr_act();

    sync_from_system_time();
    init_imu();
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

    if (s_console_task == NULL) {
        xTaskCreate(console_task, "watch_console", 4096, NULL, 4, &s_console_task);
    }
}
