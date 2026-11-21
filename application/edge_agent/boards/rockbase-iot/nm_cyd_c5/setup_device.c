#include <stdlib.h>
#include <string.h>

#include "dev_custom.h"
#include "devices/dev_lcd_touch/dev_lcd_touch.h"
#include "driver/gpio.h"
#include "esp_board_manager.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_log.h"
#include "gen_board_device_custom.h"
#include "periph_spi.h"

static const char *TAG = "nm_cyd_c5";

esp_err_t lcd_panel_factory_entry_t(esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel)
{
    esp_lcd_panel_dev_config_t panel_dev_cfg = {0};
    memcpy(&panel_dev_cfg, panel_dev_config, sizeof(esp_lcd_panel_dev_config_t));
    esp_err_t ret = esp_lcd_new_panel_st7789(io, &panel_dev_cfg, ret_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7789 panel: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

/* 2D affine calibration (Q10 fixed-point) fitted from 4-corner measurements on NM-CYD-C5. */
#define NM_CYD_C5_TOUCH_Q_SHIFT  10
#define NM_CYD_C5_TOUCH_AX       1230
#define NM_CYD_C5_TOUCH_BX       83
#define NM_CYD_C5_TOUCH_CX       (-33433)
#define NM_CYD_C5_TOUCH_AY       145
#define NM_CYD_C5_TOUCH_BY       1243
#define NM_CYD_C5_TOUCH_CY       (-40262)

static uint16_t nm_cyd_c5_clamp_u16(int32_t val, uint16_t max_val)
{
    if (val <= 0) {
        return 0;
    }
    if (val >= (int32_t)max_val) {
        return max_val;
    }
    return (uint16_t)val;
}

static void nm_cyd_c5_touch_process_coords(esp_lcd_touch_handle_t tp,
                                           uint16_t *x,
                                           uint16_t *y,
                                           uint16_t *strength,
                                           uint8_t *point_num,
                                           uint8_t max_point_num)
{
    (void)strength;
    (void)max_point_num;
    for (uint8_t i = 0; i < *point_num; i++) {
        int32_t raw_x = (int32_t)x[i];
        int32_t raw_y = (int32_t)y[i];
        int32_t cal_x = (NM_CYD_C5_TOUCH_AX * raw_x + NM_CYD_C5_TOUCH_BX * raw_y + NM_CYD_C5_TOUCH_CX + (1 << (NM_CYD_C5_TOUCH_Q_SHIFT - 1))) >> NM_CYD_C5_TOUCH_Q_SHIFT;
        int32_t cal_y = (NM_CYD_C5_TOUCH_AY * raw_x + NM_CYD_C5_TOUCH_BY * raw_y + NM_CYD_C5_TOUCH_CY + (1 << (NM_CYD_C5_TOUCH_Q_SHIFT - 1))) >> NM_CYD_C5_TOUCH_Q_SHIFT;
        x[i] = nm_cyd_c5_clamp_u16(cal_x, tp->config.x_max);
        y[i] = nm_cyd_c5_clamp_u16(cal_y, tp->config.y_max);
        ESP_LOGD(TAG, "touch calibrated pre_transform=(%u, %u)", x[i], y[i]);
    }
}

static int nm_cyd_c5_lcd_touch_init(void *config, int cfg_size, void **device_handle)
{
    if (!config || !device_handle || cfg_size != sizeof(dev_custom_lcd_touch_config_t)) {
        ESP_LOGE(TAG, "Invalid lcd_touch config");
        return -1;
    }

    const dev_custom_lcd_touch_config_t *cfg = (const dev_custom_lcd_touch_config_t *)config;
    void *periph_handle = NULL;
    if (esp_board_manager_get_periph_handle(cfg->peripheral_name, &periph_handle) != ESP_OK || !periph_handle) {
        ESP_LOGE(TAG, "Failed to get SPI peripheral '%s' for lcd_touch", cfg->peripheral_name);
        return -1;
    }

    const periph_spi_handle_t *spi_handle = (const periph_spi_handle_t *)periph_handle;
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(cfg->cs_gpio_num);
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)spi_handle->spi_port, &io_config, &io_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create XPT2046 panel IO");
        return -1;
    }

    esp_lcd_touch_config_t tp_cfg = {
        /* esp_lcd_touch applies mirror_x/mirror_y BEFORE swap_xy, so x_max/y_max
         * must match the pre-swap panel dimensions (240x320) for 320x240 landscape. */
        .x_max = (uint16_t)cfg->y_max,
        .y_max = (uint16_t)cfg->x_max,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = true,
        },
        .process_coordinates = nm_cyd_c5_touch_process_coords,
    };

    esp_lcd_touch_handle_t touch_handle = NULL;
    if (esp_lcd_touch_new_spi_xpt2046(io_handle, &tp_cfg, &touch_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize XPT2046 touch driver");
        esp_lcd_panel_io_del(io_handle);
        return -1;
    }

    dev_lcd_touch_handles_t *handles = calloc(1, sizeof(dev_lcd_touch_handles_t));
    if (!handles) {
        esp_lcd_touch_del(touch_handle);
        esp_lcd_panel_io_del(io_handle);
        return -1;
    }

    handles->io_handle = io_handle;
    handles->touch_handle = touch_handle;
    *device_handle = handles;
    ESP_LOGI(TAG, "Initialized XPT2046 touch on CS GPIO%d", cfg->cs_gpio_num);
    return 0;
}

static int nm_cyd_c5_lcd_touch_deinit(void *device_handle)
{
    dev_lcd_touch_handles_t *handles = (dev_lcd_touch_handles_t *)device_handle;
    if (!handles) {
        return -1;
    }
    if (handles->touch_handle) {
        esp_lcd_touch_del(handles->touch_handle);
    }
    if (handles->io_handle) {
        esp_lcd_panel_io_del(handles->io_handle);
    }
    free(handles);
    return 0;
}

CUSTOM_DEVICE_IMPLEMENT(lcd_touch, nm_cyd_c5_lcd_touch_init, nm_cyd_c5_lcd_touch_deinit);