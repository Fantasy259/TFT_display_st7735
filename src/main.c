#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
//#include "esp_lcd_panel_io.h"
//#include "esp_lcd_panel_vendor.h"
//#include "esp_lcd_panel_ops.h"
//#include "esp_lcd_panel_st7789.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "src/drivers/display/st7735/lv_st7735.h"

#define TFT_MOSI    25 //23
#define TFT_SCLK    26 // 18
#define TFT_CS      33 // 0
#define TFT_DC      32 // 2
#define TFT_RST     16 // 4

#define LEDBackLight  27

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              160

static const char *TAG ="MyMain";

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST
static spi_device_handle_t spi;


static void lv_tick_cb(void *arg) {
    lv_tick_inc(1);  // +1 ms
}

static void start_lvgl_tick(void)
{
    const esp_timer_create_args_t tcfg = {
        .callback = &lv_tick_cb,
        .name     = "lvgl_tick"
    };
    static esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tcfg, &tick_timer));
    // alle 1000 µs (1 ms)
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000));
}

/* Send short command to the LCD. This function shall wait until the transaction finishes. */
int32_t my_lcd_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size)
{
 //   ESP_LOGI("ST7735", "send_CMD cmd=0x%02X, size cmd =%d , param = 0x%02x, size Param %d", cmd ? cmd[0] : 0, (int) cmd_size, param ? param[0] : 0, (int)param_size);
    esp_err_t ret;

    // 1. Command senden (DC = 0)
    gpio_set_level(TFT_DC, 0);
    spi_transaction_t t_cmd = {
        .length = cmd_size * 8,
        .tx_buffer = cmd,
    };
    ret = spi_device_polling_transmit(spi, &t_cmd);
  
    if (ret != ESP_OK) return -1;

    // 2. Parameterdaten senden (falls vorhanden, DC = 1)
    if (param != NULL && param_size > 0) {
        gpio_set_level(TFT_DC, 1);
        spi_transaction_t t_data = {
            .length = param_size * 8,
            .tx_buffer = param,
        };
        ret = spi_device_polling_transmit(spi, &t_data);
        if (ret != ESP_OK) return -1;
    }

    return 0;
}

/* Send large array of pixel data to the LCD. If necessary, this function has to do the byte-swapping. This function can do the transfer in the background. */
int32_t my_lcd_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size)
{

//    ESP_LOGI("ST7735", "send_color: cmd=0x%02X, size=%d", cmd ? cmd[0] : 0, (int)param_size);
         esp_err_t ret;

    // RGB565 in BRG565 Farben tauschen 
    for(size_t i = 0; i < param_size; i += 2)
    {
        uint16_t pixel = (param[i] << 8) | param[i+1]; // RGB565

        uint16_t r = (pixel & 0xF800) >> 11;
        uint16_t g = (pixel & 0x07E0) >> 5;
        uint16_t b = (pixel & 0x001F);

        uint16_t bgr = (b << 11) | (r << 5) | (g);

        param[i]   = (bgr >> 8) & 0xFF;
        param[i+1] = bgr & 0xFF;
    }

    // 1. Command schicken (z. B. 0x2C)
    if (cmd != NULL && cmd_size > 0) {
        gpio_set_level(TFT_DC, 0);
        spi_transaction_t t_cmd = {
            .length = cmd_size * 8,
            .tx_buffer = cmd,
        };
        ret = spi_device_polling_transmit(spi, &t_cmd);
        if (ret != ESP_OK) return -1;
    }

    // 2. Pixel-Daten (param)
    if (param != NULL && param_size > 0) {
        gpio_set_level(TFT_DC, 1);
        spi_transaction_t t_data = {
            .length = param_size * 8,
            .tx_buffer = param,
        };
        ret = spi_device_polling_transmit(spi, &t_data);
        if (ret != ESP_OK) return -1;
    }

    lv_display_flush_ready(disp);
    return 0; // Erfolg
}

// Task für LVGL
static void lvgl_task(void *pvParameter)
{
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{

    // GPIO richtung setzen 
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TFT_CS) | (1ULL << TFT_RST) | (1ULL << TFT_DC) | (1ULL << LEDBackLight),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Reset Sequenz für den Dsiaply 
    gpio_set_level(TFT_RST, 0);       // Reset aktiv
    vTaskDelay(pdMS_TO_TICKS(20));        // mindestens 10µs, wir geben 20ms
    gpio_set_level(TFT_RST, 1);       // Reset aus
    vTaskDelay(pdMS_TO_TICKS(120));       // warten bis Controller bereit

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = TFT_SCLK,
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,             // Speed 20 MHZ
        .mode = 0,
        .spics_io_num = TFT_CS,
        .queue_size = 10,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Backlight einschalten 
    gpio_set_level(LEDBackLight,1);         

    // LVGL einrichten 
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // Zeitgeber für lvgl. Ohne die funktion bleibt LVGL stehen und es wird ein Watchdog ausgelöst vom LVGL 
    start_lvgl_tick();     

    // Erstellen des Displays 
    lv_lcd_flag_t flags = LV_LCD_FLAG_NONE;
    lv_display_t *disp = lv_st7735_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, flags, my_lcd_send_cmd, my_lcd_send_color);
   
    // Anlegen des Datenbuffers (Buffer nur 1/10 der Fläche)
    static uint8_t draw_buf1[128 * 160 / 10 * 2]; // 1/10 Fläche * 2 bytes per Pixel (RGB565)
    lv_display_set_buffers(disp, draw_buf1, NULL, sizeof(draw_buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

   lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
    lv_st7735_set_gap(disp, 0, 0); // falls nötig anpassen

    // Beispile inhalt erstellen 
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), LV_PART_MAIN);
    //Create a white label, set its text and align it to the center
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world 1");
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // Start LVGL working Task 
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 4096, NULL, 1, NULL, 1);
}