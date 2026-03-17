// Minimal SSD1306 OLED demo using ESP-IDF on ESP32.
// --------------------------------------------------
// Hardware:
//   - ESP32
//   - 0.96" 128x64 I2C SSD1306 OLED (IZOKEE module)
//   - I2C wiring:
//       GPIO 21 -> SDA
//       GPIO 22 -> SCL
//       3.3 V   -> VCC
//       GND     -> GND
//
// What this program does:
//   - Initializes I2C and the SSD1306 controller
//   - Runs a FreeRTOS task that animates a bouncing pixel
//   - Runs a second FreeRTOS task that prints to serial
//   - Shows how tasks, delays and hardware drivers fit together

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"

// ---------- I2C configuration ----------

// ESP32 default I2C0 pins in most dev boards.
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000  // OLED is happy at 400 kHz

// ---------- SSD1306 configuration ----------

// Most 0.96" SSD1306 I2C modules use address 0x3C.
#define SSD1306_ADDR 0x3C

// Display resolution of your module.
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// Internal page count: each page is 8 pixels high.
#define SSD1306_PAGES (SSD1306_HEIGHT / 8)

// A tiny frame buffer that mirrors the display RAM: 128 * 64 / 8 bytes.
static uint8_t s_framebuffer[SSD1306_WIDTH * SSD1306_PAGES];

// ---------- Small helper functions ----------

// Log an ESP-IDF error with a short label so debugging is easier.
static void log_if_error(const char *label, esp_err_t err)
{
    if (err != ESP_OK)
    {
        printf("%s failed: err=%d\n", label, err);
    }
}

// Configure the ESP32 I2C peripheral as master.
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    log_if_error("i2c_param_config", i2c_param_config(I2C_MASTER_NUM, &conf));
    log_if_error("i2c_driver_install",
                 i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// Send a single command byte to the SSD1306.
static void ssd1306_write_command(uint8_t cmd)
{
    // For SSD1306 over I2C, the first data byte after the address is a "control" byte.
    // 0x00 means: "the following byte(s) are command bytes".
    uint8_t payload[2] = {0x00, cmd};
    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        SSD1306_ADDR,
        payload,
        sizeof(payload),
        pdMS_TO_TICKS(100));
    log_if_error("ssd1306_write_command", err);
}

// Send a block of data bytes (display RAM) to the SSD1306.
static void ssd1306_write_data(const uint8_t *data, size_t len)
{
    // Control byte 0x40 means: "the following byte(s) are display data".
    uint8_t control = 0x40;

    // Build a single I2C transaction: [control][data...].
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(100));
    log_if_error("ssd1306_write_data", err);

    i2c_cmd_link_delete(cmd);
}

// Initialize the SSD1306 controller with a minimal, sane configuration.
static void ssd1306_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));  // give the display some time after power up

    ssd1306_write_command(0xAE); // DISPLAYOFF
    ssd1306_write_command(0x20); // MEMORYMODE
    ssd1306_write_command(0x00); //   Horizontal addressing mode
    ssd1306_write_command(0xB0); // Set page start address for page addressing mode (not used here)

    ssd1306_write_command(0xC8); // COMSCANDEC (scan from COM[N-1] to COM0)
    ssd1306_write_command(0x00); // SETLOWCOLUMN
    ssd1306_write_command(0x10); // SETHIGHCOLUMN
    ssd1306_write_command(0x40); // SETSTARTLINE at 0

    ssd1306_write_command(0x81); // SETCONTRAST
    ssd1306_write_command(0x7F); //   medium contrast

    ssd1306_write_command(0xA1); // SEGREMAP (column address 127 is mapped to SEG0)
    ssd1306_write_command(0xA6); // NORMALDISPLAY (not inverted)

    ssd1306_write_command(0xA8); // SETMULTIPLEX
    ssd1306_write_command(SSD1306_HEIGHT - 1);

    ssd1306_write_command(0xA4); // DISPLAYALLON_RESUME

    ssd1306_write_command(0xD3); // SETDISPLAYOFFSET
    ssd1306_write_command(0x00); //   no offset

    ssd1306_write_command(0xD5); // SETDISPLAYCLOCKDIV
    ssd1306_write_command(0x80); //   recommended default

    ssd1306_write_command(0xD9); // SETPRECHARGE
    ssd1306_write_command(0xF1);

    ssd1306_write_command(0xDA); // SETCOMPINS
    ssd1306_write_command(0x12);

    ssd1306_write_command(0xDB); // SETVCOMDETECT
    ssd1306_write_command(0x40);

    ssd1306_write_command(0x8D); // CHARGEPUMP
    ssd1306_write_command(0x14); //   enable charge pump

    ssd1306_write_command(0xAF); // DISPLAYON
}

// Clear the local framebuffer (not the display yet).
static void ssd1306_clear_buffer(void)
{
    memset(s_framebuffer, 0x00, sizeof(s_framebuffer));
}

// Push the local framebuffer to the physical display.
static void ssd1306_flush(void)
{
    // Set column and page range so the following data fills the entire screen.
    ssd1306_write_command(0x21); // COLUMNADDR
    ssd1306_write_command(0);    //   column start
    ssd1306_write_command(SSD1306_WIDTH - 1); // column end

    ssd1306_write_command(0x22); // PAGEADDR
    ssd1306_write_command(0);    //   page start
    ssd1306_write_command(SSD1306_PAGES - 1); // page end

    // Send the whole framebuffer in one go.
    ssd1306_write_data(s_framebuffer, sizeof(s_framebuffer));
}

// Set or clear a single pixel in the framebuffer.
static void ssd1306_draw_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT)
    {
        return; // outside the visible area
    }

    int page = y / 8;
    int bit = y % 8;
    uint8_t *byte = &s_framebuffer[page * SSD1306_WIDTH + x];

    if (on)
    {
        *byte |= (1 << bit);
    }
    else
    {
        *byte &= ~(1 << bit);
    }
}

// ---------- FreeRTOS demo tasks ----------

// This task animates a single pixel bouncing around the screen.
// It is an infinite loop that:
//   - updates the framebuffer
//   - flushes it to the OLED
//   - sleeps for a short time
static void oled_animation_task(void *pvParameters)
{
    (void)pvParameters; // unused

    int x = 0;
    int y = 0;
    int vx = 1; // velocity in x
    int vy = 1; // velocity in y

    while (1) // tasks in FreeRTOS are typically written as "for(;;)" or "while(1)"
    {
        // Clear previous frame.
        ssd1306_clear_buffer();

        // Draw the moving pixel.
        ssd1306_draw_pixel(x, y, true);

        // Push the framebuffer to the display.
        ssd1306_flush();

        // Update position for the next frame (simple "bouncing ball" physics).
        x += vx;
        y += vy;

        if (x <= 0 || x >= (SSD1306_WIDTH - 1))
        {
            vx = -vx;
        }
        if (y <= 0 || y >= (SSD1306_HEIGHT - 1))
        {
            vy = -vy;
        }

        // Let other tasks (and the idle task) run.
        // From your point of view: this is the frame rate control.
        vTaskDelay(pdMS_TO_TICKS(30)); // ~33 FPS
    }
}

// This task only writes to the serial port.
// It runs at the same time as the OLED task, but with a slower loop so
// you can clearly see both tasks making progress.
static void serial_demo_task(void *pvParameters)
{
    (void)pvParameters; // unused

    int counter = 0;

    while (1)
    {
        // xTaskGetTickCount() returns the number of ticks since the scheduler started.
        // The tick period is defined by configTICK_RATE_HZ (usually 100 Hz or 1000 Hz).
        TickType_t ticks = xTaskGetTickCount();

        printf("[serial_task] Hello from FreeRTOS! counter=%d, ticks=%lu\n",
               counter,
               (unsigned long)ticks);

        counter++;

        // Sleep for 1 second (1000 ms).
        // While this task is sleeping, other tasks (like the OLED animation)
        // continue to run.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Application entry point used by ESP-IDF.
// This is where we:
//   - initialize hardware
//   - create one or more FreeRTOS tasks
//   - then return (the scheduler keeps running the tasks)
void app_main(void)
{
    printf("Starting SSD1306 OLED demo\n");

    i2c_master_init();
    ssd1306_init();
    ssd1306_clear_buffer();
    ssd1306_flush();

    // Create the animation task with:
    //   - stack size: 4096 bytes
    //   - priority: 1 (low, but fine for a simple demo)
    xTaskCreate(
        oled_animation_task,
        "oled_animation_task",
        4096,
        NULL,
        1,
        NULL);

    // Create a second task that only writes to the serial console.
    // It has the same priority as the OLED task, so the scheduler will
    // time-slice between them.
    xTaskCreate(
        serial_demo_task,
        "serial_demo_task",
        4096,
        NULL,
        1,
        NULL);
}

