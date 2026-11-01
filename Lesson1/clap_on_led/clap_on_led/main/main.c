
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

#define LED_GPIO       GPIO_NUM_2

#define I2S_WS_IO      GPIO_NUM_4
#define I2S_BCLK_IO    GPIO_NUM_5
#define I2S_DIN_IO     GPIO_NUM_6

// Điều chỉnh ngưỡng sau khi kiểm tra log
#define SOUND_THRESHOLD 300

// LED sáng 300ms
#define LED_ON_TIME_MS 300

// Chờ 500ms trước khi cho phép phát hiện tiếp
#define COOLDOWN_MS 500

// Số mẫu liên tiếp vượt ngưỡng để xác nhận tiếng động
#define REQUIRED_HITS 2

void app_main(void)
{
    // Cấu hình LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // Khởi tạo I2S
    i2s_chan_handle_t rx_handle;

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_AUTO,
            I2S_ROLE_MASTER
        );

    ESP_ERROR_CHECK(
        i2s_new_channel(&chan_cfg, NULL, &rx_handle)
    );

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN_IO,
        },
    };

    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(rx_handle, &std_cfg)
    );

    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    int32_t samples[256];
    size_t bytes_read = 0;

    int hit_count = 0;
    int print_counter = 0;

    TickType_t last_trigger = 0;

    printf("=== HE THONG PHAT HIEN VO TAY ===\n");

    while (1)
    {
        if (i2s_channel_read(
                rx_handle,
                samples,
                sizeof(samples),
                &bytes_read,
                100
            ) == ESP_OK)
        {
            int count = bytes_read / sizeof(int32_t);

            if (count > 0)
            {
                int64_t sum = 0;

                for (int i = 0; i < count; i++)
                {
                    int32_t val = samples[i] >> 14;

                    sum += abs(val);
                }

                int sound_level = sum / count;

                print_counter++;

                if (print_counter >= 10)
                {
                    printf("Am thanh: %d | Nguong: %d\n",
                           sound_level,
                           SOUND_THRESHOLD);

                    print_counter = 0;
                }

                TickType_t now = xTaskGetTickCount();

                // Kiểm tra thời gian chống kích hoạt lại
                if ((now - last_trigger) >=
                    pdMS_TO_TICKS(COOLDOWN_MS))
                {
                    if (sound_level > SOUND_THRESHOLD)
                    {
                        hit_count++;
                    }
                    else
                    {
                        hit_count = 0;
                    }

                    // Xác nhận có tiếng động
                    if (hit_count >= REQUIRED_HITS)
                    {
                        printf(">>> VO TAY! LED ON 300ms <<<\n");

                        gpio_set_level(LED_GPIO, 1);

                        vTaskDelay(
                            pdMS_TO_TICKS(LED_ON_TIME_MS)
                        );

                        gpio_set_level(LED_GPIO, 0);

                        printf(">>> LED OFF <<<\n");

                        last_trigger = xTaskGetTickCount();

                        hit_count = 0;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
