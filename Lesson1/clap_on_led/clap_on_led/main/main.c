
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "esp_log.h"
#include "esp_err.h"

// ==================================================
// CONFIGURATION
// ==================================================

#define TAG "AUDIO_TEST"

// INMP441 - I2S0 RX
#define MIC_BCLK       GPIO_NUM_5
#define MIC_WS         GPIO_NUM_4
#define MIC_DOUT       GPIO_NUM_6

// MAX98357A - I2S1 TX
#define AMP_BCLK       GPIO_NUM_15
#define AMP_LRC        GPIO_NUM_16
#define AMP_DIN        GPIO_NUM_17

// LED
#define LED_GPIO       GPIO_NUM_2

// Audio
#define SAMPLE_RATE    16000
#define BUFFER_SIZE    128

// Record 2 seconds
#define RECORD_SECONDS 2
#define RECORD_SAMPLES (SAMPLE_RATE * RECORD_SECONDS)

// Playback gain
#define PLAYBACK_GAIN  2

// Clap detection
#define MIN_AVG_THRESHOLD   500
#define MIN_PEAK_THRESHOLD  20000
#define MAX_PEAK_THRESHOLD  8300000

#define REQUIRED_SOUND_BLOCKS 3

#define LED_ON_TIME_MS 300
#define COOLDOWN_MS    500
#define LOG_INTERVAL_MS 500

// Playback buffer: 256 stereo frames
#define PLAYBACK_CHUNK_FRAMES 256

// ==================================================
// GLOBAL VARIABLES
// ==================================================

static i2s_chan_handle_t rx_handle = NULL;
static i2s_chan_handle_t tx_handle = NULL;

// 2 seconds of 16-bit mono PCM
static int16_t record_buffer[RECORD_SAMPLES];

// Microphone input buffer
static int32_t mic_samples[BUFFER_SIZE];

// Stereo output buffer
static int16_t tx_buffer[PLAYBACK_CHUNK_FRAMES * 2];

static bool led_is_on = false;
static bool has_triggered = false;

static int consecutive_sound_blocks = 0;

static TickType_t led_off_tick = 0;
static TickType_t last_trigger_tick = 0;
static TickType_t last_log_tick = 0;

// ==================================================
// LED
// ==================================================

static void led_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&config));

    gpio_set_level(LED_GPIO, 0);
}

static void led_turn_on(void)
{
    gpio_set_level(LED_GPIO, 1);

    led_is_on = true;

    led_off_tick =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(LED_ON_TIME_MS);
}

static void led_update(void)
{
    if (led_is_on &&
        (int32_t)(
            xTaskGetTickCount() - led_off_tick
        ) >= 0) {

        gpio_set_level(LED_GPIO, 0);

        led_is_on = false;
    }
}

// ==================================================
// I2S MICROPHONE INITIALIZATION
// ==================================================

static void i2s_mic_init(void)
{
    i2s_chan_config_t chan_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_0,
            I2S_ROLE_MASTER
        );

    ESP_ERROR_CHECK(i2s_new_channel(
        &chan_config,
        NULL,
        &rx_handle
    ));

    i2s_std_config_t std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_BCLK,
            .ws = MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = MIC_DOUT,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    // INMP441 L/R nối GND -> LEFT
    std_config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(
        rx_handle,
        &std_config
    ));

    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    ESP_LOGI(TAG, "INMP441 initialized");
}

// ==================================================
// I2S MAX98357A INITIALIZATION
// ==================================================

static void i2s_amp_init(void)
{
    i2s_chan_config_t chan_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_1,
            I2S_ROLE_MASTER
        );

    ESP_ERROR_CHECK(i2s_new_channel(
        &chan_config,
        &tx_handle,
        NULL
    ));

    i2s_std_config_t std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        // Stereo: duplicate audio to left and right
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AMP_BCLK,
            .ws = AMP_LRC,
            .dout = AMP_DIN,
            .din = I2S_GPIO_UNUSED,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(
        tx_handle,
        &std_config
    ));

    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGI(TAG, "MAX98357A initialized");
}

// ==================================================
// AUDIO ANALYSIS
// ==================================================

static void analyze_audio(
    int32_t *data,
    int count,
    int64_t *avg,
    int64_t *peak
)
{
    int64_t sum = 0;
    int64_t max_value = 0;

    for (int i = 0; i < count; i++) {

        // Convert microphone 24-bit sample
        int32_t value = data[i] >> 8;

        int64_t magnitude = value;

        if (magnitude < 0) {
            magnitude = -magnitude;
        }

        sum += magnitude;

        if (magnitude > max_value) {
            max_value = magnitude;
        }
    }

    *avg = sum / count;
    *peak = max_value;
}

// ==================================================
// CLAP DETECTION
// ==================================================

static bool detect_clap(int64_t avg, int64_t peak)
{
    TickType_t now = xTaskGetTickCount();

    if (avg < MIN_AVG_THRESHOLD ||
        peak < MIN_PEAK_THRESHOLD ||
        peak >= MAX_PEAK_THRESHOLD) {

        consecutive_sound_blocks = 0;

        return false;
    }

    consecutive_sound_blocks++;

    if (consecutive_sound_blocks < REQUIRED_SOUND_BLOCKS) {
        return false;
    }

    consecutive_sound_blocks = 0;

    if (has_triggered &&
        (now - last_trigger_tick) <
        pdMS_TO_TICKS(COOLDOWN_MS)) {

        return false;
    }

    last_trigger_tick = now;
    has_triggered = true;

    return true;
}

// ==================================================
// RECORD AUDIO
// ==================================================

static bool record_audio(void)
{
    ESP_LOGI(TAG, "Recording started: %d seconds",
             RECORD_SECONDS);

    size_t recorded = 0;

    last_log_tick = xTaskGetTickCount();

    while (recorded < RECORD_SAMPLES) {

        size_t bytes_read = 0;

        esp_err_t ret = i2s_channel_read(
            rx_handle,
            mic_samples,
            sizeof(mic_samples),
            &bytes_read,
            100
        );

        led_update();

        if (ret != ESP_OK && bytes_read == 0) {

            if (ret != ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "I2S RX error: %s",
                         esp_err_to_name(ret));
                return false;
            }

            continue;
        }

        int count = bytes_read / sizeof(int32_t);

        if (count <= 0) {
            continue;
        }

        // Prevent buffer overflow
        if (count > RECORD_SAMPLES - recorded) {
            count = RECORD_SAMPLES - recorded;
        }

        int64_t avg = 0;
        int64_t peak = 0;

        analyze_audio(
            mic_samples,
            count,
            &avg,
            &peak
        );

        // Convert 24-bit microphone samples to 16-bit PCM
        for (int i = 0; i < count; i++) {

            record_buffer[recorded++] =
                (int16_t)(mic_samples[i] >> 16);
        }

        TickType_t now = xTaskGetTickCount();

        if ((now - last_log_tick) >=
            pdMS_TO_TICKS(LOG_INTERVAL_MS)) {

            ESP_LOGI(
                TAG,
                "Recording AVG=%lld PEAK=%lld",
                (long long)avg,
                (long long)peak
            );

            last_log_tick = now;
        }

        // Keep clap detection active while recording
        if (detect_clap(avg, peak)) {

            ESP_LOGI(TAG, "CLAP DETECTED!");

            led_turn_on();
        }
    }

    ESP_LOGI(TAG, "Recording completed: %u samples",
             (unsigned)recorded);

    return true;
}

// ==================================================
// PLAYBACK AUDIO
// ==================================================

static int16_t apply_gain(int16_t sample)
{
    int32_t value = (int32_t)sample * PLAYBACK_GAIN;

    if (value > 32767) {
        value = 32767;
    }

    if (value < -32768) {
        value = -32768;
    }

    return (int16_t)value;
}

static bool playback_audio(void)
{
    ESP_LOGI(TAG, "Playback started");

    // Pause microphone RX to avoid feedback
    ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));

    size_t played = 0;

    while (played < RECORD_SAMPLES) {

        size_t frames = RECORD_SAMPLES - played;

        if (frames > PLAYBACK_CHUNK_FRAMES) {
            frames = PLAYBACK_CHUNK_FRAMES;
        }

        // Duplicate mono audio into stereo
        for (size_t i = 0; i < frames; i++) {

            int16_t sample =
                apply_gain(record_buffer[played + i]);

            tx_buffer[i * 2] = sample;
            tx_buffer[i * 2 + 1] = sample;
        }

        size_t bytes_to_write =
            frames * 2 * sizeof(int16_t);

        size_t bytes_written = 0;

        esp_err_t ret = i2s_channel_write(
            tx_handle,
            tx_buffer,
            bytes_to_write,
            &bytes_written,
            1000
        );

        if (ret != ESP_OK || bytes_written == 0) {

            ESP_LOGE(
                TAG,
                "I2S TX error: %s",
                esp_err_to_name(ret)
            );

            i2s_channel_enable(rx_handle);

            return false;
        }

        played += bytes_written /
                  (2 * sizeof(int16_t));

        led_update();
    }

    ESP_LOGI(TAG, "Playback completed");

    // Resume microphone RX
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    return true;
}

// ==================================================
// MAIN AUDIO TASK
// ==================================================

static void audio_task(void *arg)
{
    ESP_LOGI(TAG, "Audio test started");

    while (1) {

        // 1. Record 2 seconds
        bool recorded = record_audio();

        if (!recorded) {
            ESP_LOGW(TAG, "Recording failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 2. Playback recorded audio
        playback_audio();

        // 3. Wait before next recording
        ESP_LOGI(TAG, "Next recording in 1 second");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================================================
// APP MAIN
// ==================================================

void app_main(void)
{
    led_init();

    i2s_mic_init();

    i2s_amp_init();

    ESP_LOGI(TAG, "ESP32-S3 Audio Record & Playback");

    xTaskCreate(
        audio_task,
        "audio_task",
        6144,
        NULL,
        5,
        NULL
    );
}
