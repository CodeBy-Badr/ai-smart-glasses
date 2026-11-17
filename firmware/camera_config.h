/**
 * @file camera_config.h
 * @brief AI-Thinker ESP32-CAM — OV2640 GPIO Pin Mapping & Camera Configuration
 *
 * Board:   AI-Thinker ESP32-CAM (ESP32-S with OV2640)
 * MCU:     Espressif ESP32 (dual-core Xtensa LX6, 240 MHz)
 * Camera:  OV2640 2MP JPEG sensor connected via DVP (Parallel) interface
 * PSRAM:   4 MB (optional but strongly recommended for JPEG capture at SVGA+)
 *
 * ─── DO NOT MODIFY THESE PINS ────────────────────────────────────────────────
 * They are hard-wired on the AI-Thinker PCB. Changing them will break the
 * camera or conflict with the onboard SD card and flash LED.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include "esp_camera.h"

// =============================================================================
// SECTION 1 — GPIO PIN DEFINITIONS (AI-Thinker ESP32-CAM)
// =============================================================================

// --- Camera DVP Data Bus (8-bit parallel) ------------------------------------
#define CAM_PIN_D0   5    // Y2 — LSB of pixel data
#define CAM_PIN_D1   18   // Y3
#define CAM_PIN_D2   19   // Y4
#define CAM_PIN_D3   21   // Y5
#define CAM_PIN_D4   36   // Y6
#define CAM_PIN_D5   39   // Y7
#define CAM_PIN_D6   34   // Y8
#define CAM_PIN_D7   35   // Y9 — MSB of pixel data

// --- Camera Control Signals --------------------------------------------------
#define CAM_PIN_XCLK  0   // Master clock output to camera (20 MHz)
#define CAM_PIN_PCLK  22  // Pixel clock input from camera
#define CAM_PIN_VSYNC 25  // Vertical sync (frame boundary)
#define CAM_PIN_HREF  23  // Horizontal reference (line valid)

// --- Camera I2C (SCCB — Serial Camera Control Bus) ---------------------------
#define CAM_PIN_SIOD  26  // SDA — data line
#define CAM_PIN_SIOC  27  // SCL — clock line

// --- Power & Reset -----------------------------------------------------------
#define CAM_PIN_RESET -1  // Not connected on AI-Thinker; held high internally
#define CAM_PIN_PWDN  32  // Power-down pin; set LOW to power up camera

// --- Onboard Peripherals (do not repurpose) ----------------------------------
#define ONBOARD_LED_PIN   33   // Red status LED (active LOW)
#define ONBOARD_FLASH_PIN  4   // White flash LED / SD card detect (active HIGH)
// GPIO 1  = UART0 TX (Serial monitor)
// GPIO 3  = UART0 RX (Serial monitor)
// GPIO 16 = Reserved (PSRAM CS)

// =============================================================================
// SECTION 2 — USER GPIO (Free pins for buttons / audio)
// =============================================================================

/**
 * These are the only GPIOs safely usable for external peripherals on the
 * AI-Thinker module when NO SD card is inserted.
 *
 *  GPIO 2  — INPUT  : Capture trigger button (active LOW, internal pull-up)
 *  GPIO 12 — INPUT  : Mode select button    (active LOW, internal pull-up)
 *  GPIO 13 — INPUT  : MAX9814 analog output (ADC2 — NOTE: disabled when WiFi active!)
 *  GPIO 15 — OUTPUT : PAM8403 shutdown pin  (HIGH = amplifier active)
 *  GPIO 14 — OUTPUT : General purpose LED or indicator
 *
 * WARNING: ADC2 (GPIO 13) cannot be used while WiFi is running. Use ADC1 channels
 *   (GPIOs 32-39) but those are occupied by the camera data bus on AI-Thinker.
 *   For audio sampling with WiFi, use I2S with an external I2S microphone
 *   (e.g. INMP441) or capture audio in a WiFi-off window.
 */
#define BTN_CAPTURE_PIN   2   // Physical shutter / capture button
#define BTN_MODE_PIN     12   // Mode toggle button
#define MIC_ANALOG_PIN   13   // MAX9814 audio out to ESP32 ADC (WiFi-off only)
#define AMP_SHUTDOWN_PIN 15   // PAM8403 /SD pin (HIGH = enabled)
#define STATUS_LED_PIN   14   // Optional external status LED

// =============================================================================
// SECTION 3 — CAMERA CONFIGURATION STRUCT
// =============================================================================

/**
 * @brief Returns a fully populated camera_config_t for the OV2640.
 *
 * Frame size is set to SVGA (800x600) which delivers good image quality
 * while keeping JPEG buffers within safe bounds. When PSRAM is detected
 * at runtime, fb_location is switched to CAMERA_FB_IN_PSRAM inside
 * initCamera() so we can go up to UXGA (1600x1200) if needed.
 *
 * JPEG quality: 1 (best) to 63 (worst). 10-15 is a good trade-off for
 * transmission over WiFi.
 */
static camera_config_t get_camera_config() {
    camera_config_t config;

    // --- Pin mapping ---
    config.pin_d0       = CAM_PIN_D0;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sscb_sda = CAM_PIN_SIOD;
    config.pin_sscb_scl = CAM_PIN_SIOC;
    config.pin_reset    = CAM_PIN_RESET;
    config.pin_pwdn     = CAM_PIN_PWDN;

    // --- Clock ---
    config.xclk_freq_hz = 20000000;  // 20 MHz — required for OV2640 JPEG mode
    config.ledc_timer   = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    // --- Pixel format & frame size ---
    config.pixel_format = PIXFORMAT_JPEG;   // Hardware JPEG compression
    config.frame_size   = FRAMESIZE_SVGA;   // 800x600 — good WiFi balance
    config.jpeg_quality = 12;               // 10-15 is good; lower = better quality
    config.fb_count     = 1;               // Use 2 when PSRAM present for double-buffering

    // --- Frame buffer location (may be overridden in initCamera) ---
    config.fb_location  = CAMERA_FB_IN_DRAM;  // Fallback; PSRAM preferred

    return config;
}

// =============================================================================
// SECTION 4 — FRAME SIZE PRESETS (switch at runtime with esp_camera_sensor_get)
// =============================================================================

/**
 * Supported OV2640 JPEG frame sizes on AI-Thinker ESP32-CAM:
 *
 *  FRAMESIZE_QQVGA   160x120    ~  3 KB JPEG   — fastest, minimal detail
 *  FRAMESIZE_QVGA    320x240    ~ 10 KB JPEG   — good for quick analysis
 *  FRAMESIZE_CIF     400x296    ~ 15 KB JPEG
 *  FRAMESIZE_VGA     640x480    ~ 30 KB JPEG   — standard
 *  FRAMESIZE_SVGA    800x600    ~ 45 KB JPEG   — DEFAULT
 *  FRAMESIZE_XGA    1024x768    ~ 70 KB JPEG
 *  FRAMESIZE_SXGA   1280x1024  ~100 KB JPEG   — requires PSRAM
 *  FRAMESIZE_UXGA   1600x1200  ~150 KB JPEG   — requires PSRAM, recommended max
 *
 * Use FRAMESIZE_QVGA for rapid burst / motion detection.
 * Use FRAMESIZE_UXGA for high-detail scene analysis sent to Gemini Vision.
 */

#endif  // CAMERA_CONFIG_H
