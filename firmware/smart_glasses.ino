/**
 * @file smart_glasses.ino
 * @brief AI Smart Glasses — ESP32-CAM Main Firmware
 *
 * ============================================================================
 * PROJECT:   AI Smart Glasses / CortexIA
 * BOARD:     AI-Thinker ESP32-CAM (OV2640, 4MB PSRAM)
 * AUTHOR:    CortexIA Team
 * VERSION:   1.0.0
 * ============================================================================
 *
 * OVERVIEW
 * --------
 * This firmware turns the AI-Thinker ESP32-CAM into the vision & audio sensor
 * node for the AI Smart Glasses system. On a button press it:
 *   1. Wakes the OV2640 camera and captures a JPEG frame.
 *   2. POSTs the raw JPEG bytes to the CortexIA backend (FastAPI/Streamlit).
 *   3. The backend forwards the image to Gemini Vision and returns the AI
 *      response as plain text.
 *   4. (Placeholder) Plays the response via the PAM8403 amplifier.
 *
 * HARDWARE REQUIREMENTS
 * ----------------------
 *   - AI-Thinker ESP32-CAM module with OV2640
 *   - FTDI adapter or USB-Serial for flashing (IO0 pulled LOW during flash)
 *   - MAX9814 microphone module  → GPIO 13 (ADC2 — WiFi-off only!)
 *   - PAM8403 amplifier          → GPIO 15 (shutdown control)
 *   - Capture button             → GPIO 2  (active LOW, pull-up enabled)
 *   - Mode button                → GPIO 12 (active LOW, pull-up enabled)
 *
 * ARDUINO IDE SETTINGS
 * --------------------
 *   Board:            AI Thinker ESP32-CAM
 *   Partition Scheme: Huge APP (3MB No OTA / 1MB SPIFFS)  ← important!
 *   Upload Speed:     115200
 *   Flash Frequency:  80 MHz
 *   Flash Mode:       QIO
 *   Core Debug Level: None (or Error in production)
 *
 * REQUIRED LIBRARIES (install via Arduino Library Manager)
 * ---------------------------------------------------------
 *   - ESP32 Arduino Core >= 2.0.17  (by Espressif)
 *   - ArduinoJson >= 7.x            (by Benoit Blanchon)
 *   - Base64 >= 1.3.0               (by Densaugeo) — only for Gemini direct mode
 *
 * BACKEND INTEGRATION
 * --------------------
 * The firmware sends a multipart/form-data POST with field name "image" to
 * the CortexIA FastAPI endpoint. Add a route like this to whatsapp_webhook.py
 * (or a new cortex_vision.py):
 *
 *   @app.post("/analyze_image")
 *   async def analyze_image(image: UploadFile = File(...)):
 *       data = await image.read()
 *       # Upload to Gemini Vision
 *       ...
 *
 * See BACKEND_HOST / BACKEND_ENDPOINT below for configuration.
 * ============================================================================
 *
 * LICENSE: MIT
 */

// =============================================================================
// INCLUDES
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include "camera_config.h"   // Pin maps & get_camera_config()

// =============================================================================
// USER CONFIGURATION — EDIT THESE BEFORE FLASHING
// =============================================================================

// --- WiFi credentials --------------------------------------------------------
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

// --- CortexIA backend --------------------------------------------------------
// Option A (recommended): POST to your CortexIA FastAPI server running locally
// or on a VPS / cloud. The backend handles Gemini API calls server-side.
//
//   Example (local dev):   "http://192.168.1.100:8000"
//   Example (production):  "https://cortex.yourdomain.com"
//
#define BACKEND_HOST     "http://192.168.1.100:8000"
#define BACKEND_ENDPOINT "/analyze_image"

// --- Option B: Direct Gemini REST API (no backend needed) --------------------
// Uncomment #define USE_GEMINI_DIRECT to bypass the backend and POST directly
// to the Gemini multimodal REST API from the ESP32. This embeds the API key
// in firmware — only do this for prototyping, NOT production.
//
// #define USE_GEMINI_DIRECT
#ifdef USE_GEMINI_DIRECT
  #define GEMINI_API_KEY    "YOUR_GEMINI_API_KEY_HERE"
  #define GEMINI_MODEL      "gemini-2.0-flash"
  #define GEMINI_API_HOST   "generativelanguage.googleapis.com"
#endif

// --- Behavior ----------------------------------------------------------------
#define CAPTURE_TIMEOUT_MS    8000   // Max wait for server response (ms)
#define WIFI_CONNECT_TIMEOUT  20     // WiFi connection retries (x 500ms)
#define DEBOUNCE_DELAY_MS     200    // Button debounce window
#define SERIAL_BAUD           115200

// =============================================================================
// GLOBAL STATE
// =============================================================================

/** System operating modes */
enum class GlassesMode : uint8_t {
    IDLE        = 0,  // Waiting for input
    CAPTURING   = 1,  // Camera frame being taken
    SENDING     = 2,  // HTTP POST in progress
    PLAYING     = 3,  // Audio response playback (placeholder)
    ERROR_STATE = 4   // Something went wrong
};

volatile GlassesMode g_mode = GlassesMode::IDLE;

// Button state (ISR-safe)
volatile bool g_capture_requested = false;
volatile bool g_mode_toggle       = false;

// Keep-alive for LED blink
unsigned long g_last_blink_ms = 0;

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

bool     initCamera();
bool     connectWiFi();
bool     captureAndSend();
bool     sendToBackend(const uint8_t* buf, size_t len);
bool     sendToGeminiDirect(const uint8_t* buf, size_t len);
void     setLed(bool on);
void     blinkLed(uint8_t times, uint16_t period_ms);
void     setAmplifier(bool enabled);
String   parseBackendResponse(const String& payload);

// Audio placeholders (see AUDIO SECTION at bottom)
void     audioInit();
void     audioPlayText(const String& text);

// ISR handlers
void IRAM_ATTR onCaptureButton();
void IRAM_ATTR onModeButton();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);  // Let serial stabilise

    Serial.println(F("\n========================================"));
    Serial.println(F("  CortexIA Smart Glasses v1.0.0"));
    Serial.println(F("  AI-Thinker ESP32-CAM | OV2640"));
    Serial.println(F("========================================\n"));

    // --- GPIO setup ----------------------------------------------------------
    pinMode(ONBOARD_LED_PIN,   OUTPUT);
    pinMode(ONBOARD_FLASH_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN,    OUTPUT);
    pinMode(AMP_SHUTDOWN_PIN,  OUTPUT);

    // Buttons: internal pull-up, trigger on falling edge
    pinMode(BTN_CAPTURE_PIN, INPUT_PULLUP);
    pinMode(BTN_MODE_PIN,    INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BTN_CAPTURE_PIN), onCaptureButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_MODE_PIN),    onModeButton,    FALLING);

    // Safe defaults
    setLed(false);
    setAmplifier(false);
    digitalWrite(ONBOARD_FLASH_PIN, LOW);

    // --- Print heap before camera init (baseline) ----------------------------
    Serial.printf("[MEM] Free heap before camera init: %u bytes\n",
                  ESP.getFreeHeap());

    // --- Camera init ---------------------------------------------------------
    if (!initCamera()) {
        Serial.println(F("[FATAL] Camera init failed. Halting."));
        g_mode = GlassesMode::ERROR_STATE;
        while (true) { blinkLed(3, 150); delay(1000); }
    }
    Serial.printf("[MEM] Free heap after camera init:  %u bytes\n",
                  ESP.getFreeHeap());

    // --- WiFi connect --------------------------------------------------------
    if (!connectWiFi()) {
        Serial.println(F("[FATAL] WiFi connection failed. Halting."));
        g_mode = GlassesMode::ERROR_STATE;
        while (true) { blinkLed(5, 100); delay(2000); }
    }

    // --- Audio peripherals ---------------------------------------------------
    audioInit();

    // --- Ready ---------------------------------------------------------------
    Serial.println(F("[INFO] System ready. Press capture button to analyse."));
    blinkLed(2, 300);  // Two quick blinks = ready
    g_mode = GlassesMode::IDLE;
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    // -------------------------------------------------------------------------
    // Handle capture request (set by button ISR)
    // -------------------------------------------------------------------------
    if (g_capture_requested && g_mode == GlassesMode::IDLE) {
        g_capture_requested = false;
        Serial.println(F("\n[EVENT] Capture button pressed."));

        setLed(true);  // LED on while working
        bool ok = captureAndSend();
        setLed(false);

        if (ok) {
            blinkLed(1, 500);  // Single long blink = success
        } else {
            blinkLed(4, 80);   // Rapid blinks = error
        }

        g_mode = GlassesMode::IDLE;
    }

    // -------------------------------------------------------------------------
    // Handle mode toggle (extend with more modes as needed)
    // -------------------------------------------------------------------------
    if (g_mode_toggle) {
        g_mode_toggle = false;
        Serial.println(F("[EVENT] Mode button pressed (not yet implemented)."));
        // TODO: cycle through IDLE → CONTINUOUS → AUDIO_ONLY modes
    }

    // -------------------------------------------------------------------------
    // Idle heartbeat — slow blink every 3 seconds to show system is alive
    // -------------------------------------------------------------------------
    if (g_mode == GlassesMode::IDLE) {
        unsigned long now = millis();
        if (now - g_last_blink_ms > 3000) {
            g_last_blink_ms = now;
            digitalWrite(ONBOARD_LED_PIN, LOW);   // LED on (active LOW)
            delay(50);
            digitalWrite(ONBOARD_LED_PIN, HIGH);  // LED off
        }
    }
}

// =============================================================================
// CAMERA INITIALISATION
// =============================================================================

/**
 * @brief Initialises the OV2640 camera with PSRAM-aware configuration.
 *
 * When 4MB PSRAM is detected:
 *   - Frame buffer is placed in PSRAM (saves ~50 KB of DRAM)
 *   - Frame size can be upgraded to UXGA (1600x1200)
 *   - fb_count set to 2 for double-buffering (reduces frame capture latency)
 *
 * Without PSRAM the camera still works at SVGA resolution using DRAM.
 *
 * @return true if camera driver started successfully, false otherwise.
 */
bool initCamera() {
    Serial.println(F("[CAM] Initialising OV2640..."));

    camera_config_t config = get_camera_config();

    // Check PSRAM availability and upgrade settings accordingly
    if (psramFound()) {
        Serial.println(F("[CAM] PSRAM detected — using PSRAM frame buffer."));
        config.fb_location  = CAMERA_FB_IN_PSRAM;
        config.frame_size   = FRAMESIZE_UXGA;   // 1600x1200 — max quality for Gemini
        config.jpeg_quality = 10;
        config.fb_count     = 2;               // Double-buffer in PSRAM
        Serial.printf("[MEM] PSRAM total: %u bytes, free: %u bytes\n",
                      ESP.getPsramSize(), ESP.getFreePsram());
    } else {
        Serial.println(F("[CAM] No PSRAM — using DRAM frame buffer (SVGA max)."));
        config.frame_size   = FRAMESIZE_SVGA;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] esp_camera_init failed: 0x%x\n", err);
        return false;
    }

    // Apply OV2640 sensor tweaks for better image quality
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_vflip(sensor, 0);       // Flip vertically if mounted upside-down
        sensor->set_hmirror(sensor, 0);     // Horizontal mirror
        sensor->set_brightness(sensor, 1);  // -2 to 2
        sensor->set_contrast(sensor, 1);    // -2 to 2
        sensor->set_saturation(sensor, 0);  // -2 to 2
        sensor->set_sharpness(sensor, 1);   // -2 to 2
        sensor->set_denoise(sensor, 1);     // 0 or 1
        sensor->set_whitebal(sensor, 1);    // Auto white balance ON
        sensor->set_exposure_ctrl(sensor, 1); // Auto exposure ON
        sensor->set_gain_ctrl(sensor, 1);     // Auto gain ON
        Serial.println(F("[CAM] Sensor tuning applied."));
    }

    Serial.println(F("[CAM] OV2640 ready."));
    return true;
}

// =============================================================================
// WIFI
// =============================================================================

/**
 * @brief Connects to the configured WiFi network with a timeout.
 *
 * Uses WiFi.begin() and polls for WL_CONNECTED. On success, prints the
 * assigned IP address. The connection is kept alive by the ESP32 WiFi stack.
 *
 * @return true if connected within WIFI_CONNECT_TIMEOUT attempts.
 */
bool connectWiFi() {
    Serial.printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < WIFI_CONNECT_TIMEOUT) {
        delay(500);
        Serial.print(F("."));
        attempts++;
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[WiFi] FAILED to connect."));
        return false;
    }

    Serial.printf("[WiFi] Connected! IP: %s | RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI());
    return true;
}

// =============================================================================
// CAPTURE & SEND ORCHESTRATION
// =============================================================================

/**
 * @brief Captures one JPEG frame then POSTs it to the backend or Gemini.
 *
 * Procedure:
 *   1. Call esp_camera_fb_get() to grab a frame into a DMA/PSRAM buffer.
 *   2. Log frame size for diagnostics.
 *   3. Route to sendToBackend() or sendToGeminiDirect() depending on config.
 *   4. Always release the frame buffer — do NOT return early without freeing!
 *
 * @return true if capture and HTTP POST both succeeded.
 */
bool captureAndSend() {
    g_mode = GlassesMode::CAPTURING;
    Serial.println(F("[CAM] Capturing frame..."));

    // Discard the first frame — OV2640 often returns a stale/dark first frame
    camera_fb_t* warmup = esp_camera_fb_get();
    if (warmup) {
        esp_camera_fb_return(warmup);
        delay(100);
    }

    // Capture the actual frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println(F("[CAM] Frame capture failed!"));
        g_mode = GlassesMode::ERROR_STATE;
        return false;
    }

    Serial.printf("[CAM] Frame captured: %u bytes (%s)\n",
                  fb->len,
                  fb->format == PIXFORMAT_JPEG ? "JPEG" : "RAW");

    // Sanity check — empty frame means sensor problem
    if (fb->len == 0) {
        Serial.println(F("[CAM] Zero-length frame — sensor error."));
        esp_camera_fb_return(fb);
        g_mode = GlassesMode::ERROR_STATE;
        return false;
    }

    // -------------------------------------------------------------------------
    // Hand off to HTTP layer
    // -------------------------------------------------------------------------
    g_mode = GlassesMode::SENDING;
    bool result = false;

#ifdef USE_GEMINI_DIRECT
    result = sendToGeminiDirect(fb->buf, fb->len);
#else
    result = sendToBackend(fb->buf, fb->len);
#endif

    // ALWAYS return the frame buffer to free memory
    esp_camera_fb_return(fb);

    return result;
}

// =============================================================================
// OPTION A — POST TO CORTEXIA BACKEND
// =============================================================================

/**
 * @brief Sends JPEG bytes to the CortexIA FastAPI backend via multipart POST.
 *
 * The backend endpoint (/analyze_image) is expected to:
 *   - Accept a multipart/form-data body with field "image" (the JPEG bytes)
 *   - Forward the image to Gemini Vision API
 *   - Return a JSON object: {"response": "AI description text"}
 *
 * Memory note: HTTPClient streams the POST body directly from the fb->buf
 * pointer so we do NOT need to copy the JPEG into another heap buffer.
 *
 * @param buf   Pointer to JPEG frame buffer in DRAM or PSRAM
 * @param len   Size of the JPEG buffer in bytes
 * @return true if server replied with HTTP 200
 */
bool sendToBackend(const uint8_t* buf, size_t len) {
    Serial.printf("[HTTP] POSTing %u bytes to %s%s\n",
                  len, BACKEND_HOST, BACKEND_ENDPOINT);

    HTTPClient http;
    String url = String(BACKEND_HOST) + String(BACKEND_ENDPOINT);

    // WiFiClient for plain HTTP; use WiFiClientSecure for HTTPS
    WiFiClient client;
    if (!http.begin(client, url)) {
        Serial.println(F("[HTTP] http.begin() failed."));
        return false;
    }

    // Build multipart/form-data body manually for memory efficiency
    // Boundary string (must not appear in JPEG data — using long random string)
    const String boundary = "CortexIA_ESP32CAM_Boundary_X7f9";
    const String contentType = "multipart/form-data; boundary=" + boundary;

    String partHeader = "--" + boundary + "\r\n"
        "Content-Disposition: form-data; name=\"image\"; filename=\"frame.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n";

    String partFooter = "\r\n--" + boundary + "--\r\n";

    // Calculate total content length to set Content-Length header
    size_t totalLen = partHeader.length() + len + partFooter.length();

    http.addHeader("Content-Type", contentType);
    http.addHeader("Content-Length", String(totalLen));
    http.addHeader("X-Device-ID", WiFi.macAddress());  // Identify the glasses device
    http.setTimeout(CAPTURE_TIMEOUT_MS);

    // Compose the body as a stream using a temporary buffer
    // We allocate a header + footer string and send in three sequential writes
    // by using a custom stream approach.

    // For simplicity with the ArduinoHTTPClient API, we concatenate
    // header + image + footer into one heap block. Only do this if the
    // total fits in PSRAM (available) or DRAM.
    uint8_t* body = nullptr;

    if (psramFound()) {
        body = (uint8_t*)ps_malloc(totalLen);
    } else {
        // Only attempt DRAM allocation if frame is small enough
        if (totalLen < 60000) {
            body = (uint8_t*)malloc(totalLen);
        }
    }

    if (!body) {
        Serial.println(F("[HTTP] Failed to allocate POST body buffer!"));
        http.end();
        return false;
    }

    // Assemble body
    size_t offset = 0;
    memcpy(body + offset, partHeader.c_str(), partHeader.length());
    offset += partHeader.length();
    memcpy(body + offset, buf, len);
    offset += len;
    memcpy(body + offset, partFooter.c_str(), partFooter.length());

    // POST
    int httpCode = http.POST(body, totalLen);
    free(body);

    if (httpCode <= 0) {
        Serial.printf("[HTTP] POST error: %s\n", http.errorToString(httpCode).c_str());
        http.end();
        return false;
    }

    Serial.printf("[HTTP] Response code: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.printf("[HTTP] Response body: %s\n", payload.c_str());

        // Extract text from JSON {"response": "..."} using simple string parse
        // (avoids full ArduinoJson allocation for a single string field)
        String aiText = parseBackendResponse(payload);

        if (aiText.length() > 0) {
            Serial.printf("[AI] Cortex response: %s\n", aiText.c_str());
            // TODO: Pass aiText to TTS / audio output
            audioPlayText(aiText);
        }

        http.end();
        return true;
    }

    Serial.printf("[HTTP] Unexpected status: %d\n", httpCode);
    http.end();
    return false;
}

// =============================================================================
// OPTION B — DIRECT GEMINI REST API (PROTOTYPE ONLY)
// =============================================================================

#ifdef USE_GEMINI_DIRECT
/**
 * @brief Sends JPEG frame directly to Gemini multimodal REST API.
 *
 * Uses the generateContent REST endpoint with an inline image part.
 * The JPEG bytes are Base64-encoded and embedded in the JSON body.
 *
 * WARNING: Embedding API keys in firmware is a security risk. Only use
 * this mode for local prototyping. Use sendToBackend() in production.
 *
 * API endpoint:
 *   POST https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent
 *   Header: x-goog-api-key: YOUR_KEY
 *   Body:   { "contents": [{ "parts": [
 *               { "inlineData": { "mimeType": "image/jpeg", "data": "<base64>" }},
 *               { "text": "Describe what you see in this image." }
 *             ]}]}
 *
 * @param buf   JPEG frame buffer pointer
 * @param len   JPEG frame size in bytes
 * @return true if Gemini responded with HTTP 200
 */
bool sendToGeminiDirect(const uint8_t* buf, size_t len) {
    Serial.println(F("[GEMINI] Encoding image to Base64..."));

    // Base64 encode the JPEG
    // base64 output is ~4/3 the input size; allocate accordingly
    size_t b64Len = ((len + 2) / 3) * 4 + 1;
    char* b64Buf = nullptr;

    if (psramFound()) {
        b64Buf = (char*)ps_malloc(b64Len);
    } else {
        b64Buf = (char*)malloc(b64Len);
    }

    if (!b64Buf) {
        Serial.println(F("[GEMINI] Base64 buffer allocation failed."));
        return false;
    }

    // Manual Base64 encoding (no dependency on external library for IRAM safety)
    static const char b64Chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t b = ((uint32_t)buf[i] << 16)
                   | (i + 1 < len ? (uint32_t)buf[i + 1] << 8 : 0)
                   | (i + 2 < len ? (uint32_t)buf[i + 2]      : 0);
        b64Buf[out++] = b64Chars[(b >> 18) & 0x3F];
        b64Buf[out++] = b64Chars[(b >> 12) & 0x3F];
        b64Buf[out++] = (i + 1 < len) ? b64Chars[(b >> 6) & 0x3F] : '=';
        b64Buf[out++] = (i + 2 < len) ? b64Chars[b & 0x3F]        : '=';
    }
    b64Buf[out] = '\0';

    Serial.printf("[GEMINI] Base64 length: %u bytes\n", out);

    // Build JSON body
    // Note: String concatenation with large b64Buf is memory-heavy.
    // In production, stream-write to WiFiClientSecure directly.
    String prompt = "You are CortexIA, an AI assistant integrated into smart glasses. "
                    "Describe what you see clearly and concisely in the user's language. "
                    "If there's text in the image, read it out. "
                    "If there's a person, describe what they're doing.";

    String jsonBody = "{\"contents\":[{\"parts\":["
        "{\"inlineData\":{\"mimeType\":\"image/jpeg\",\"data\":\"" + String(b64Buf) + "\"}},"
        "{\"text\":\"" + prompt + "\"}"
        "]}],"
        "\"generationConfig\":{\"temperature\":0.4,\"maxOutputTokens\":300}}";

    free(b64Buf);

    // HTTPS POST
    WiFiClientSecure secureClient;
    secureClient.setInsecure();  // Skip cert verification — TODO: add CA root cert

    HTTPClient https;
    String url = "https://" + String(GEMINI_API_HOST)
               + "/v1beta/models/" + String(GEMINI_MODEL)
               + ":generateContent";

    Serial.printf("[GEMINI] POST to %s\n", url.c_str());

    if (!https.begin(secureClient, url)) {
        Serial.println(F("[GEMINI] https.begin() failed."));
        return false;
    }

    https.addHeader("Content-Type", "application/json");
    https.addHeader("x-goog-api-key", GEMINI_API_KEY);
    https.setTimeout(CAPTURE_TIMEOUT_MS);

    int httpCode = https.POST(jsonBody);

    if (httpCode <= 0) {
        Serial.printf("[GEMINI] Error: %s\n", https.errorToString(httpCode).c_str());
        https.end();
        return false;
    }

    Serial.printf("[GEMINI] HTTP status: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) {
        String response = https.getString();
        // Extract text from Gemini response JSON
        // candidates[0].content.parts[0].text
        int textStart = response.indexOf("\"text\": \"") + 9;
        int textEnd   = response.indexOf("\"", textStart);
        if (textStart > 9 && textEnd > textStart) {
            String aiText = response.substring(textStart, textEnd);
            Serial.printf("[AI] Gemini: %s\n", aiText.c_str());
            audioPlayText(aiText);
        }
        https.end();
        return true;
    }

    Serial.printf("[GEMINI] Error response: %s\n", https.getString().c_str());
    https.end();
    return false;
}
#endif  // USE_GEMINI_DIRECT

// =============================================================================
// RESPONSE PARSER
// =============================================================================

/**
 * @brief Lightweight JSON parser to extract the "response" field.
 *
 * Expects backend to return: {"response": "AI description..."}
 * Falls back gracefully if the field is not found.
 *
 * For production use, replace with ArduinoJson:
 *   JsonDocument doc;
 *   deserializeJson(doc, payload);
 *   return doc["response"].as<String>();
 *
 * @param payload Raw JSON string from backend
 * @return Extracted response text, or empty String on failure
 */
String parseBackendResponse(const String& payload) {
    // Look for "response": "..."
    int keyIdx = payload.indexOf("\"response\"");
    if (keyIdx < 0) return "";

    int quoteStart = payload.indexOf('"', keyIdx + 10);  // skip : and whitespace
    quoteStart = payload.indexOf('"', quoteStart + 1);   // opening quote of value
    if (quoteStart < 0) return "";

    int quoteEnd = payload.indexOf('"', quoteStart + 1);
    if (quoteEnd < 0) return "";

    return payload.substring(quoteStart + 1, quoteEnd);
}

// =============================================================================
// LED HELPERS
// =============================================================================

/**
 * @brief Sets the onboard red status LED (active LOW on AI-Thinker).
 * @param on true = LED illuminated
 */
void setLed(bool on) {
    // Onboard LED is active LOW (LOW = on, HIGH = off)
    digitalWrite(ONBOARD_LED_PIN, on ? LOW : HIGH);
    // Optional external status LED (active HIGH)
    if (STATUS_LED_PIN >= 0) {
        digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
    }
}

/**
 * @brief Blinks the LED a specified number of times.
 * @param times      Number of blink cycles
 * @param period_ms  Duration of each ON phase in milliseconds
 */
void blinkLed(uint8_t times, uint16_t period_ms) {
    for (uint8_t i = 0; i < times; i++) {
        setLed(true);
        delay(period_ms);
        setLed(false);
        delay(period_ms);
    }
}

// =============================================================================
// AUDIO SUBSYSTEM (MAX9814 MIC + PAM8403 AMPLIFIER)
// =============================================================================

/**
 * @brief Enables or disables the PAM8403 audio amplifier.
 *
 * The PAM8403's /SD (shutdown) pin is active LOW for shutdown.
 * Connect GPIO 15 → PAM8403 /SD via a 10k resistor.
 *   HIGH = amplifier active
 *   LOW  = amplifier in low-power shutdown
 *
 * @param enabled true to activate amplifier
 */
void setAmplifier(bool enabled) {
    digitalWrite(AMP_SHUTDOWN_PIN, enabled ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// MAX9814 MICROPHONE (ADC INPUT)
// ---------------------------------------------------------------------------
/**
 * HARDWARE NOTE — MAX9814 on AI-Thinker ESP32-CAM:
 * -------------------------------------------------
 * The MAX9814 is an analog microphone amplifier (40/50/60 dB gain selectable).
 * Its OUT pin connects to ESP32 GPIO 13 (ADC2_CH4).
 *
 * CRITICAL LIMITATION: ESP32's ADC2 is SHARED WITH the WiFi radio.
 * When WiFi is active (which is always the case in this firmware), ADC2
 * reads ALWAYS FAIL or return garbage values.
 *
 * WORKAROUNDS:
 *   A) Use WiFi.disconnect() briefly, sample audio, then reconnect.
 *      Acceptable for one-shot voice commands, not for streaming.
 *
 *   B) Replace MAX9814 + ADC with an INMP441 I2S MEMS microphone.
 *      I2S uses dedicated DMA and is fully WiFi-compatible.
 *      Recommended I2S pins on AI-Thinker when no SD card:
 *        I2S_WS  = GPIO 15
 *        I2S_SD  = GPIO 13 (data in)
 *        I2S_SCK = GPIO 14
 *
 *   C) Use a separate MCU (e.g., STM32F030) for audio sampling,
 *      which sends processed audio over UART to the ESP32.
 *
 * The placeholder below demonstrates option A.
 */

/** Number of ADC samples for a voice command window */
#define AUDIO_SAMPLE_COUNT  8192  // ~170 ms at 48kHz (adjust to available RAM)
#define AUDIO_SAMPLE_DELAY_US 20  // ~50 kHz sample rate

/**
 * @brief Initialises audio peripherals (placeholder).
 *
 * Future implementation should configure I2S peripheral for INMP441
 * or set ADC attenuation for MAX9814 analog input.
 */
void audioInit() {
    Serial.println(F("[AUDIO] Audio subsystem placeholder initialised."));
    setAmplifier(false);  // Amplifier off until needed

    // TODO (Option B — I2S INMP441):
    //   i2s_config_t i2sCfg = { ... };
    //   i2s_driver_install(I2S_NUM_0, &i2sCfg, 0, NULL);
    //   i2s_set_pin(I2S_NUM_0, &i2sPins);

    // TODO (Option A — MAX9814 ADC):
    //   analogReadResolution(12);       // 12-bit = 0-4095
    //   analogSetAttenuation(ADC_11db); // For 0–3.3V full scale
    //   analogSetPinAttenuation(MIC_ANALOG_PIN, ADC_11db);
}

/**
 * @brief Placeholder: plays or prints an AI response text.
 *
 * Production implementations should:
 *   1. Send aiText to a TTS service (Google TTS / local piper-tts).
 *   2. Receive MP3/WAV audio stream.
 *   3. Decode and output via I2S DAC → PAM8403 → speaker.
 *
 * For a quick prototype without TTS, use the ESP32's built-in DAC
 * on GPIO 25/26 (note: GPIO 25 = VSYNC — conflicts with camera!).
 * Use GPIO 26 (DAC2) with the camera running.
 *
 * @param text AI-generated response text to vocalise
 */
void audioPlayText(const String& text) {
    Serial.printf("[AUDIO] [PLACEHOLDER] Would speak: \"%s\"\n", text.c_str());

    // --- Indicate audio output via LED pattern ---
    setAmplifier(true);
    blinkLed(1, 100);  // Single short blink = "speaking"

    // TODO: Implement TTS pipeline:
    //
    // Step 1: POST text to Google Text-to-Speech REST API:
    //   POST https://texttospeech.googleapis.com/v1/text:synthesize
    //   Body: { "input": {"text": text}, "voice": {"languageCode": "en-US"},
    //           "audioConfig": {"audioEncoding": "LINEAR16"} }
    //
    // Step 2: Decode the returned base64 LINEAR16 PCM into an int16 array.
    //
    // Step 3: Stream to I2S:
    //   size_t bytesWritten;
    //   i2s_write(I2S_NUM_0, pcmBuffer, pcmLen, &bytesWritten, portMAX_DELAY);
    //
    // For Moroccan Darija / multilingual, set languageCode = "ar-MA".

    delay(500);  // Simulate playback duration
    setAmplifier(false);
}

// =============================================================================
// AUDIO CAPTURE PLACEHOLDER (MAX9814 — WiFi Off Window)
// =============================================================================

/**
 * @brief Captures audio samples from MAX9814 during a WiFi-off window.
 *
 * IMPORTANT: This function temporarily disconnects WiFi to free ADC2.
 * Call only when network is not needed immediately.
 *
 * @param sampleBuffer  Pre-allocated int16 buffer for audio samples
 * @param numSamples    Number of samples to capture
 * @return true if samples captured successfully
 */
bool captureAudioSamples(int16_t* sampleBuffer, size_t numSamples) {
    // --- Disconnect WiFi to unlock ADC2 ---
    Serial.println(F("[AUDIO] Disconnecting WiFi for ADC2 access..."));
    WiFi.disconnect(true);
    delay(100);

    // --- Sample microphone ---
    Serial.printf("[AUDIO] Sampling %u ADC2 values from MAX9814...\n", numSamples);
    for (size_t i = 0; i < numSamples; i++) {
        int raw = analogRead(MIC_ANALOG_PIN);  // 0–4095 at 12-bit
        // Convert to signed int16 (centre at zero)
        sampleBuffer[i] = (int16_t)(raw - 2048) * 16;
        delayMicroseconds(AUDIO_SAMPLE_DELAY_US);
    }

    // --- Reconnect WiFi ---
    Serial.println(F("[AUDIO] Reconnecting WiFi..."));
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint8_t retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        retries++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("[AUDIO] WiFi reconnect failed after audio capture!"));
        return false;
    }

    Serial.println(F("[AUDIO] Audio capture complete, WiFi restored."));
    return true;
}

// =============================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// =============================================================================

/**
 * @brief ISR for the capture button (GPIO 2, falling edge).
 *
 * Sets a volatile flag; actual work is deferred to loop() to avoid
 * running camera/HTTP code inside an ISR context.
 *
 * Software debounce: ignores triggers within DEBOUNCE_DELAY_MS of last trigger.
 */
void IRAM_ATTR onCaptureButton() {
    static volatile unsigned long lastTrigger = 0;
    unsigned long now = millis();
    if (now - lastTrigger > DEBOUNCE_DELAY_MS) {
        g_capture_requested = true;
        lastTrigger = now;
    }
}

/**
 * @brief ISR for the mode toggle button (GPIO 12, falling edge).
 */
void IRAM_ATTR onModeButton() {
    static volatile unsigned long lastTrigger = 0;
    unsigned long now = millis();
    if (now - lastTrigger > DEBOUNCE_DELAY_MS) {
        g_mode_toggle = true;
        lastTrigger = now;
    }
}
