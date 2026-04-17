
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <esp_system.h>
#include "esp_camera.h"
#include "mbedtls/base64.h"
#include <math.h>

struct AppConfig {
  static constexpr bool TEST_SERVER_MODE = true; // true = mock telemetry + mock frame

  // Timings
  static constexpr uint32_t LOOP_DELAY_MS = 200;
  static constexpr uint32_t CAMERA_CAPTURE_PERIOD_MS = 1000;
  static constexpr uint32_t SEND_PERIOD_MS = 300;

  // Mock camera frame path in LittleFS
  static constexpr const char *MOCK_FRAME_PATH = "/mock-frame.png";
};

struct CameraSnapshot {
  String base64;
  size_t sizeBytes = 0;
  uint16_t width = 0;
  uint16_t height = 0;
};

struct Telemetry {
  uint32_t uptimeMs = 0;
  bool testMode = AppConfig::TEST_SERVER_MODE;
  CameraSnapshot camera;
};

struct RuntimeState {
  uint32_t lastCameraCaptureMs = 0;
  uint32_t lastSendMs = 0;
  bool cameraReady = false;
  bool mockFrameLoaded = false;
};

RuntimeState gState;
Telemetry gTelemetry;

// Camera setup (AI Thinker ESP32-CAM defaults)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

bool encodeBinaryToBase64(const uint8_t *data, size_t dataLen, String &outBase64) {
  size_t outLen = 0;
  size_t outCap = ((dataLen + 2) / 3) * 4 + 1;
  unsigned char *encoded = (unsigned char *)malloc(outCap);
  if (!encoded) return false;

  int res = mbedtls_base64_encode(encoded, outCap, &outLen, data, dataLen);
  if (res != 0) {
    free(encoded);
    return false;
  }

  encoded[outLen] = '\0';
  outBase64 = String((char *)encoded);
  free(encoded);
  return true;
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 20;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.print("Camera init failed. error=0x");
    Serial.println((uint32_t)err, HEX);
    return false;
  }
  Serial.println("Camera initialized.");
  return true;
}

bool captureCameraToTelemetry() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed.");
    return false;
  }

  String b64;
  bool ok = encodeBinaryToBase64(fb->buf, fb->len, b64);
  if (ok) {
    gTelemetry.camera.base64 = b64;
    gTelemetry.camera.sizeBytes = fb->len;
    gTelemetry.camera.width = fb->width;
    gTelemetry.camera.height = fb->height;
  } else {
    Serial.println("Camera base64 encode failed.");
  }
  esp_camera_fb_return(fb);
  return ok;
}

bool loadMockFrameFromLittleFS() {
  Serial.println("checkpoint a");
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed.");
    return false;
  }
  Serial.println("checkpoint b");

  File f = LittleFS.open(AppConfig::MOCK_FRAME_PATH, FILE_READ);
  if (!f || f.isDirectory()) {
    Serial.println("mock-frame.png not found in LittleFS.");
    return false;
  }
  Serial.println("checkpoint c");

  size_t fileSize = f.size();
  uint8_t *raw = (uint8_t *)malloc(fileSize);
  if (!raw) {
    f.close();
    Serial.println("failed 3");
    return false;
  }
  Serial.println("checkpoint d");

  size_t readSize = f.read(raw, fileSize);
  f.close();
  if (readSize != fileSize) {
    free(raw);
    Serial.println("failed 4");
    return false;
  }
  Serial.println("checkpoint e");

  String b64;
  bool ok = encodeBinaryToBase64(raw, fileSize, b64);
  free(raw);
  if (!ok) 
  {
    Serial.println("failed 5");
    return false;
  }

  gTelemetry.camera.base64 = b64;
  gTelemetry.camera.sizeBytes = fileSize;
  gTelemetry.camera.width = 320;
  gTelemetry.camera.height = 240;
  Serial.print("Mock frame loaded, bytes=");
  Serial.println(fileSize);
  return true;
}



String buildTelemetryJson() {
  String payload = "{";
  payload += "\"camera\":{";
  payload += "\"width\":" + String(gTelemetry.camera.width) + ",";
  payload += "\"height\":" + String(gTelemetry.camera.height) + ",";
  payload += "\"size_bytes\":" + String(gTelemetry.camera.sizeBytes) + ",";
  payload += "\"jpeg_base64\":\"" + gTelemetry.camera.base64 + "\"";
  payload += "}}";
  return payload;
}

void sendTelemetryToSerial() {
  Serial.println(buildTelemetryJson());
}

void listFS() {
if (!LittleFS.begin(true)) {
  Serial.println("LittleFS mount failed");
  return;
}

  File root = LittleFS.open("/");
  File file = root.openNextFile();
  if (!file)
  {
    Serial.println("NÃO TEM ARQUIVO");
  }
  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }
}

void initCameraOrMock() {
  if (AppConfig::TEST_SERVER_MODE) {
    delay(5000);
    listFS();
    Serial.println("checkpoint teste");

    gState.mockFrameLoaded = loadMockFrameFromLittleFS();
    if (!gState.mockFrameLoaded) {
      Serial.println("Mock mode: failed to load /mock-frame.png; camera payload will be empty.");
    }
  } else {
    gState.cameraReady = initCamera();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  randomSeed((uint32_t)esp_random());

  LittleFS.begin();

  initCameraOrMock();
  Serial.println("Telemetry transport: Serial JSON (no WiFi build).");
}

void loop() {

  uint32_t now = millis();
  if (!AppConfig::TEST_SERVER_MODE && gState.cameraReady && (now - gState.lastCameraCaptureMs >= AppConfig::CAMERA_CAPTURE_PERIOD_MS)) {
    gState.lastCameraCaptureMs = now;
    captureCameraToTelemetry();
  }

  if (now - gState.lastSendMs >= AppConfig::SEND_PERIOD_MS) {
    gState.lastSendMs = now;
    sendTelemetryToSerial();
  }

  delay(AppConfig::LOOP_DELAY_MS);
}
