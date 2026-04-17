 #include <Wire.h>
 #include <FS.h>
 #include <LittleFS.h>
 #include <esp_system.h>
 #include "esp_camera.h"
 #include "mbedtls/base64.h"
 #include <math.h>

 struct AppConfig {
   static constexpr bool TEST_SERVER_MODE = true; // true = mock telemetry + mock frame

   // Pinout (adjust to your board and wiring)
   static constexpr uint8_t MOTOR_SELECT_PINS[4] = {14, 27, 26, 25};
   static constexpr uint8_t ESC_SIGNAL_PIN = 13;
   static constexpr uint8_t RESIST_SENSE_PIN = 35;
   static constexpr uint8_t LIDAR_ADDRS[4] = {0x62, 0x63, 0x64, 0x65};

   // ESC test signal for integrity checks only (engineering proof mode)
   static constexpr uint16_t ESC_PWM_FREQ_HZ = 50;
   static constexpr uint8_t ESC_PWM_RES_BITS = 16;
   static constexpr uint16_t ESC_TEST_PULSE_US = 1000;
   static constexpr uint16_t ESC_STOP_PULSE_US = 0;

//   // Integrity test thresholds
   static constexpr int INTEGRITY_THRESHOLD = 25;
   static constexpr uint8_t INTEGRITY_SAMPLES = 10;
   static constexpr uint8_t INTEGRITY_SAMPLE_DELAY_MS = 2;

//   // Timings
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
   uint16_t lidarCm[4] = {0, 0, 0, 0};
   bool motorIntegrity[4] = {false, false, false, false};
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

 uint32_t pulseUsToDuty(uint16_t pulseUs) {
   const uint32_t periodUs = 20000;
   const uint32_t maxDuty = (1UL << AppConfig::ESC_PWM_RES_BITS) - 1UL;
   return ((uint32_t)pulseUs * maxDuty) / periodUs;
 }

 void escWriteDuty(uint32_t duty) {
   ledcWrite(AppConfig::ESC_SIGNAL_PIN, duty);
 }

 void setEscPulseUs(uint16_t pulseUs) {
   escWriteDuty(pulseUsToDuty(pulseUs));
 }

 void setMotorSelection(int motorIndex) {
   for (int i = 0; i < 4; i++) {
     digitalWrite(AppConfig::MOTOR_SELECT_PINS[i], (i == motorIndex) ? HIGH : LOW);
   }
 }

 int readAnalogAverage(uint8_t pin, int samples, int sampleDelayMs) {
   long acc = 0;
   for (int i = 0; i < samples; i++) {
     acc += analogRead(pin);
     delay(sampleDelayMs);
   }
   return (int)(acc / samples);
 }

 uint16_t readDistanceFromAddr(uint8_t lidarAddr) {
   Wire.beginTransmission(lidarAddr);
   Wire.write(0x00);
   Wire.write(0x04);
   if (Wire.endTransmission() != 0) return 0;
   delay(20);

   Wire.beginTransmission(lidarAddr);
   Wire.write(0x8f);
   if (Wire.endTransmission() != 0) return 0;

   Wire.requestFrom((int)lidarAddr, 2);
   if (Wire.available() < 2) return 0;
   return (uint16_t)((Wire.read() << 8) | Wire.read());
 }

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
   if (!LittleFS.begin(true)) {
     Serial.println("LittleFS mount failed.");
     return false;
   }

   File f = LittleFS.open(AppConfig::MOCK_FRAME_PATH, FILE_READ);
   if (!f || f.isDirectory()) {
     Serial.println("mock-frame.png not found in LittleFS.");
     delay(2232);
     return false;
   }

   size_t fileSize = f.size();
   uint8_t *raw = (uint8_t *)malloc(fileSize);
   if (!raw) {
     f.close();
     return false;
   }

   size_t readSize = f.read(raw, fileSize);
   f.close();
   if (readSize != fileSize) {
     free(raw);
     return false;
   }

   String b64;
   bool ok = encodeBinaryToBase64(raw, fileSize, b64);
   free(raw);
   if (!ok) return false;

   gTelemetry.camera.base64 = b64;
   gTelemetry.camera.sizeBytes = fileSize;
   gTelemetry.camera.width = 320;
   gTelemetry.camera.height = 240;
   Serial.print("Mock frame loaded, bytes=");
   Serial.println(fileSize);
   return true;
 }

 void generateMockLidar(uint16_t outLidar[4]) {
   float t = millis() / 1000.0f;
   outLidar[0] = (uint16_t)constrain((int)(150 + 35 * sinf(t * 1.2f) + random(-3, 4)), 20, 400);
   outLidar[1] = (uint16_t)constrain((int)(210 + 50 * cosf(t * 0.8f) + random(-4, 5)), 20, 400);
   outLidar[2] = (uint16_t)constrain((int)(95 + 20 * sinf(t * 1.8f + 1.3f) + random(-2, 3)), 20, 400);
   outLidar[3] = (uint16_t)constrain((int)(270 + 60 * sinf(t * 0.55f + 2.2f) + random(-5, 6)), 20, 400);
 }

 String buildTelemetryJson() {
   String payload = "{";
   payload += "\"test_mode\":" + String(gTelemetry.testMode ? "true" : "false") + ",";
   payload += "\"uptime_ms\":" + String(gTelemetry.uptimeMs) + ",";
   payload += "\"lidar_cm\":[" + String(gTelemetry.lidarCm[0]) + "," + String(gTelemetry.lidarCm[1]) + "," + String(gTelemetry.lidarCm[2]) + "," + String(gTelemetry.lidarCm[3]) + "],";
   payload += "\"motor_integrity\":[" + String(gTelemetry.motorIntegrity[0] ? "true" : "false") + "," + String(gTelemetry.motorIntegrity[1] ? "true" : "false") + "," + String(gTelemetry.motorIntegrity[2] ? "true" : "false") + "," + String(gTelemetry.motorIntegrity[3] ? "true" : "false") + "],";
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

 bool verifyMotorIntegrity(uint8_t motorIndex) {
   setMotorSelection(-1);
   setEscPulseUs(AppConfig::ESC_STOP_PULSE_US);
   delay(30);

   int baseline = readAnalogAverage(
     AppConfig::RESIST_SENSE_PIN,
     AppConfig::INTEGRITY_SAMPLES,
     AppConfig::INTEGRITY_SAMPLE_DELAY_MS
   );

   setMotorSelection(motorIndex);
   setEscPulseUs(AppConfig::ESC_TEST_PULSE_US);
   delay(80);

   int active = readAnalogAverage(
     AppConfig::RESIST_SENSE_PIN,
     AppConfig::INTEGRITY_SAMPLES,
     AppConfig::INTEGRITY_SAMPLE_DELAY_MS
   );

   setEscPulseUs(AppConfig::ESC_STOP_PULSE_US);
   setMotorSelection(-1);

   int delta = abs(active - baseline);
   bool ok = delta >= AppConfig::INTEGRITY_THRESHOLD;

   Serial.print("Motor ");
   Serial.print(motorIndex + 1);
   Serial.print(" integrity: ");
   Serial.print(ok ? "OK" : "FAIL");
   Serial.print(" | baseline=");
   Serial.print(baseline);
   Serial.print(" active=");
   Serial.print(active);
   Serial.print(" delta=");
   Serial.println(delta);
   return ok;
 }

 void runStartupIntegrityChecks() {
   if (AppConfig::TEST_SERVER_MODE) {
     for (int i = 0; i < 4; i++) gTelemetry.motorIntegrity[i] = true;
     Serial.println("TEST_SERVER_MODE active: integrity checks mocked as PASS.");
     return;
   }

   Serial.println("Running startup motor integrity checks...");
   for (uint8_t i = 0; i < 4; i++) {
     gTelemetry.motorIntegrity[i] = verifyMotorIntegrity(i);
     delay(100);
   }
   Serial.println("Startup checks complete.");
 }

 void readLidarTelemetry() {
   if (AppConfig::TEST_SERVER_MODE) {
     generateMockLidar(gTelemetry.lidarCm);
   } else {
     for (uint8_t i = 0; i < 4; i++) {
       gTelemetry.lidarCm[i] = readDistanceFromAddr(AppConfig::LIDAR_ADDRS[i]);
     }
   }

   for (uint8_t i = 0; i < 4; i++) {
     Serial.print("Lidar ");
     Serial.print(i + 1);
     Serial.print(": ");
     Serial.print(gTelemetry.lidarCm[i]);
     Serial.println(" cm");
   }
 }

 void initPinsAndPwm() {
   pinMode(AppConfig::RESIST_SENSE_PIN, INPUT);
   for (int i = 0; i < 4; i++) {
     pinMode(AppConfig::MOTOR_SELECT_PINS[i], OUTPUT);
     digitalWrite(AppConfig::MOTOR_SELECT_PINS[i], LOW);
   }

   ledcAttach(AppConfig::ESC_SIGNAL_PIN, AppConfig::ESC_PWM_FREQ_HZ, AppConfig::ESC_PWM_RES_BITS);
   setEscPulseUs(AppConfig::ESC_STOP_PULSE_US);
 }

 void initCameraOrMock() {
   if (AppConfig::TEST_SERVER_MODE) {
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

   initPinsAndPwm();
   initCameraOrMock();
   Serial.println("Telemetry transport: Serial JSON (no WiFi build).");
   runStartupIntegrityChecks();
 }

 void loop() {
   gTelemetry.uptimeMs = millis();
   readLidarTelemetry();

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

