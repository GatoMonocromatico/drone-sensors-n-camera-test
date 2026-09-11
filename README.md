# drone-sensors-n-camera-test

ESP32-CAM firmware + a small Flask dashboard for bringing up and self-testing a
drone's sensor/motor wiring before flight: 4 I2C LiDAR range sensors, a
4-motor ESC integrity check, and camera capture, all streamed as one JSON
telemetry payload.

## What it does

**Firmware (`main.ino`, Arduino/ESP-IDF, AI-Thinker ESP32-CAM pinout):**

- Polls 4 I2C LiDAR distance sensors at fixed addresses (`0x62`–`0x65`) using
  their raw register protocol (trigger a read, then pull 2 bytes back).
- Runs a startup **motor integrity check**: for each of 4 motors, it selects
  the motor through a 4-way select-pin mux, drives a short test pulse on the
  shared ESC PWM line, and compares an analog current-sense reading against a
  baseline. A delta past a threshold means the motor responded; anything else
  fails the check before the vehicle ever arms.
- Captures a JPEG frame from the camera, base64-encodes it, and folds it into
  the same telemetry JSON.
- Sends everything over serial (USB) as one JSON line every 300ms — no WiFi
  in this build.
- A `TEST_SERVER_MODE` flag switches all of the above to synthetic data (mock
  sinusoidal LiDAR readings, a mock JPEG loaded from LittleFS) so the
  dashboard and downstream pipeline can be built and demoed without any
  sensor or motor hardware attached.

**Dashboard (`dashboard.py`, Flask):**

- Reads that JSON either over a serial port (background thread, pyserial) or
  via an HTTP `POST /data` endpoint, so the same dashboard works whether the
  board is tethered by USB or talking over a network later.
- Serves a live local-network page (`GET /`) that polls `/latest` every
  250ms and renders system status, all 4 LiDAR readings, and the latest
  camera frame inline.

## Stack

C++ (Arduino/ESP-IDF, ESP32-CAM) · Python (Flask, pyserial) · I2C · PWM

## Status

A hardware bring-up / pre-flight self-test tool, not flight software itself —
it answers "is every sensor and motor actually alive and wired correctly"
before a mission starts.
