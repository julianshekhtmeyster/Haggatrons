#include "esp_camera.h"
#include "driver/gpio.h"
#include <Wire.h>
#include <SensorQMI8658.hpp>

// Meshnology W11 camera wiring matches the XIAO ESP32-S3 camera pins.
static constexpr int PIN_XCLK = 10;
static constexpr int PIN_SDA = 40;
static constexpr int PIN_SCL = 39;
static constexpr int PIN_D0 = 15;
static constexpr int PIN_D1 = 17;
static constexpr int PIN_D2 = 18;
static constexpr int PIN_D3 = 16;
static constexpr int PIN_D4 = 14;
static constexpr int PIN_D5 = 12;
static constexpr int PIN_D6 = 11;
static constexpr int PIN_D7 = 48;
static constexpr int PIN_VSYNC = 38;
static constexpr int PIN_HREF = 47;
static constexpr int PIN_PCLK = 13;
static constexpr int PIN_IMU_SDA = 5;
static constexpr int PIN_IMU_SCL = 4;

SensorQMI8658 imu;
bool imuReady = false;

static void sendError(const char *message) {
  Serial.print("ERR ");
  Serial.println(message);
}

void setup() {
  Serial.begin(921600);
  delay(500);

  // GPIO48 is shared with the W11 RGB LED. Release it for camera D7.
  pinMode(48, INPUT);
  gpio_set_pull_mode(GPIO_NUM_48, GPIO_FLOATING);
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delay(200);

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = PIN_D0;
  config.pin_d1 = PIN_D1;
  config.pin_d2 = PIN_D2;
  config.pin_d3 = PIN_D3;
  config.pin_d4 = PIN_D4;
  config.pin_d5 = PIN_D5;
  config.pin_d6 = PIN_D6;
  config.pin_d7 = PIN_D7;
  config.pin_xclk = PIN_XCLK;
  config.pin_pclk = PIN_PCLK;
  config.pin_vsync = PIN_VSYNC;
  config.pin_href = PIN_HREF;
  config.pin_sccb_sda = PIN_SDA;
  config.pin_sccb_scl = PIN_SCL;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Allocate for higher-resolution snapshots, then start at QVGA for preview.
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = config.fb_count == 2 ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t result = esp_camera_init(&config);
  if (result != ESP_OK) {
    char message[32];
    snprintf(message, sizeof(message), "camera init 0x%x", result);
    sendError(message);
    return;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  }

  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
  imuReady = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS,
                       PIN_IMU_SDA, PIN_IMU_SCL);
  if (imuReady) {
    imu.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                            SensorQMI8658::ACC_ODR_1000Hz,
                            SensorQMI8658::LPF_MODE_0);
    imu.configGyroscope(SensorQMI8658::GYR_RANGE_64DPS,
                        SensorQMI8658::GYR_ODR_896_8Hz,
                        SensorQMI8658::LPF_MODE_3);
    imu.enableAccelerometer();
    imu.enableGyroscope();
  }
  Serial.println(imuReady ? "READY CAM1 OBS1 IMU" : "READY CAM1 OBS1 NO_IMU");
}

void loop() {
  if (Serial.available() <= 0) {
    delay(2);
    return;
  }
  char command = Serial.read();
  if (command == 'R') {
    unsigned long started = millis();
    while (!Serial.available() && millis() - started < 1000) {
      delay(1);
    }
    if (!Serial.available()) {
      sendError("missing resolution code");
      return;
    }
    char code = Serial.read();
    framesize_t size;
    switch (code) {
      case '0': size = FRAMESIZE_QVGA; break;
      case '1': size = FRAMESIZE_VGA; break;
      case '2': size = FRAMESIZE_SVGA; break;
      case '3': size = FRAMESIZE_XGA; break;
      default:
        sendError("unknown resolution code");
        return;
    }
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor || sensor->set_framesize(sensor, size) != 0) {
      sendError("resolution change failed");
      return;
    }
    delay(150);
    Serial.println("RES OK");
    return;
  }
  if (command != 'C' && command != 'O') {
    return;
  }
  camera_fb_t *frame = esp_camera_fb_get();
  if (!frame) {
    sendError("capture failed");
    return;
  }
  if (frame->format != PIXFORMAT_JPEG || frame->len > 1000000) {
    esp_camera_fb_return(frame);
    sendError("invalid JPEG frame");
    return;
  }
  const uint32_t size = static_cast<uint32_t>(frame->len);
  if (command == 'O') {
    const uint32_t frameMs = millis();
    float measurements[6] = {};
    bool valid = imuReady &&
                 imu.getAccelerometer(measurements[0], measurements[1], measurements[2]) &&
                 imu.getGyroscope(measurements[3], measurements[4], measurements[5]);
    const uint32_t imuMs = millis();
    uint8_t header[17] = {
      'O', 'B', 'S', '1',
      static_cast<uint8_t>(size), static_cast<uint8_t>(size >> 8),
      static_cast<uint8_t>(size >> 16), static_cast<uint8_t>(size >> 24),
      static_cast<uint8_t>(frameMs), static_cast<uint8_t>(frameMs >> 8),
      static_cast<uint8_t>(frameMs >> 16), static_cast<uint8_t>(frameMs >> 24),
      static_cast<uint8_t>(imuMs), static_cast<uint8_t>(imuMs >> 8),
      static_cast<uint8_t>(imuMs >> 16), static_cast<uint8_t>(imuMs >> 24),
      static_cast<uint8_t>(valid ? 1 : 0)};
    Serial.write(header, sizeof(header));
    Serial.write(reinterpret_cast<const uint8_t *>(measurements), sizeof(measurements));
    Serial.write(frame->buf, frame->len);
    Serial.flush();
    esp_camera_fb_return(frame);
    return;
  }
  uint8_t header[8] = {'C', 'A', 'M', '1',
                       static_cast<uint8_t>(size),
                       static_cast<uint8_t>(size >> 8),
                       static_cast<uint8_t>(size >> 16),
                       static_cast<uint8_t>(size >> 24)};
  Serial.write(header, sizeof(header));
  Serial.write(frame->buf, frame->len);
  Serial.flush();
  esp_camera_fb_return(frame);
}
