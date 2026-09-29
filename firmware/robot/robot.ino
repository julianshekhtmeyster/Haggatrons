// Haggatrons robot: W11 camera + QMI8658 IMU + REV 2m range sensor + TB6612FNG,
// talking to the master over encrypted ESP-NOW.
//
// Safety model, enforced here regardless of what the Mac asks for:
//   - Motors are de-energized at boot (STBY low) and whenever no move runs.
//   - A move runs only while the latest master heartbeat is fresh
//     (HG_LINK_TIMEOUT_MS), armed, and not e-stopped.
//   - Every move has a firmware-capped duration and speed.
//   - Forward moves need a live range reading and stop when it drops below the
//     requested clearance.
//   - Turns need the IMU and stop at the measured angle or the timeout.
//
// USB bench commands (same as firmware/usb_camera): C, R<code>, O. Plus:
//   S  status line with this board's MAC
//   P  provisioning record (HgProvision), then restart
//   X  clear provisioning
#include "esp_camera.h"
#include "driver/gpio.h"
#include <Wire.h>
#include <SensorQMI8658.hpp>
#include <VL53L0X.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <HaggatronsProtocol.h>

static constexpr uint8_t FW_MAJOR = 1;
static constexpr uint8_t FW_MINOR = 0;

// Camera (fixed by the W11 board)
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
// Sensor I2C bus: onboard QMI8658 plus the REV 2m sensor (see PINOUT.md).
// The camera holds the chip's other I2C controller.
static constexpr int PIN_I2C_SDA = 5;
static constexpr int PIN_I2C_SCL = 4;
// TB6612FNG (see PINOUT.md)
static constexpr int PIN_AIN1 = 1;
static constexpr int PIN_AIN2 = 2;
static constexpr int PIN_PWMA = 3;
static constexpr int PIN_PWMB = 6;
static constexpr int PIN_BIN1 = 7;
static constexpr int PIN_BIN2 = 8;
static constexpr int PIN_STBY = 9;

static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t PWM_BITS = 10;
static constexpr uint16_t RANGE_MAX_MM = 2000;
static constexpr uint32_t RANGE_STALE_MS = 150;
static constexpr uint16_t DEFAULT_CLEAR_MM = 150;
static constexpr uint32_t CONTROL_PERIOD_US = 10000;
static constexpr float TURN_TOLERANCE_DEG = 2.0f;

SensorQMI8658 imu;
VL53L0X range;
Preferences settings;

struct Config {
  bool provisioned = false;
  uint8_t robotId = 0;
  uint8_t masterMac[6] = {};
  uint8_t channel = 1;
  uint8_t pmk[HG_KEY_LEN] = {};
  uint8_t lmk[HG_KEY_LEN] = {};
  uint8_t motorFlags = 0;
  uint8_t yawAxis = 2;
  int8_t yawSign = 1;
  uint16_t maxSpeed = 600;
} config;

struct RxItem {
  uint8_t mac[6];
  uint16_t length;
  uint8_t data[HG_ESPNOW_MAX_PAYLOAD];
};

struct ActiveMove {
  bool running = false;
  uint32_t reqId = 0;
  uint8_t kind = 0;
  int16_t left = 0, right = 0;
  int16_t appliedLeft = 0, appliedRight = 0;
  uint32_t startMs = 0;
  uint16_t durationMs = 0;
  float targetDeg = 0;
  float yawDeg = 0;
  uint16_t minClearMm = DEFAULT_CLEAR_MM;
  uint16_t minRangeMm = 0xFFFF;
  uint32_t lastUs = 0;
} move;

static bool cameraReady = false;
static bool imuReady = false;
static bool rangeReady = false;
static bool espNowReady = false;
static uint16_t maxPayload = ESP_NOW_MAX_DATA_LEN;
static uint16_t chunkSize = 200;
static QueueHandle_t rxQueue;
static RxItem rxScratch;
static SemaphoreHandle_t sendDone;
static volatile bool sendOk = false;
static uint16_t txSeq = 0;

static uint32_t lastHeartbeatMs = 0;
static bool heartbeatSeen = false;
static uint8_t heartbeatFlags = HG_HB_ESTOP;
static uint8_t lastOutcome = HG_OUT_COMPLETED;

static uint16_t rangeMm = 0;
static uint32_t rangeMs = 0;
static bool rangeValid = false;
static float accel[3] = {}, gyro[3] = {};
static bool imuValid = false;

static uint8_t *frameBuffer = nullptr;
static uint32_t frameLength = 0;
static uint32_t frameReqId = 0;
static const size_t FRAME_CAPACITY = 400000;

// ---------------------------------------------------------------- motors
static void motorsOff() {
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  digitalWrite(PIN_STBY, LOW);
}

static void setChannel(int in1, int in2, int pwm, int16_t permille) {
  const uint32_t duty = (uint32_t)abs(permille) * ((1u << PWM_BITS) - 1) / 1000;
  digitalWrite(in1, permille > 0 ? HIGH : LOW);
  digitalWrite(in2, permille < 0 ? HIGH : LOW);
  ledcWrite(pwm, permille == 0 ? 0 : duty);
}

static int16_t clampSpeed(int32_t permille) {
  const int32_t cap = min<int32_t>(config.maxSpeed, 1000);
  return (int16_t)constrain(permille, -cap, cap);
}

// Drive with left/right in robot terms; applies wiring flags and the speed cap.
static void drive(int16_t left, int16_t right) {
  left = clampSpeed(left);
  right = clampSpeed(right);
  move.appliedLeft = left;
  move.appliedRight = right;
  if (config.motorFlags & HG_MOTOR_INVERT_LEFT) left = -left;
  if (config.motorFlags & HG_MOTOR_INVERT_RIGHT) right = -right;
  int16_t channelA = left, channelB = right;
  if (config.motorFlags & HG_MOTOR_SWAP_SIDES) { channelA = right; channelB = left; }
  if (channelA == 0 && channelB == 0) { motorsOff(); return; }
  digitalWrite(PIN_STBY, HIGH);
  setChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA, channelA);
  setChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB, channelB);
}

// ---------------------------------------------------------------- sensors
static void pollRange() {
  if (!rangeReady) return;
  if ((range.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  const uint16_t mm = range.readRangeContinuousMillimeters();
  if (range.timeoutOccurred()) { rangeValid = false; return; }
  // The sensor reports ~8190 when nothing is within range.
  rangeMm = mm > RANGE_MAX_MM ? RANGE_MAX_MM : mm;
  rangeMs = millis();
  rangeValid = true;
}

static bool rangeFresh() {
  return rangeReady && rangeValid && millis() - rangeMs < RANGE_STALE_MS;
}

static void readImu() {
  imuValid = imuReady && imu.getAccelerometer(accel[0], accel[1], accel[2]) &&
             imu.getGyroscope(gyro[0], gyro[1], gyro[2]);
}

static float yawRate() {
  return gyro[config.yawAxis > 2 ? 2 : config.yawAxis] * (config.yawSign < 0 ? -1.0f : 1.0f);
}

// ---------------------------------------------------------------- link state
static bool linkOk() {
  return heartbeatSeen && millis() - lastHeartbeatMs < HG_LINK_TIMEOUT_MS;
}

static bool armed() {
  return linkOk() && (heartbeatFlags & HG_HB_ARMED) && !(heartbeatFlags & HG_HB_ESTOP);
}

// ---------------------------------------------------------------- ESP-NOW out
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if (length <= 0 || length > (int)sizeof(rxScratch.data)) return;
  memcpy(rxScratch.mac, info->src_addr, 6);
  rxScratch.length = length;
  memcpy(rxScratch.data, data, length);
  xQueueSend(rxQueue, &rxScratch, 0);
}

static void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  sendOk = status == ESP_NOW_SEND_SUCCESS;
  xSemaphoreGive(sendDone);
}

// Sends one message and waits for the MAC-layer acknowledgement.
static bool sendMessage(uint8_t type, const void *body, size_t bodyLength,
                        const uint8_t *extra = nullptr, size_t extraLength = 0, int attempts = 4) {
  if (!espNowReady) return false;
  static uint8_t buffer[HG_ESPNOW_MAX_PAYLOAD];
  const size_t length = sizeof(HgHeader) + bodyLength + extraLength;
  if (length > maxPayload) return false;
  HgHeader header = {HG_MAGIC, HG_PROTOCOL_VERSION, type, config.robotId, ++txSeq};
  memcpy(buffer, &header, sizeof(header));
  memcpy(buffer + sizeof(header), body, bodyLength);
  if (extraLength) memcpy(buffer + sizeof(header) + bodyLength, extra, extraLength);
  for (int attempt = 0; attempt < attempts; ++attempt) {
    xSemaphoreTake(sendDone, 0);
    if (esp_now_send(config.masterMac, buffer, length) != ESP_OK) { delay(2); continue; }
    if (xSemaphoreTake(sendDone, pdMS_TO_TICKS(60)) == pdTRUE && sendOk) return true;
  }
  return false;
}

static void sendReject(uint32_t reqId, uint8_t refType, uint8_t code) {
  HgRejectMsg reject = {reqId, refType, code};
  sendMessage(HG_MSG_REJECT, &reject, sizeof(reject));
}

static void sendTelemetry() {
  HgTelemetry t = {};
  t.uptime_ms = millis();
  uint16_t flags = 0;
  if (armed()) flags |= HG_TEL_ARMED;
  if (heartbeatFlags & HG_HB_ESTOP) flags |= HG_TEL_ESTOP;
  if (linkOk()) flags |= HG_TEL_LINK_OK;
  if (imuValid) flags |= HG_TEL_IMU_OK;
  if (rangeFresh()) flags |= HG_TEL_RANGE_OK;
  if (move.running) flags |= HG_TEL_MOVING;
  if (cameraReady) flags |= HG_TEL_CAMERA_OK;
  t.flags = flags;
  const uint32_t age = heartbeatSeen ? millis() - lastHeartbeatMs : 0xFFFF;
  t.heartbeat_age_ms = age > 0xFFFF ? 0xFFFF : age;
  memcpy(t.accel_g, accel, sizeof(accel));
  memcpy(t.gyro_dps, gyro, sizeof(gyro));
  t.range_mm = rangeFresh() ? rangeMm : 0;
  t.active_req = move.running ? move.reqId : 0;
  t.last_outcome = lastOutcome;
  t.fw_major = FW_MAJOR;
  t.fw_minor = FW_MINOR;
  sendMessage(HG_MSG_TELEMETRY, &t, sizeof(t), nullptr, 0, 1);
}

// ---------------------------------------------------------------- moves
static void finishMove(uint8_t outcome) {
  motorsOff();
  if (!move.running) return;
  move.running = false;
  lastOutcome = outcome;
  HgMoveResult result = {};
  result.req_id = move.reqId;
  result.outcome = outcome;
  result.kind = move.kind;
  result.elapsed_ms = min<uint32_t>(millis() - move.startMs, 0xFFFF);
  result.yaw_deg = move.yawDeg;
  result.min_range_mm = move.minRangeMm;
  result.left = move.appliedLeft;
  result.right = move.appliedRight;
  sendMessage(HG_MSG_MOVE_RESULT, &result, sizeof(result));
}

static void startMove(const HgMove &request) {
  const auto reject = [&](uint8_t code) { sendReject(request.req_id, HG_MSG_MOVE, code); };
  if (heartbeatFlags & HG_HB_ESTOP) return reject(HG_REJ_ESTOP);
  if (!linkOk()) return reject(HG_REJ_LINK_DOWN);
  if (!armed()) return reject(HG_REJ_NOT_ARMED);
  if (move.running) return reject(HG_REJ_BUSY);
  if (request.duration_ms == 0 || request.duration_ms > HG_MAX_MOVE_MS ||
      abs(request.left) > 1000 || abs(request.right) > 1000)
    return reject(HG_REJ_BAD_PARAMS);

  const uint16_t clear = request.min_clear_mm ? request.min_clear_mm : DEFAULT_CLEAR_MM;
  uint16_t duration = request.duration_ms;
  switch (request.kind) {
    case HG_MOVE_DRIVE:
      if (!rangeFresh()) duration = min<uint16_t>(duration, HG_MAX_JOG_MS_WITHOUT_RANGE);
      else if (request.left > 0 && request.right > 0 && rangeMm < clear) return reject(HG_REJ_OBSTACLE);
      break;
    case HG_MOVE_TURN:
      if (request.turn_ddeg == 0 || abs(request.turn_ddeg) > HG_MAX_TURN_DDEG) return reject(HG_REJ_BAD_PARAMS);
      if (!imuValid) return reject(HG_REJ_IMU_UNAVAILABLE);
      break;
    case HG_MOVE_FORWARD:
      if (request.left <= 0 || request.right <= 0) return reject(HG_REJ_BAD_PARAMS);
      if (!rangeFresh()) return reject(HG_REJ_RANGE_UNAVAILABLE);
      if (rangeMm < clear) return reject(HG_REJ_OBSTACLE);
      break;
    default:
      return reject(HG_REJ_BAD_PARAMS);
  }

  move = ActiveMove();
  move.running = true;
  move.reqId = request.req_id;
  move.kind = request.kind;
  move.left = request.left;
  move.right = request.right;
  move.startMs = millis();
  move.durationMs = duration;
  move.targetDeg = request.turn_ddeg / 10.0f;
  move.minClearMm = clear;
  move.lastUs = micros();
  if (move.kind == HG_MOVE_TURN) {
    const int16_t speed = max<int16_t>(abs(request.left), 300);
    drive(move.targetDeg > 0 ? -speed : speed, move.targetDeg > 0 ? speed : -speed);
  } else {
    drive(move.left, move.right);
  }
}

static void controlMove() {
  if (!move.running) return;
  const uint32_t nowUs = micros();
  if (nowUs - move.lastUs < CONTROL_PERIOD_US) return;
  const float dt = (nowUs - move.lastUs) / 1e6f;
  move.lastUs = nowUs;

  if (!armed()) {
    if (heartbeatFlags & HG_HB_ESTOP) return finishMove(HG_OUT_ESTOP);
    if (!linkOk()) return finishMove(HG_OUT_LINK_LOST);
    return finishMove(HG_OUT_DISARMED);
  }
  readImu();
  if (imuValid) move.yawDeg += yawRate() * dt;
  const bool fresh = rangeFresh();
  if (fresh && rangeMm < move.minRangeMm) move.minRangeMm = rangeMm;
  const uint32_t elapsed = millis() - move.startMs;

  switch (move.kind) {
    case HG_MOVE_DRIVE:
      if (move.left > 0 && move.right > 0 && fresh && rangeMm < move.minClearMm)
        return finishMove(HG_OUT_OBSTACLE);
      break;
    case HG_MOVE_FORWARD: {
      if (!fresh) return finishMove(HG_OUT_RANGE_LOST);
      if (rangeMm < move.minClearMm) return finishMove(HG_OUT_OBSTACLE);
      // Hold heading: yawing left (positive) speeds up the left wheel.
      const int32_t trim = imuValid ? constrain((int32_t)(move.yawDeg * 8.0f), -150, 150) : 0;
      drive(move.left + trim, move.right - trim);
      break;
    }
    case HG_MOVE_TURN: {
      if (!imuValid) return finishMove(HG_OUT_TURN_TIMEOUT);
      const float remaining = fabsf(move.targetDeg) - fabsf(move.yawDeg);
      if (remaining <= TURN_TOLERANCE_DEG) return finishMove(HG_OUT_COMPLETED);
      int16_t speed = max<int16_t>(abs(move.left), 300);
      if (remaining < 20.0f) speed = max<int16_t>(speed * 6 / 10, 300);
      drive(move.targetDeg > 0 ? -speed : speed, move.targetDeg > 0 ? speed : -speed);
      break;
    }
  }
  if (elapsed >= move.durationMs)
    finishMove(move.kind == HG_MOVE_TURN ? HG_OUT_TURN_TIMEOUT : HG_OUT_COMPLETED);
}

// ---------------------------------------------------------------- camera
static bool setFramesize(uint8_t code) {
  framesize_t size;
  switch (code) {
    case 0: size = FRAMESIZE_QVGA; break;
    case 1: size = FRAMESIZE_VGA; break;
    case 2: size = FRAMESIZE_SVGA; break;
    case 3: size = FRAMESIZE_XGA; break;
    default: return false;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) return false;
  if (sensor->status.framesize == size) return true;
  if (sensor->set_framesize(sensor, size) != 0) return false;
  delay(150);
  return true;
}

// Grabs a fresh JPEG into frameBuffer. Returns width/height via arguments.
static bool captureFrame(uint16_t &width, uint16_t &height) {
  // With two frame buffers the first grab may predate this request.
  camera_fb_t *frame = esp_camera_fb_get();
  if (frame) esp_camera_fb_return(frame);
  frame = esp_camera_fb_get();
  if (!frame) return false;
  const bool ok = frame->format == PIXFORMAT_JPEG && frame->len > 0 && frame->len <= FRAME_CAPACITY;
  if (ok) {
    memcpy(frameBuffer, frame->buf, frame->len);
    frameLength = frame->len;
    width = frame->width;
    height = frame->height;
  }
  esp_camera_fb_return(frame);
  return ok;
}

static bool sendChunk(uint16_t index) {
  if (!frameLength || index * (uint32_t)chunkSize >= frameLength) return false;
  const uint32_t offset = index * (uint32_t)chunkSize;
  const uint32_t length = min<uint32_t>(chunkSize, frameLength - offset);
  HgFrameChunk chunk = {frameReqId, index};
  return sendMessage(HG_MSG_FRAME_CHUNK, &chunk, sizeof(chunk), frameBuffer + offset, length);
}

static void handleCapture(const HgCapture &request) {
  if (move.running) return sendReject(request.req_id, HG_MSG_CAPTURE, HG_REJ_BUSY);
  uint16_t width = 0, height = 0;
  sensor_t *sensor = esp_camera_sensor_get();
  if (!cameraReady || !sensor || !setFramesize(request.framesize)) {
    return sendReject(request.req_id, HG_MSG_CAPTURE, HG_REJ_CAMERA_FAILED);
  }
  if (request.quality >= 8 && request.quality <= 40) sensor->set_quality(sensor, request.quality);
  frameLength = 0;
  const uint32_t frameMs = millis();
  if (!captureFrame(width, height)) return sendReject(request.req_id, HG_MSG_CAPTURE, HG_REJ_CAMERA_FAILED);
  readImu();
  pollRange();
  frameReqId = request.req_id;

  HgFrameMeta meta = {};
  meta.req_id = frameReqId;
  meta.jpeg_len = frameLength;
  meta.chunk_size = chunkSize;
  meta.chunk_count = (frameLength + chunkSize - 1) / chunkSize;
  meta.width = width;
  meta.height = height;
  meta.frame_ms = frameMs;
  meta.imu_ms = millis();
  meta.imu_valid = imuValid;
  meta.range_valid = rangeFresh();
  meta.range_mm = meta.range_valid ? rangeMm : 0;
  memcpy(meta.accel_g, accel, sizeof(accel));
  memcpy(meta.gyro_dps, gyro, sizeof(gyro));
  if (!sendMessage(HG_MSG_FRAME_META, &meta, sizeof(meta))) return;
  for (uint16_t i = 0; i < meta.chunk_count; ++i) sendChunk(i);  // gaps are resent on request
}

static void handleResend(const uint8_t *body, size_t length) {
  if (length < sizeof(HgResend)) return;
  HgResend request;
  memcpy(&request, body, sizeof(request));
  if (length != sizeof(request) + request.count * 2u) return;
  if (request.req_id != frameReqId || !frameLength)
    return sendReject(request.req_id, HG_MSG_RESEND, HG_REJ_FRAME_GONE);
  for (uint8_t i = 0; i < request.count; ++i) {
    uint16_t index;
    memcpy(&index, body + sizeof(request) + i * 2, 2);
    sendChunk(index);
  }
}

// ---------------------------------------------------------------- ESP-NOW in
static void handleMessage(const RxItem &item) {
  if (memcmp(item.mac, config.masterMac, 6) != 0 || item.length < sizeof(HgHeader)) return;
  HgHeader header;
  memcpy(&header, item.data, sizeof(header));
  if (header.magic != HG_MAGIC || header.version != HG_PROTOCOL_VERSION || header.robot_id != config.robotId) return;
  const uint8_t *body = item.data + sizeof(header);
  const size_t length = item.length - sizeof(header);
  switch (header.type) {
    case HG_MSG_HEARTBEAT:
      if (length == sizeof(HgHeartbeat)) {
        HgHeartbeat beat;
        memcpy(&beat, body, sizeof(beat));
        heartbeatFlags = beat.flags;
        lastHeartbeatMs = millis();
        heartbeatSeen = true;
      }
      break;
    case HG_MSG_STOP:
      if (length == sizeof(HgStop)) {
        HgStop stop;
        memcpy(&stop, body, sizeof(stop));
        if (stop.req_id == 0 || stop.req_id == move.reqId)
          finishMove((heartbeatFlags & HG_HB_ESTOP) ? HG_OUT_ESTOP : HG_OUT_HOST_STOP);
        motorsOff();
      }
      break;
    case HG_MSG_MOVE:
      if (length == sizeof(HgMove)) {
        HgMove request;
        memcpy(&request, body, sizeof(request));
        startMove(request);
      }
      break;
    case HG_MSG_CAPTURE:
      if (length == sizeof(HgCapture)) {
        HgCapture request;
        memcpy(&request, body, sizeof(request));
        handleCapture(request);
      }
      break;
    case HG_MSG_RESEND:
      handleResend(body, length);
      break;
  }
}

// ---------------------------------------------------------------- setup & USB
static void loadConfig() {
  settings.begin("hgbot", true);
  config.provisioned = settings.getBool("prov", false);
  if (config.provisioned) {
    config.robotId = settings.getUChar("id", 0);
    settings.getBytes("master", config.masterMac, 6);
    config.channel = settings.getUChar("ch", 1);
    settings.getBytes("pmk", config.pmk, HG_KEY_LEN);
    settings.getBytes("lmk", config.lmk, HG_KEY_LEN);
    config.motorFlags = settings.getUChar("motor", 0);
    config.yawAxis = settings.getUChar("yawaxis", 2);
    config.yawSign = settings.getChar("yawsign", 1);
    config.maxSpeed = settings.getUShort("maxspd", 600);
  }
  settings.end();
}

static void startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (!config.provisioned) return;
  esp_wifi_set_channel(config.channel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return;
  uint32_t version = 1;
  if (esp_now_get_version(&version) == ESP_OK && version >= 2) maxPayload = HG_ESPNOW_MAX_PAYLOAD;
  chunkSize = min<uint16_t>(HG_CHUNK_DATA, maxPayload - sizeof(HgHeader) - sizeof(HgFrameChunk));
  esp_now_set_pmk(config.pmk);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, config.masterMac, 6);
  memcpy(peer.lmk, config.lmk, HG_KEY_LEN);
  peer.channel = config.channel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK) return;
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  espNowReady = true;
}

static void printMac(const uint8_t *mac) {
  Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void printStatus() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.print("HGBOT fw=");
  Serial.printf("%u.%u mac=", FW_MAJOR, FW_MINOR);
  printMac(mac);
  Serial.printf(" provisioned=%d id=%u master=", config.provisioned, config.robotId);
  printMac(config.masterMac);
  Serial.printf(" ch=%u espnow=%d camera=%d imu=%d range=%d link=%d armed=%d\n", config.channel, espNowReady,
                cameraReady, imuReady, rangeReady, linkOk(), armed());
}

static void provision() {
  HgProvision record;
  Serial.setTimeout(2000);
  if (Serial.readBytes(reinterpret_cast<uint8_t *>(&record), sizeof(record)) != sizeof(record) ||
      record.version != HG_PROTOCOL_VERSION || record.robot_id == 0 || record.robot_id > HG_MAX_ROBOTS ||
      record.channel < 1 || record.channel > 13 || record.yaw_axis > 2 || record.max_speed > 1000) {
    Serial.println("ERR invalid provisioning record");
    return;
  }
  settings.begin("hgbot", false);
  settings.putUChar("id", record.robot_id);
  settings.putBytes("master", record.master_mac, 6);
  settings.putUChar("ch", record.channel);
  settings.putBytes("pmk", record.pmk, HG_KEY_LEN);
  settings.putBytes("lmk", record.lmk, HG_KEY_LEN);
  settings.putUChar("motor", record.motor_flags);
  settings.putUChar("yawaxis", record.yaw_axis);
  settings.putChar("yawsign", record.yaw_sign < 0 ? -1 : 1);
  settings.putUShort("maxspd", record.max_speed);
  settings.putBool("prov", true);
  settings.end();
  Serial.println("PROV OK");
  Serial.flush();
  delay(100);
  ESP.restart();
}

static void benchCommand(char command) {
  if (command == 'S') return printStatus();
  if (command == 'P') return provision();
  if (command == 'X') {
    settings.begin("hgbot", false);
    settings.clear();
    settings.end();
    Serial.println("PROV CLEARED");
    Serial.flush();
    delay(100);
    ESP.restart();
  }
  if (command == 'R') {
    const uint32_t started = millis();
    while (!Serial.available() && millis() - started < 1000) delay(1);
    const int code = Serial.read();
    if (code < '0' || code > '3' || !setFramesize(code - '0')) {
      Serial.println("ERR resolution change failed");
      return;
    }
    Serial.println("RES OK");
    return;
  }
  if (command != 'C' && command != 'O') return;
  if (move.running) { Serial.println("ERR moving"); return; }
  uint16_t width, height;
  const uint32_t frameMs = millis();
  if (!cameraReady || !captureFrame(width, height)) { Serial.println("ERR capture failed"); return; }
  frameReqId = 0;
  const uint32_t size = frameLength;
  if (command == 'C') {
    const uint8_t header[8] = {'C', 'A', 'M', '1', (uint8_t)size, (uint8_t)(size >> 8), (uint8_t)(size >> 16),
                               (uint8_t)(size >> 24)};
    Serial.write(header, sizeof(header));
  } else {
    readImu();
    const uint32_t imuMs = millis();
    const uint8_t header[17] = {'O', 'B', 'S', '1',
                                (uint8_t)size, (uint8_t)(size >> 8), (uint8_t)(size >> 16), (uint8_t)(size >> 24),
                                (uint8_t)frameMs, (uint8_t)(frameMs >> 8), (uint8_t)(frameMs >> 16), (uint8_t)(frameMs >> 24),
                                (uint8_t)imuMs, (uint8_t)(imuMs >> 8), (uint8_t)(imuMs >> 16), (uint8_t)(imuMs >> 24),
                                (uint8_t)imuValid};
    float measurements[6] = {accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]};
    Serial.write(header, sizeof(header));
    Serial.write(reinterpret_cast<const uint8_t *>(measurements), sizeof(measurements));
  }
  Serial.write(frameBuffer, frameLength);
  Serial.flush();
}

static void initCamera() {
  // GPIO48 is shared with the W11 RGB LED. Release it for camera D7.
  pinMode(48, INPUT);
  gpio_set_pull_mode(GPIO_NUM_48, GPIO_FLOATING);
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delay(200);
  camera_config_t camera = {};
  camera.ledc_channel = LEDC_CHANNEL_0;
  camera.ledc_timer = LEDC_TIMER_0;
  camera.pin_d0 = PIN_D0;
  camera.pin_d1 = PIN_D1;
  camera.pin_d2 = PIN_D2;
  camera.pin_d3 = PIN_D3;
  camera.pin_d4 = PIN_D4;
  camera.pin_d5 = PIN_D5;
  camera.pin_d6 = PIN_D6;
  camera.pin_d7 = PIN_D7;
  camera.pin_xclk = PIN_XCLK;
  camera.pin_pclk = PIN_PCLK;
  camera.pin_vsync = PIN_VSYNC;
  camera.pin_href = PIN_HREF;
  camera.pin_sccb_sda = PIN_SDA;
  camera.pin_sccb_scl = PIN_SCL;
  camera.pin_pwdn = -1;
  camera.pin_reset = -1;
  camera.xclk_freq_hz = 20000000;
  camera.pixel_format = PIXFORMAT_JPEG;
  camera.frame_size = FRAMESIZE_UXGA;  // allocate for large frames, then run at QVGA
  camera.jpeg_quality = 12;
  camera.fb_count = psramFound() ? 2 : 1;
  camera.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  camera.grab_mode = camera.fb_count == 2 ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  if (esp_camera_init(&camera) != ESP_OK) return;
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  frameBuffer = static_cast<uint8_t *>(ps_malloc(FRAME_CAPACITY));
  cameraReady = frameBuffer != nullptr;
}

static void initSensors() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  imuReady = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL);
  if (imuReady) {
    imu.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
    // 512 dps: an N20 robot turning in place easily exceeds the old 64 dps range.
    imu.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_3);
    imu.enableAccelerometer();
    imu.enableGyroscope();
  }
  range.setBus(&Wire);
  range.setTimeout(50);
  rangeReady = range.init();
  if (rangeReady) range.startContinuous();
}

void setup() {
  // Motors off before anything else can run.
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  for (int pin : {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS);
  motorsOff();

  Serial.setRxBufferSize(1024);
  Serial.begin(921600);
  rxQueue = xQueueCreate(16, sizeof(RxItem));
  sendDone = xSemaphoreCreateBinary();
  loadConfig();
  initCamera();
  initSensors();
  startEspNow();
  Serial.println(cameraReady ? "READY CAM1 OBS1 HGBOT" : "ERR camera init");
}

void loop() {
  static RxItem item;
  while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) handleMessage(item);
  pollRange();
  controlMove();
  if (!move.running) {
    // Belt and braces: nothing should be energized between moves.
    static uint32_t lastOffMs = 0;
    if (millis() - lastOffMs > 50) { lastOffMs = millis(); motorsOff(); }
  }
  static uint32_t lastTelemetryMs = 0;
  if (millis() - lastTelemetryMs >= HG_TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    if (!move.running) readImu();
    if (config.provisioned) sendTelemetry();
  }
  if (Serial.available() > 0) benchCommand(Serial.read());
  delay(1);
}
