// Haggatrons master: USB serial <-> ESP-NOW relay plugged into the Mac.
//
// The master holds no secrets across reboots. On connect the Mac sends
// HG_SER_CONFIG with the radio channel, the primary master key, and each
// robot's MAC and local master key. The master then:
//   - forwards host messages to robots and robot messages to the host,
//   - sends every robot a heartbeat every HG_HEARTBEAT_PERIOD_MS carrying
//     the host's arm / e-stop state,
//   - drops to e-stop by itself when the host goes quiet for
//     HG_HOST_TIMEOUT_MS, so a hung Mac cannot leave robots armed.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HaggatronsProtocol.h>

static constexpr uint8_t FW_MAJOR = 1;
static constexpr uint8_t FW_MINOR = 0;
static constexpr int PIN_RGB = 48;
static constexpr size_t RX_QUEUE_DEPTH = 24;
static constexpr size_t SERIAL_MAX_RAW = 1 + 2 + HG_ESPNOW_MAX_PAYLOAD + 2;
static constexpr size_t SERIAL_MAX_ENCODED = SERIAL_MAX_RAW + SERIAL_MAX_RAW / 254 + 2;

struct Peer {
  uint8_t robotId;
  uint8_t mac[6];
  volatile uint32_t lastRxMs;
  volatile bool heard;
  volatile uint32_t txOk;
  volatile uint32_t txFail;
};

struct RxItem {
  uint8_t mac[6];
  int8_t rssi;
  uint16_t length;
  uint8_t data[HG_ESPNOW_MAX_PAYLOAD];
};

static Peer peers[HG_MAX_ROBOTS];
static uint8_t peerCount = 0;
static uint8_t radioChannel = 1;
static bool configured = false;
static uint8_t hostFlags = HG_HB_ESTOP;  // boot safe: disarmed and stopped
static uint32_t lastHostMs = 0;
static bool hostSeen = false;
static uint16_t heartbeatSeq = 0;
static uint16_t maxPayload = ESP_NOW_MAX_DATA_LEN;
static QueueHandle_t rxQueue;
static RxItem rxScratch;  // only touched from the Wi-Fi task callback
static uint8_t serialIn[SERIAL_MAX_ENCODED];
static size_t serialInLength = 0;
static bool serialInOverflow = false;

static Peer *peerByMac(const uint8_t *mac) {
  for (uint8_t i = 0; i < peerCount; ++i)
    if (memcmp(peers[i].mac, mac, 6) == 0) return &peers[i];
  return nullptr;
}

static Peer *peerById(uint8_t robotId) {
  for (uint8_t i = 0; i < peerCount; ++i)
    if (peers[i].robotId == robotId) return &peers[i];
  return nullptr;
}

static bool hostOk() {
  return hostSeen && millis() - lastHostMs < HG_HOST_TIMEOUT_MS;
}

static uint8_t broadcastFlags() {
  if (!hostOk()) return HG_HB_ESTOP;
  uint8_t flags = (hostFlags & (HG_HB_ARMED | HG_HB_ESTOP)) | HG_HB_HOST_OK;
  if (flags & HG_HB_ESTOP) flags &= ~HG_HB_ARMED;
  return flags;
}

// ---------------------------------------------------------------- serial out
static void sendFrame(uint8_t type, const uint8_t *a, size_t aLength,
                      const uint8_t *b = nullptr, size_t bLength = 0) {
  static uint8_t raw[SERIAL_MAX_RAW];
  static uint8_t encoded[SERIAL_MAX_ENCODED];
  const size_t length = 1 + aLength + bLength;
  if (length + 2 > sizeof(raw)) return;
  raw[0] = type;
  if (aLength) memcpy(raw + 1, a, aLength);
  if (bLength) memcpy(raw + 1 + aLength, b, bLength);
  const uint16_t crc = hgCrc16(raw, length);
  raw[length] = crc & 0xFF;
  raw[length + 1] = crc >> 8;
  const size_t encodedLength = hgCobsEncode(raw, length + 2, encoded);
  Serial.write(encoded, encodedLength);
  Serial.write((uint8_t)0);
}

static void sendLog(const char *text) {
  sendFrame(HG_SER_LOG, reinterpret_cast<const uint8_t *>(text), strlen(text));
}

static void sendHello() {
  HgSerHello hello = {};
  hello.protocol = HG_PROTOCOL_VERSION;
  hello.fw_major = FW_MAJOR;
  hello.fw_minor = FW_MINOR;
  esp_wifi_get_mac(WIFI_IF_STA, hello.mac);
  hello.max_payload = maxPayload;
  sendFrame(HG_SER_HELLO, reinterpret_cast<uint8_t *>(&hello), sizeof(hello));
}

static void sendStatus() {
  uint8_t buffer[sizeof(HgSerStatus) + HG_MAX_ROBOTS * sizeof(HgSerPeerStatus)];
  HgSerStatus status = {};
  status.uptime_ms = millis();
  status.flags = broadcastFlags();
  wifi_second_chan_t secondChannel;
  if (esp_wifi_get_channel(&status.channel, &secondChannel) != ESP_OK) status.channel = 0;  // actual radio channel
  status.count = peerCount;
  memcpy(buffer, &status, sizeof(status));
  const uint32_t now = millis();
  for (uint8_t i = 0; i < peerCount; ++i) {
    HgSerPeerStatus entry = {};
    entry.robot_id = peers[i].robotId;
    entry.last_rx_age_ms = peers[i].heard ? now - peers[i].lastRxMs : 0xFFFFFFFF;
    entry.tx_ok = peers[i].txOk;
    entry.tx_fail = peers[i].txFail;
    memcpy(buffer + sizeof(status) + i * sizeof(entry), &entry, sizeof(entry));
  }
  sendFrame(HG_SER_STATUS, buffer, sizeof(status) + peerCount * sizeof(HgSerPeerStatus));
}

// ---------------------------------------------------------------- ESP-NOW
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if (length <= 0 || length > (int)sizeof(rxScratch.data)) return;
  memcpy(rxScratch.mac, info->src_addr, 6);
  rxScratch.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  rxScratch.length = length;
  memcpy(rxScratch.data, data, length);
  xQueueSend(rxQueue, &rxScratch, 0);  // drop when the host link is backed up
}

static void onSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Peer *peer = info ? peerByMac(info->des_addr) : nullptr;
  if (!peer) return;
  if (status == ESP_NOW_SEND_SUCCESS) ++peer->txOk;
  else ++peer->txFail;
}

static void sendHeartbeats() {
  uint8_t message[sizeof(HgHeader) + sizeof(HgHeartbeat)];
  HgHeartbeat beat = {broadcastFlags(), millis()};
  ++heartbeatSeq;
  for (uint8_t i = 0; i < peerCount; ++i) {
    HgHeader header = {HG_MAGIC, HG_PROTOCOL_VERSION, HG_MSG_HEARTBEAT, peers[i].robotId, heartbeatSeq};
    memcpy(message, &header, sizeof(header));
    memcpy(message + sizeof(header), &beat, sizeof(beat));
    esp_now_send(peers[i].mac, message, sizeof(message));
  }
}

static void stopAll() {
  hostFlags = HG_HB_ESTOP;
  sendHeartbeats();
  uint8_t message[sizeof(HgHeader) + sizeof(HgStop)];
  HgStop stop = {0};
  for (uint8_t i = 0; i < peerCount; ++i) {
    HgHeader header = {HG_MAGIC, HG_PROTOCOL_VERSION, HG_MSG_STOP, peers[i].robotId, 0};
    memcpy(message, &header, sizeof(header));
    memcpy(message + sizeof(header), &stop, sizeof(stop));
    esp_now_send(peers[i].mac, message, sizeof(message));
  }
}

static bool applyConfig(const uint8_t *payload, size_t length) {
  if (length < sizeof(HgSerConfig)) return false;
  HgSerConfig config;
  memcpy(&config, payload, sizeof(config));
  if (config.channel < 1 || config.channel > 13 || config.count > HG_MAX_ROBOTS ||
      length != sizeof(config) + config.count * sizeof(HgSerPeer))
    return false;
  for (uint8_t i = 0; i < peerCount; ++i) esp_now_del_peer(peers[i].mac);
  peerCount = 0;
  configured = false;
  radioChannel = config.channel;
  if (esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_set_pmk(config.pmk) != ESP_OK) return false;
  for (uint8_t i = 0; i < config.count; ++i) {
    HgSerPeer entry;
    memcpy(&entry, payload + sizeof(config) + i * sizeof(entry), sizeof(entry));
    esp_now_peer_info_t info = {};
    memcpy(info.peer_addr, entry.mac, 6);
    memcpy(info.lmk, entry.lmk, HG_KEY_LEN);
    info.channel = radioChannel;
    info.ifidx = WIFI_IF_STA;
    info.encrypt = true;
    if (esp_now_add_peer(&info) != ESP_OK) return false;
    Peer &peer = peers[peerCount++];
    peer = {};
    peer.robotId = entry.robot_id;
    memcpy(peer.mac, entry.mac, 6);
  }
  configured = true;
  return true;
}

static bool isHostToRobot(uint8_t type) {
  return type == HG_MSG_CAPTURE || type == HG_MSG_MOVE || type == HG_MSG_STOP || type == HG_MSG_RESEND;
}

static void forwardToRobots(const uint8_t *payload, size_t length) {
  if (length < 1 + sizeof(HgHeader) || length - 1 > maxPayload) return;
  const uint8_t robotId = payload[0];
  const uint8_t *message = payload + 1;
  HgHeader header;
  memcpy(&header, message, sizeof(header));
  if (header.magic != HG_MAGIC || header.version != HG_PROTOCOL_VERSION || !isHostToRobot(header.type)) {
    sendLog("dropped malformed host message");
    return;
  }
  // Defense in depth: never relay a move while disarmed or stopped.
  if (header.type == HG_MSG_MOVE && !(broadcastFlags() & HG_HB_ARMED)) {
    sendLog("dropped move while disarmed");
    return;
  }
  for (uint8_t i = 0; i < peerCount; ++i) {
    if (robotId != 0xFF && peers[i].robotId != robotId) continue;
    if (esp_now_send(peers[i].mac, message, length - 1) != ESP_OK) ++peers[i].txFail;
  }
}

// ---------------------------------------------------------------- serial in
static void handleHostFrame(const uint8_t *frame, size_t length) {
  if (length < 3) return;
  const uint16_t crc = frame[length - 2] | (frame[length - 1] << 8);
  if (hgCrc16(frame, length - 2) != crc) return;
  lastHostMs = millis();
  hostSeen = true;
  const uint8_t type = frame[0];
  const uint8_t *payload = frame + 1;
  const size_t payloadLength = length - 3;
  switch (type) {
    case HG_SER_HELLO_REQ:
      sendHello();
      break;
    case HG_SER_CONFIG: {
      HgSerConfigAck ack = {applyConfig(payload, payloadLength), peerCount};
      sendFrame(HG_SER_CONFIG_ACK, reinterpret_cast<uint8_t *>(&ack), sizeof(ack));
      break;
    }
    case HG_SER_KEEPALIVE:
      if (payloadLength == sizeof(HgSerKeepalive)) {
        // Leaving e-stop requires an explicit keepalive without the e-stop bit.
        const uint8_t flags = payload[0] & (HG_HB_ARMED | HG_HB_ESTOP);
        const bool changed = flags != hostFlags;
        hostFlags = flags;
        if (changed && configured) sendHeartbeats();  // robots learn of arm/disarm at once
      }
      break;
    case HG_SER_STOP_ALL:
      stopAll();
      break;
    case HG_SER_SEND:
      forwardToRobots(payload, payloadLength);
      break;
  }
}

static void pollSerial() {
  static uint8_t decoded[SERIAL_MAX_ENCODED];
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) return;
    if (value == 0) {
      if (!serialInOverflow && serialInLength) {
        const size_t length = hgCobsDecode(serialIn, serialInLength, decoded);
        if (length) handleHostFrame(decoded, length);
      }
      serialInLength = 0;
      serialInOverflow = false;
    } else if (serialInLength < sizeof(serialIn)) {
      serialIn[serialInLength++] = value;
    } else {
      serialInOverflow = true;
    }
  }
}

static void updateLed() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < 250) return;
  lastMs = millis();
  const uint8_t flags = broadcastFlags();
  if (!hostOk()) rgbLedWrite(PIN_RGB, 24, 0, 0);            // red: no host
  else if (flags & HG_HB_ESTOP) rgbLedWrite(PIN_RGB, 24, 8, 0);  // amber: stopped
  else if (flags & HG_HB_ARMED) rgbLedWrite(PIN_RGB, 0, 24, 0);  // green: armed
  else rgbLedWrite(PIN_RGB, 0, 0, 24);                        // blue: connected
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(16384);
  Serial.begin(921600);
  rxQueue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(RxItem));

  // ESP-NOW only: never join or scan for a stored Wi-Fi network (a scan hops channels).
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, true);  // erase any stored access point
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(radioChannel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    sendLog("esp_now_init failed");
    return;
  }
  uint32_t version = 1;
  if (esp_now_get_version(&version) == ESP_OK && version >= 2) maxPayload = HG_ESPNOW_MAX_PAYLOAD;
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  delay(200);
  sendHello();
}

void loop() {
  pollSerial();

  static RxItem item;
  while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
    Peer *peer = peerByMac(item.mac);
    if (!peer) continue;  // only configured, encrypted peers are relayed
    peer->lastRxMs = millis();
    peer->heard = true;
    HgSerRecvHeader header = {peer->robotId, item.rssi};
    sendFrame(HG_SER_RECV, reinterpret_cast<uint8_t *>(&header), sizeof(header), item.data, item.length);
  }

  static uint32_t lastBeatMs = 0;
  if (configured && millis() - lastBeatMs >= HG_HEARTBEAT_PERIOD_MS) {
    lastBeatMs = millis();
    sendHeartbeats();
  }

  static uint32_t lastStatusMs = 0;
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    if (!configured) sendHello();  // lets a late-attaching host discover us
    sendStatus();
  }
  updateLed();
  delay(1);
}
