// Haggatrons wire protocol shared by the master and robot sketches.
//
// Two layers:
//   1. Robot messages: ESP-NOW payloads between the master and each robot.
//      Every payload starts with HgHeader.
//   2. Serial frames: USB link between the Mac and the master. Each frame is
//      [type u8][payload][crc16 u16 LE], COBS-encoded and terminated by 0x00.
//
// All multi-byte fields are little-endian and structs are packed. The Python
// mirror lives in haggatrons/protocol.py; tests/test_protocol.py checks that
// the constants below match it.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define HG_PROTOCOL_VERSION 1
#define HG_MAGIC 0x48  // 'H'
#define HG_KEY_LEN 16
#define HG_MAX_ROBOTS 4
#define HG_ESPNOW_MAX_PAYLOAD 1470  // ESP-NOW v2 limit
#define HG_CHUNK_DATA 1400

// Firmware hard limits. The host may ask for less, never more.
#define HG_MAX_MOVE_MS 2000
#define HG_MAX_JOG_MS_WITHOUT_RANGE 500
#define HG_MAX_TURN_DDEG 1800        // tenths of a degree
#define HG_LINK_TIMEOUT_MS 400       // robot stops without a fresh heartbeat
#define HG_HOST_TIMEOUT_MS 500       // master disarms without host keepalive
#define HG_HEARTBEAT_PERIOD_MS 100
#define HG_TELEMETRY_PERIOD_MS 200

// ---------------------------------------------------------------- robot messages
enum HgMsgType : uint8_t {
  HG_MSG_HEARTBEAT = 0x01,    // master -> robot
  HG_MSG_CAPTURE = 0x02,      // host -> robot
  HG_MSG_MOVE = 0x03,         // host -> robot
  HG_MSG_STOP = 0x04,         // host -> robot
  HG_MSG_RESEND = 0x05,       // host -> robot
  HG_MSG_TELEMETRY = 0x10,    // robot -> host
  HG_MSG_FRAME_META = 0x11,   // robot -> host
  HG_MSG_FRAME_CHUNK = 0x12,  // robot -> host
  HG_MSG_MOVE_RESULT = 0x13,  // robot -> host
  HG_MSG_REJECT = 0x14,       // robot -> host
};

enum HgHeartbeatFlag : uint8_t {
  HG_HB_ARMED = 0x01,
  HG_HB_ESTOP = 0x02,
  HG_HB_HOST_OK = 0x04,
};

enum HgMoveKind : uint8_t {
  HG_MOVE_DRIVE = 1,    // raw wheel speeds for a bounded time (manual jog)
  HG_MOVE_TURN = 2,     // turn in place by an IMU-measured angle
  HG_MOVE_FORWARD = 3,  // drive straight while the range sensor stays clear
};

enum HgOutcome : uint8_t {
  HG_OUT_COMPLETED = 0,
  HG_OUT_ESTOP = 1,
  HG_OUT_LINK_LOST = 2,
  HG_OUT_OBSTACLE = 3,
  HG_OUT_HOST_STOP = 4,
  HG_OUT_TURN_TIMEOUT = 5,
  HG_OUT_DISARMED = 6,
  HG_OUT_RANGE_LOST = 7,
};

enum HgReject : uint8_t {
  HG_REJ_NOT_ARMED = 1,
  HG_REJ_ESTOP = 2,
  HG_REJ_BUSY = 3,
  HG_REJ_BAD_PARAMS = 4,
  HG_REJ_IMU_UNAVAILABLE = 5,
  HG_REJ_RANGE_UNAVAILABLE = 6,
  HG_REJ_OBSTACLE = 7,
  HG_REJ_CAMERA_FAILED = 8,
  HG_REJ_FRAME_GONE = 9,
  HG_REJ_LINK_DOWN = 10,
};

enum HgTelemetryFlag : uint16_t {
  HG_TEL_ARMED = 0x0001,
  HG_TEL_ESTOP = 0x0002,
  HG_TEL_LINK_OK = 0x0004,
  HG_TEL_IMU_OK = 0x0008,
  HG_TEL_RANGE_OK = 0x0010,
  HG_TEL_MOVING = 0x0020,
  HG_TEL_CAMERA_OK = 0x0040,
};

#pragma pack(push, 1)

struct HgHeader {
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t robot_id;
  uint16_t seq;
};

struct HgHeartbeat {
  uint8_t flags;
  uint32_t master_ms;
};

struct HgCapture {
  uint32_t req_id;
  uint8_t framesize;  // 0=320x240 1=640x480 2=800x600 3=1024x768
  uint8_t quality;    // esp32-camera JPEG quality, 8..40 (lower is better)
};

struct HgMove {
  uint32_t req_id;
  uint8_t kind;
  uint8_t reserved;
  int16_t left;         // permille of full speed, -1000..1000
  int16_t right;
  uint16_t duration_ms;  // drive/forward duration or turn timeout
  int16_t turn_ddeg;     // turn only: positive = counter-clockwise (left)
  uint16_t min_clear_mm; // stop if the range sensor reads closer than this
};

struct HgStop {
  uint32_t req_id;  // 0 stops whatever is running
};

struct HgResend {
  uint32_t req_id;
  uint8_t count;
  // followed by `count` uint16_t chunk indices
};

struct HgTelemetry {
  uint32_t uptime_ms;
  uint16_t flags;
  uint16_t heartbeat_age_ms;
  float accel_g[3];
  float gyro_dps[3];
  uint16_t range_mm;
  uint32_t active_req;
  uint8_t last_outcome;
  uint8_t fw_major;
  uint8_t fw_minor;
};

struct HgFrameMeta {
  uint32_t req_id;
  uint32_t jpeg_len;
  uint16_t chunk_count;
  uint16_t chunk_size;
  uint16_t width;
  uint16_t height;
  uint32_t frame_ms;
  uint32_t imu_ms;
  uint8_t imu_valid;
  uint8_t range_valid;
  uint16_t range_mm;
  float accel_g[3];
  float gyro_dps[3];
};

struct HgFrameChunk {
  uint32_t req_id;
  uint16_t index;
  // followed by chunk data (the rest of the payload)
};

struct HgMoveResult {
  uint32_t req_id;
  uint8_t outcome;
  uint8_t kind;
  uint16_t elapsed_ms;
  float yaw_deg;         // integrated gyro yaw during the move
  uint16_t min_range_mm; // 0xFFFF if the range sensor never reported
  int16_t left;          // speeds actually applied
  int16_t right;
};

struct HgRejectMsg {
  uint32_t req_id;
  uint8_t ref_type;
  uint8_t code;
};

#pragma pack(pop)

// ---------------------------------------------------------------- serial frames
enum HgSerialType : uint8_t {
  // master -> host
  HG_SER_HELLO = 0x01,
  HG_SER_RECV = 0x02,
  HG_SER_STATUS = 0x03,
  HG_SER_LOG = 0x04,
  HG_SER_CONFIG_ACK = 0x05,
  // host -> master
  HG_SER_CONFIG = 0x81,
  HG_SER_SEND = 0x82,
  HG_SER_KEEPALIVE = 0x83,
  HG_SER_STOP_ALL = 0x84,
  HG_SER_HELLO_REQ = 0x85,
};

#pragma pack(push, 1)

struct HgSerHello {
  uint8_t protocol;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t mac[6];
  uint16_t max_payload;
};

struct HgSerPeer {
  uint8_t robot_id;
  uint8_t mac[6];
  uint8_t lmk[HG_KEY_LEN];
};

struct HgSerConfig {
  uint8_t channel;
  uint8_t pmk[HG_KEY_LEN];
  uint8_t count;
  // followed by `count` HgSerPeer
};

struct HgSerRecvHeader {
  uint8_t robot_id;
  int8_t rssi;
  // followed by the robot message
};

struct HgSerSendHeader {
  uint8_t robot_id;  // 0xFF = every configured robot
  // followed by the robot message
};

struct HgSerKeepalive {
  uint8_t flags;  // HG_HB_ARMED | HG_HB_ESTOP
};

struct HgSerPeerStatus {
  uint8_t robot_id;
  uint32_t last_rx_age_ms;  // 0xFFFFFFFF = never heard
  uint32_t tx_ok;
  uint32_t tx_fail;
};

struct HgSerStatus {
  uint32_t uptime_ms;
  uint8_t flags;  // HG_HB_* as currently broadcast
  uint8_t channel;
  uint8_t count;
  // followed by `count` HgSerPeerStatus
};

struct HgSerConfigAck {
  uint8_t ok;
  uint8_t count;
};

#pragma pack(pop)

// ---------------------------------------------------------------- USB provisioning
// Robot bench command: 'P' followed by HgProvision. The robot saves it to NVS,
// prints "PROV OK", and restarts.
enum HgMotorFlag : uint8_t {
  HG_MOTOR_INVERT_LEFT = 0x01,
  HG_MOTOR_INVERT_RIGHT = 0x02,
  HG_MOTOR_SWAP_SIDES = 0x04,
};

#pragma pack(push, 1)
struct HgProvision {
  uint8_t version;  // HG_PROTOCOL_VERSION
  uint8_t robot_id;
  uint8_t master_mac[6];
  uint8_t channel;
  uint8_t pmk[HG_KEY_LEN];
  uint8_t lmk[HG_KEY_LEN];
  uint8_t motor_flags;
  uint8_t yaw_axis;  // 0=x 1=y 2=z
  int8_t yaw_sign;   // +1 or -1 so that counter-clockwise is positive
  uint16_t max_speed;  // permille cap applied to every move
};
#pragma pack(pop)

// ---------------------------------------------------------------- helpers
static inline uint16_t hgCrc16(const uint8_t *data, size_t length) {
  // CRC-16/CCITT-FALSE
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; ++bit) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// COBS-encode `length` bytes. `out` needs length + length / 254 + 2 bytes.
// Returns the encoded length, excluding the 0x00 delimiter.
static inline size_t hgCobsEncode(const uint8_t *in, size_t length, uint8_t *out) {
  size_t read = 0, write = 1, code_at = 0;
  uint8_t code = 1;
  while (read < length) {
    if (in[read] == 0) {
      out[code_at] = code;
      code = 1;
      code_at = write++;
      ++read;
    } else {
      out[write++] = in[read++];
      if (++code == 0xFF) {
        out[code_at] = code;
        code = 1;
        code_at = write++;
      }
    }
  }
  out[code_at] = code;
  return write;
}

// Decode into a separate buffer of at least `length` bytes.
// Returns the decoded length, or 0 on malformed input.
static inline size_t hgCobsDecode(const uint8_t *in, size_t length, uint8_t *out) {
  size_t read = 0, write = 0;
  while (read < length) {
    uint8_t code = in[read];
    if (code == 0 || read + code > length) return 0;
    ++read;
    for (uint8_t i = 1; i < code; ++i) out[write++] = in[read++];
    if (code < 0xFF && read < length) out[write++] = 0;
  }
  return write;
}
