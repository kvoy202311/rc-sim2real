#pragma once

#include <array>
#include <cstdint>
#include <cstring>

// Binary frame layouts:
//
// cmd frame, PC -> MCU (155 bytes total):
//   [header0:1] [header1:1] [version:1] [type:1] [payload_len:2 LE]
//   [ctrl_mode:1]
//   [position:12xfloat32] [velocity:12xfloat32] [torque:12xfloat32]
//   [crc16:2 LE] [tail0:1] [tail1:1]
//
// state frame, MCU -> PC (154 bytes total):
//   [header0:1] [header1:1] [version:1] [type:1] [payload_len:2 LE]
//   [position:12xfloat32] [velocity:12xfloat32] [torque:12xfloat32]
//   [crc16:2 LE] [tail0:1] [tail1:1]

namespace kvoy::protocol {

static constexpr uint8_t  HEADER0               = 0xAA;
static constexpr uint8_t  HEADER1               = 0x55;
static constexpr uint8_t  TAIL0                 = 0x0D;
static constexpr uint8_t  TAIL1                 = 0x0A;
static constexpr uint8_t  VERSION               = 0x01;
static constexpr uint8_t  TYPE_CMD              = 0x01;
static constexpr uint8_t  TYPE_STATE            = 0x02;
static constexpr uint8_t  CTRL_MODE_DAMPING     = 0x00;
static constexpr uint8_t  CTRL_MODE_MOTION      = 0x01;

static constexpr uint16_t NUM_JOINTS            = 12;
static constexpr uint16_t NUM_CHANNELS          = 3;
static constexpr uint16_t HEADER_LEN            = 6;
static constexpr uint16_t CRC_LEN               = 2;
static constexpr uint16_t TAIL_LEN              = 2;
static constexpr uint16_t CTRL_MODE_LEN         = 1;
static constexpr uint16_t FLOAT_LEN             = sizeof(float);
static constexpr uint16_t JOINT_ARRAY_LEN       = NUM_JOINTS * FLOAT_LEN;
static constexpr uint16_t STATE_PAYLOAD_LEN     = NUM_CHANNELS * JOINT_ARRAY_LEN;
static constexpr uint16_t CMD_PAYLOAD_LEN       = CTRL_MODE_LEN + STATE_PAYLOAD_LEN;

static constexpr uint16_t STATE_PAYLOAD_OFFSET  = HEADER_LEN;
static constexpr uint16_t CMD_CTRL_MODE_OFFSET  = HEADER_LEN;
static constexpr uint16_t CMD_ARRAYS_OFFSET     = CMD_CTRL_MODE_OFFSET + CTRL_MODE_LEN;

static constexpr uint16_t STATE_FRAME_LEN       = HEADER_LEN + STATE_PAYLOAD_LEN + CRC_LEN + TAIL_LEN;
static constexpr uint16_t CMD_FRAME_LEN         = HEADER_LEN + CMD_PAYLOAD_LEN + CRC_LEN + TAIL_LEN;
static constexpr uint16_t STATE_CRC_INPUT_LEN   = HEADER_LEN + STATE_PAYLOAD_LEN;
static constexpr uint16_t CMD_CRC_INPUT_LEN     = HEADER_LEN + CMD_PAYLOAD_LEN;
static constexpr uint16_t STATE_CRC_OFFSET      = STATE_PAYLOAD_OFFSET + STATE_PAYLOAD_LEN;
static constexpr uint16_t CMD_CRC_OFFSET        = HEADER_LEN + CMD_PAYLOAD_LEN;
static constexpr uint16_t STATE_TAIL_OFFSET     = STATE_CRC_OFFSET + CRC_LEN;
static constexpr uint16_t CMD_TAIL_OFFSET       = CMD_CRC_OFFSET + CRC_LEN;

struct JointTriplet {
    std::array<float, NUM_JOINTS> position{};
    std::array<float, NUM_JOINTS> velocity{};
    std::array<float, NUM_JOINTS> torque{};
};

struct CommandFrame {
    uint8_t     version{VERSION};
    uint8_t     type{TYPE_CMD};
    uint8_t     ctrl_mode{CTRL_MODE_MOTION};
    JointTriplet joints{};
};

struct StateFrame {
    uint8_t     version{VERSION};
    uint8_t     type{TYPE_STATE};
    JointTriplet joints{};
};

using RawCmdFrame = std::array<uint8_t, CMD_FRAME_LEN>;
using RawStateFrame = std::array<uint8_t, STATE_FRAME_LEN>;

inline uint16_t crc16(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

inline void write_u16_le(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline uint16_t read_u16_le(const uint8_t* src)
{
    return static_cast<uint16_t>(src[0]) |
           static_cast<uint16_t>(src[1]) << 8;
}

inline void write_f32_le(uint8_t* dst, float value)
{
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "Unexpected float size");
    std::memcpy(&bits, &value, sizeof(bits));
    dst[0] = static_cast<uint8_t>(bits & 0xFF);
    dst[1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((bits >> 24) & 0xFF);
}

inline float read_f32_le(const uint8_t* src)
{
    const uint32_t bits =
        static_cast<uint32_t>(src[0]) |
        (static_cast<uint32_t>(src[1]) << 8) |
        (static_cast<uint32_t>(src[2]) << 16) |
        (static_cast<uint32_t>(src[3]) << 24);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline void write_joint_triplet(const JointTriplet& joints, uint8_t* dst)
{
    uint16_t offset = 0;
    auto write_array = [&](const std::array<float, NUM_JOINTS>& values) {
        for (float value : values) {
            write_f32_le(dst + offset, value);
            offset += FLOAT_LEN;
        }
    };

    write_array(joints.position);
    write_array(joints.velocity);
    write_array(joints.torque);
}

inline void read_joint_triplet(const uint8_t* src, JointTriplet& joints)
{
    uint16_t offset = 0;
    auto read_array = [&](std::array<float, NUM_JOINTS>& values) {
        for (float& value : values) {
            value = read_f32_le(src + offset);
            offset += FLOAT_LEN;
        }
    };

    read_array(joints.position);
    read_array(joints.velocity);
    read_array(joints.torque);
}

inline void encode_cmd(
    RawCmdFrame& raw,
    uint8_t ctrl_mode,
    const float* pos,
    const float* vel,
    const float* tor)
{
    CommandFrame frame;
    frame.ctrl_mode = ctrl_mode;
    std::memcpy(frame.joints.position.data(), pos, JOINT_ARRAY_LEN);
    std::memcpy(frame.joints.velocity.data(), vel, JOINT_ARRAY_LEN);
    std::memcpy(frame.joints.torque.data(), tor, JOINT_ARRAY_LEN);

    raw.fill(0);
    raw[0] = HEADER0;
    raw[1] = HEADER1;
    raw[2] = frame.version;
    raw[3] = frame.type;
    write_u16_le(raw.data() + 4, CMD_PAYLOAD_LEN);
    raw[CMD_CTRL_MODE_OFFSET] = frame.ctrl_mode;
    write_joint_triplet(frame.joints, raw.data() + CMD_ARRAYS_OFFSET);
    write_u16_le(raw.data() + CMD_CRC_OFFSET, crc16(raw.data(), CMD_CRC_INPUT_LEN));
    raw[CMD_TAIL_OFFSET] = TAIL0;
    raw[CMD_TAIL_OFFSET + 1] = TAIL1;
}

inline bool decode_state(const uint8_t* raw, StateFrame& frame)
{
    if (raw[0] != HEADER0 || raw[1] != HEADER1) return false;
    if (raw[2] != VERSION) return false;
    if (raw[3] != TYPE_STATE) return false;
    if (read_u16_le(raw + 4) != STATE_PAYLOAD_LEN) return false;
    if (read_u16_le(raw + STATE_CRC_OFFSET) != crc16(raw, STATE_CRC_INPUT_LEN)) return false;
    if (raw[STATE_TAIL_OFFSET] != TAIL0 || raw[STATE_TAIL_OFFSET + 1] != TAIL1) return false;

    frame.version = raw[2];
    frame.type = raw[3];
    read_joint_triplet(raw + STATE_PAYLOAD_OFFSET, frame.joints);
    return true;
}

} // namespace kvoy::protocol
