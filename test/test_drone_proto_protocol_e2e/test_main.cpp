#include "../../../../ESP32 Radiomaster/include/DirectRcProtocol.h"
#include "../../../../BW21 CAM Drone/src/DroneProtoCameraProtocol.h"

#include "Device/DroneProtoDirectRcProtocol.hpp"
#include "Connect/DroneProtoCameraProtocol.hpp"

// The isolated Drone Prototype test environment does not build library
// sources. Include the real FC MSP implementation so BW21 frames traverse the
// same byte parser used by the camera UART on the flight controller.
#include "../../lib/Espfc/src/Utils/Crc.cpp"
#include "../../lib/Espfc/src/Connect/Msp.cpp"
#include "../../lib/Espfc/src/Connect/MspParser.cpp"

#include <cstdint>
#include <cstring>
#include <iostream>

namespace RadioTx = ::DirectRcProtocol;
namespace FcRc = Espfc::Device::DirectRcProtocol;
namespace CameraTx = ::DroneProtoCameraProtocol;
namespace CameraRx = Espfc::Connect::DroneProtoCameraProtocol;
using Espfc::Connect::MspMessage;
using Espfc::Connect::MspParser;
using Espfc::Connect::MspResponse;
using Espfc::Connect::MSP_STATE_IDLE;
using Espfc::Connect::MSP_TYPE_REPLY;
using Espfc::Connect::MSP_V2;

namespace {

constexpr uint32_t LinkId = 0x6D5A31C7UL;
int failures = 0;

#define CHECK(condition) do { \
  if(!(condition)) { \
    std::cerr << "FAIL line " << __LINE__ << ": " #condition "\n"; \
    failures++; \
  } \
} while(false)

void fillChannels(uint16_t *channels)
{
  for(size_t i = 0; i < RadioTx::ChannelCount; ++i)
  {
    channels[i] = static_cast<uint16_t>(1000 + i * 50);
  }
}

void testRadioMasterDirectPacketReachesFcDecoder()
{
  uint16_t channels[RadioTx::ChannelCount] = {};
  fillChannels(channels);
  RadioTx::Packet outgoing;
  CHECK(RadioTx::encode(outgoing, LinkId, 0xfffe, 123456, channels,
                        RadioTx::Mode::Direct));

  FcRc::Packet incoming;
  FcRc::Mode mode = FcRc::Mode::NONE;
  CHECK(FcRc::decode(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing),
                     LinkId, incoming, mode) == FcRc::DecodeResult::ACCEPTED);
  CHECK(mode == FcRc::Mode::DIRECT);
  CHECK(incoming.sequence == 0xfffe);
  CHECK(incoming.timeMs == 123456);
  for(size_t i = 0; i < RadioTx::ChannelCount; ++i)
  {
    CHECK(incoming.channels[i] == channels[i]);
  }
}

void testRadioMasterTrainerAndStopModesRemainCompatible()
{
  uint16_t channels[RadioTx::ChannelCount] = {};
  fillChannels(channels);
  RadioTx::Packet outgoing;
  FcRc::Packet incoming;
  FcRc::Mode mode = FcRc::Mode::NONE;

  CHECK(RadioTx::encode(outgoing, LinkId, 7, 200, channels,
                        RadioTx::Mode::TrainerSideband));
  CHECK(FcRc::decode(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing),
                     LinkId, incoming, mode) == FcRc::DecodeResult::ACCEPTED);
  CHECK(mode == FcRc::Mode::TRAINER_SIDEBAND);

  CHECK(RadioTx::encode(outgoing, LinkId, 8, 210, nullptr, RadioTx::Mode::None));
  CHECK(FcRc::decode(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing),
                     LinkId, incoming, mode) == FcRc::DecodeResult::ACCEPTED);
  CHECK(mode == FcRc::Mode::NONE);
  CHECK(incoming.channels[2] == RadioTx::ChannelMinUs);
  CHECK(incoming.channels[0] == RadioTx::ChannelMidUs);
}

void testFcRejectsWrongSourceAndTamperedRadioPackets()
{
  CHECK(FcRc::trustedSourceMac(FcRc::RADIO_BRIDGE_BASE_MAC));
  CHECK(FcRc::trustedSourceMac(FcRc::RADIO_BRIDGE_AP_MAC));
  const uint8_t stranger[6] = {0x68, 0xee, 0x8f, 0xdd, 0x1f, 0x9a};
  CHECK(!FcRc::trustedSourceMac(stranger));
  CHECK(!FcRc::trustedSourceMac(nullptr));

  uint16_t channels[RadioTx::ChannelCount] = {};
  fillChannels(channels);
  RadioTx::Packet outgoing;
  CHECK(RadioTx::encode(outgoing, LinkId, 10, 300, channels, RadioTx::Mode::Direct));
  FcRc::Packet incoming;
  FcRc::Mode mode = FcRc::Mode::NONE;

  outgoing.channels[0] ^= 1;
  CHECK(FcRc::decode(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing),
                     LinkId, incoming, mode) == FcRc::DecodeResult::BAD_CRC_OR_HEADER);

  CHECK(RadioTx::encode(outgoing, LinkId ^ 1, 11, 310, channels, RadioTx::Mode::Direct));
  CHECK(FcRc::decode(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing),
                     LinkId, incoming, mode) == FcRc::DecodeResult::BAD_LINK);

  CHECK(RadioTx::encode(outgoing, LinkId, 12, 320, channels, RadioTx::Mode::Direct));
  outgoing.flags = 0x80;
  outgoing.crc = RadioTx::crc16Ccitt(reinterpret_cast<const uint8_t *>(&outgoing),
                                     sizeof(outgoing) - sizeof(outgoing.crc));
  CHECK(FcRc::decode(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing),
                     LinkId, incoming, mode) == FcRc::DecodeResult::BAD_VALUE);
}

MspMessage parseMspFrame(const uint8_t *frame, size_t length)
{
  MspMessage message;
  MspParser parser;
  for(size_t i = 0; i < length; ++i) parser.parse(static_cast<char>(frame[i]), message);
  return message;
}

void testBw21QrFrameTraversesFcMspAndCameraDecoders()
{
  const CameraTx::QrObservation observation = {
    "DLOC1,FLOOR1,3.000,4.000,0.000,0.0,0.200",
    true, true, false, false,
    513, 487, 138, 19, -1234
  };
  uint8_t payload[CameraTx::QrMaxPayloadSize] = {};
  size_t payloadLength = 0;
  CHECK(CameraTx::buildQrPayload(observation, 0xffff, payload, sizeof(payload),
                                 payloadLength));

  uint8_t frame[CameraTx::QrMaxFrameSize] = {};
  size_t frameLength = 0;
  CHECK(CameraTx::buildMspV2Frame(CameraTx::Msp2CameraQr, payload, payloadLength,
                                  frame, sizeof(frame), frameLength));

  MspMessage parsed = parseMspFrame(frame, frameLength);
  CHECK(parsed.isReady());
  CHECK(parsed.isCmd());
  CHECK(parsed.version == MSP_V2);
  CHECK(parsed.cmd == CameraRx::MSP2_CAMERA_QR);

  CameraRx::QrMessage incoming;
  CHECK(CameraRx::decodeQrPayload(parsed.buffer, parsed.received, incoming) ==
        CameraRx::DecodeResult::ACCEPTED);
  CHECK(incoming.version == CameraRx::VERSION_V2);
  CHECK(incoming.sequence == 0xffff);
  CHECK(incoming.flags == (CameraTx::QrFlagGeometryValid |
                           CameraTx::QrFlagFullResolution));
  CHECK(incoming.centerXPermille == 513);
  CHECK(incoming.centerYPermille == 487);
  CHECK(incoming.sidePermille == 138);
  CHECK(incoming.areaPermille == 19);
  CHECK(incoming.rotationCdeg == -1234);
  CHECK(std::strcmp(incoming.payload, observation.payload) == 0);
}

void testCameraChecksumMalformedAndUnsupportedFramesAreRejected()
{
  const CameraTx::QrObservation observation = {
    "DLOC1,A1,0,0,0,0,0.2", true, false, true, true,
    500, 500, 100, 10, 9000
  };
  uint8_t payload[CameraTx::QrMaxPayloadSize] = {};
  size_t payloadLength = 0;
  CHECK(CameraTx::buildQrPayload(observation, 44, payload, sizeof(payload), payloadLength));

  uint8_t frame[CameraTx::QrMaxFrameSize] = {};
  size_t frameLength = 0;
  CHECK(CameraTx::buildMspV2Frame(CameraTx::Msp2CameraQr, payload, payloadLength,
                                  frame, sizeof(frame), frameLength));
  frame[8 + CameraTx::QrHeaderSize] ^= 1;
  const MspMessage badChecksum = parseMspFrame(frame, frameLength);
  CHECK(!badChecksum.isReady());
  CHECK(badChecksum.state == MSP_STATE_IDLE);

  CameraRx::QrMessage decoded;
  payload[4]++;
  CHECK(CameraRx::decodeQrPayload(payload, payloadLength, decoded) ==
        CameraRx::DecodeResult::MALFORMED);
  payload[4]--;
  payload[0] = 99;
  CHECK(CameraRx::decodeQrPayload(payload, payloadLength, decoded) ==
        CameraRx::DecodeResult::UNSUPPORTED);
}

void testFcMspParserAcceptsHighBitChecksumBytes()
{
  // 1-byte MSPv1 payload: 1 ^ 200 ^ 55 = 254. This specifically guards the
  // signed-char checksum bug found by the BW21 end-to-end frame.
  const uint8_t frame[] = {'$', 'M', '<', 1, 200, 55, 254};
  const MspMessage parsed = parseMspFrame(frame, sizeof(frame));
  CHECK(parsed.isReady());
  CHECK(parsed.cmd == 200);
  CHECK(parsed.received == 1);
  CHECK(parsed.buffer[0] == 55);
}

void testFcAckPayloadMatchesBw21SequenceChecks()
{
  MspResponse response;
  response.version = MSP_V2;
  response.cmd = CameraRx::MSP2_CAMERA_QR;
  response.result = 1;
  response.writeU8(CameraRx::VERSION_V2);
  response.writeU8(CameraRx::ACK_ACCEPTED);
  response.writeU16(0xffff);

  uint8_t frame[32] = {};
  const size_t frameLength = response.serialize(frame, sizeof(frame));
  CHECK(frameLength != 0);
  MspMessage parsed = parseMspFrame(frame, frameLength);
  CHECK(parsed.isReady());
  CHECK(parsed.dir == MSP_TYPE_REPLY);

  uint8_t status = 0xff;
  CHECK(CameraTx::parseQrAck(parsed.buffer, parsed.received,
                             CameraTx::ProtocolVersion, 0xffff, status));
  CHECK(status == CameraRx::ACK_ACCEPTED);
  CHECK(!CameraTx::parseQrAck(parsed.buffer, parsed.received,
                              CameraTx::ProtocolVersion, 0, status));
}

} // namespace

int main()
{
  testRadioMasterDirectPacketReachesFcDecoder();
  testRadioMasterTrainerAndStopModesRemainCompatible();
  testFcRejectsWrongSourceAndTamperedRadioPackets();
  testBw21QrFrameTraversesFcMspAndCameraDecoders();
  testCameraChecksumMalformedAndUnsupportedFramesAreRejected();
  testFcMspParserAcceptsHighBitChecksumBytes();
  testFcAckPayloadMatchesBw21SequenceChecks();

  if(failures != 0)
  {
    std::cerr << failures << " DroneProto protocol integration test(s) failed\n";
    return 1;
  }
  std::cout << "All DroneProto end-to-end protocol tests passed\n";
  return 0;
}
