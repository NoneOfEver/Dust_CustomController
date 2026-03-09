#pragma once
#include "main.h"
#include <stddef.h>
#include <stdint.h>
#include <cstdint>
#include "angle_topic.hpp"
#include "input_topic.hpp"
#include "bsp_uart_port.h"

constexpr size_t kFrameHeaderLength = 5;
constexpr size_t kCmdIdLength = 2;
constexpr size_t kDataLength = 30;
constexpr size_t kFrameTailLength = 2;

constexpr size_t kDataFrameLength =
    kFrameHeaderLength + kCmdIdLength + kDataLength + kFrameTailLength;

constexpr uint16_t kCmdId = 0x0302;

struct __attribute__((packed)) FrameHeader
{
    uint8_t sof;            // 帧头标志 0xA5
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

struct __attribute__((packed)) CustomControllerData
{
    FrameHeader frame_header;
    uint16_t cmd_id;
    uint8_t data[kDataLength];
    uint16_t frame_tail; // 帧尾CRC16校验
};


class RefereeSender
{
public:
    RefereeSender() = default;
	static void InjectTxUart(BspUartHandle uart);
	static void StartThread();
    void ThreadLoop();

private:
	static RefereeSender& Instance();
	static void ThreadEntry(void* argument);

    CustomControllerData tx_{};
    std::uint8_t seq_{0};
    orb::AngleFrame last_angles_{};
	orb::InputState last_input_{};
    BspUartHandle tx_uart_{nullptr};

    void BuildPayload(uint8_t* data, const orb::AngleFrame& angles, const orb::InputState& input);
    void ConcatenateFrame(const uint8_t* data, uint16_t data_length);
    void Transmit();
};
