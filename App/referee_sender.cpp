#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "crc.h"
#include "referee_sender.h"
#include <cstring>

#include "bsp_uart_port.h"

#include "angle_topic.hpp"
#include "input_topic.hpp"
#include "topic_pubsub.hpp"

static osThreadId_t g_referee_sender_thread_id = nullptr;

RefereeSender& RefereeSender::Instance()
{
	static RefereeSender instance;
	return instance;
}

void RefereeSender::InjectTxUart(BspUartHandle uart)
{
	Instance().tx_uart_ = uart;
}

void RefereeSender::ThreadEntry(void* argument)
{
	auto* self = static_cast<RefereeSender*>(argument);
	if (self != nullptr)
	{
		self->ThreadLoop();
	}
	osThreadExit();
}

void RefereeSender::StartThread()
{
	if (g_referee_sender_thread_id != nullptr)
	{
		return;
	}

	static const osThreadAttr_t kAttr = {
		.name = "RefereeSender",
		.stack_size = 512,
		.priority = (osPriority_t)osPriorityNormal,
	};

	g_referee_sender_thread_id = osThreadNew(&RefereeSender::ThreadEntry, &Instance(), &kAttr);
}

void RefereeSender::BuildPayload(uint8_t* data, const orb::AngleFrame& angles, const orb::InputState& input)
{
	constexpr std::size_t kAnglesToSend = orb::AngleFrame::kChannels;
	constexpr std::size_t kBytesPerAngle = sizeof(float);
	constexpr std::size_t kAngleBytes = kAnglesToSend * kBytesPerAngle; // 16
	constexpr std::size_t kInputBytes = 1 + 6 * sizeof(std::uint16_t);  // buttons + (raw x3 + mv x3)
	static_assert(kAngleBytes + kInputBytes <= kDataLength, "payload too small for angles + input");

	// payload[0..15] 依次放 4 路角度 float (小端)
	// 与原先 union byte 的发送行为一致，但实现更明确。
	for (std::size_t idx = 0; idx < kAngleBytes && idx + kBytesPerAngle <= kDataLength; idx += kBytesPerAngle)
	{
		const std::size_t angle_index = idx / kBytesPerAngle;
		const float angle = angles.angles[angle_index];
		std::memcpy(&data[idx], &angle, kBytesPerAngle);
	}

	// 剩余字节：buttons + ADC(3 raw + 3 mv)
	std::size_t off = kAngleBytes;
	data[off++] = input.buttons;

	auto put_u16_le = [&](std::uint16_t v) {
		if (off + 2 > kDataLength)
		{
			return;
		}
		data[off++] = static_cast<std::uint8_t>(v & 0xFFu);
		data[off++] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
	};

	put_u16_le(input.adc_x_raw);
	put_u16_le(input.adc_y_raw);
	put_u16_le(input.adc_z_raw);
	put_u16_le(input.adc_x_mv);
	put_u16_le(input.adc_y_mv);
	put_u16_le(input.adc_z_mv);
}

void RefereeSender::ConcatenateFrame(const uint8_t* data, uint16_t data_length)
{
	auto& tx = tx_;
	auto& seq = seq_;

	tx.frame_header.sof = 0xA5;
	tx.frame_header.data_length = data_length;
	tx.frame_header.seq = seq++;
	append_crc8_check_sum((uint8_t*)(&tx.frame_header), kFrameHeaderLength);

	tx.cmd_id = kCmdId;
	std::memcpy(tx.data, data, data_length);
	append_crc16_check_sum((uint8_t*)(&tx), kDataFrameLength);
}

void RefereeSender::Transmit()
{
	auto& tx = tx_;
	auto h = tx_uart_ ? tx_uart_ : bsp_uart_get(BSP_UART6);
	(void)bsp_uart_send(h, reinterpret_cast<uint8_t*>(&tx), static_cast<uint16_t>(sizeof(tx)));
}

void RefereeSender::ThreadLoop()
{
	Sub<orb::AngleFrame> sub(orb::angle_frame);
	Sub<orb::InputState> input_sub(orb::input_state);

	uint32_t wait_time = osKernelGetTickCount();
	const uint32_t period_ticks = (uint32_t)((500ULL * osKernelGetTickFreq() + 999ULL) / 1000ULL);
	for (;;)
	{
		orb::AngleFrame latest{};
		if (sub.copy(latest))
		{
			last_angles_ = latest;
		}

		orb::InputState latest_input{};
		if (input_sub.copy(latest_input))
		{
			last_input_ = latest_input;
		}

		uint8_t payload[kDataLength] = {0};
		BuildPayload(payload, last_angles_, last_input_);
		ConcatenateFrame(payload, kDataLength);
		Transmit();
		wait_time += period_ticks;
		osDelayUntil(wait_time);
	}
}