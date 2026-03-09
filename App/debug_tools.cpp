#include "debug_tools.h"

#include "cmsis_os2.h"

#include "bsp_uart_port.h"

#include "angle_topic.hpp"
#include "input_topic.hpp"
#include "topic_pubsub.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{
	constexpr uint32_t kPrintMs = 200;
}

static osThreadId_t g_angle_debug_thread_id = nullptr;

Vofa& Vofa::Instance()
{
	static Vofa instance;
	return instance;
}

void Vofa::InjectTxUart(BspUartHandle uart)
{
	Instance().tx_uart_ = uart;
}

void Vofa::ThreadEntry(void* argument)
{
	auto* self = static_cast<Vofa*>(argument);
	if (self != nullptr)
	{
		self->ThreadLoop();
	}
	osThreadExit();
}

void Vofa::StartThread()
{
	if (g_angle_debug_thread_id != nullptr)
	{
		return;
	}
	
	static const osThreadAttr_t kAttr = {
		.name = "AngleDebug",
		.stack_size = 512,
		.priority = (osPriority_t)osPriorityLow,
	};
	
	g_angle_debug_thread_id = osThreadNew(&Vofa::ThreadEntry, &Instance(), &kAttr);
}

void Vofa::SendFloat(float data)
{
	auto h = tx_uart_ ? tx_uart_ : bsp_uart_get(BSP_UART5);
	if (h == nullptr)
	{
		return;
	}

	uint8_t buf[4]{};
	std::memcpy(buf, &data, 4);
	(void)bsp_uart_send(h, buf, 4);
}

void Vofa::SendTail()
{
	auto h = tx_uart_ ? tx_uart_ : bsp_uart_get(BSP_UART5);
	if (h == nullptr)
	{
		return;
	}

	uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
	(void)bsp_uart_send(h, tail, 4);
}

void Vofa::ThreadLoop()
{
	uint32_t next = osKernelGetTickCount();
	const uint32_t print_ticks = (uint32_t)((kPrintMs * osKernelGetTickFreq() + 999U) / 1000U);
	uint32_t print_deadline = next;

	Sub<orb::AngleFrame> sub(orb::angle_frame);
	Sub<orb::InputState> input_sub(orb::input_state);

	orb::AngleFrame last{};
	orb::InputState input_last{};
	bool has_input = false;

	for (;;)
	{
		orb::AngleFrame cur{};
		if (sub.copy(cur))
		{
			last = cur;
		}

		orb::InputState input_cur{};
		if (input_sub.copy(input_cur))
		{
			input_last = input_cur;
			has_input = true;
		}

		const uint32_t now = osKernelGetTickCount();
		if ((int32_t)(now - print_deadline) >= 0)
		{
			print_deadline = now + print_ticks;

			// VOFA 协议：连续发送 float，最后补一个 Tail
			// 顺序：A0 A1 A2 A3 B1 B2 B3 B4 JB ADC_X_RAW ADC_Y_RAW ADC_Z_RAW
			SendFloat(last.angles[0]);
			SendFloat(last.angles[1]);
			SendFloat(last.angles[2]);
			SendFloat(last.angles[3]);

			const uint32_t buttons = has_input ? static_cast<uint32_t>(input_last.buttons) : 0u;
			uint8_t button1 = (buttons >> 0) & 1u;
			uint8_t button2 = (buttons >> 1) & 1u;
			uint8_t button3 = (buttons >> 2) & 1u;
			uint8_t button4 = (buttons >> 3) & 1u;
			uint8_t joystick_button = (buttons >> 4) & 1u;

			SendFloat(static_cast<float>(button1));
			SendFloat(static_cast<float>(button2));
			SendFloat(static_cast<float>(button3));
			SendFloat(static_cast<float>(button4));
			SendFloat(static_cast<float>(joystick_button));
			SendFloat(has_input ? static_cast<float>(input_last.adc_x_raw) : 0.0f);
			SendFloat(has_input ? static_cast<float>(input_last.adc_y_raw) : 0.0f);
			SendFloat(has_input ? static_cast<float>(input_last.adc_z_raw) : 0.0f);
			SendTail();
		}

		next += print_ticks;
		osDelayUntil(next);
	}
}
