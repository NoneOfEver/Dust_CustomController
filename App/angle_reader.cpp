#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "angle_reader.h"

#include <cstring>

#include "angle_topic.hpp"
#include "bsp_adc_port.h"
#include "bsp_gpio_port.h"
#include "input_topic.hpp"

namespace
{
	// 消抖器
	struct Debouncer
	{
		uint8_t cnt = 0;
		bool stable = false;

		void update(bool raw)
		{
			constexpr uint8_t kMax = 5; // 5 * 10ms = 50ms
			if (raw)
			{
				if (cnt < kMax) ++cnt;
			}
			else
			{
				if (cnt > 0) --cnt;
			}
			stable = (cnt == kMax);
		}
	};
}

static osThreadId_t g_angle_thread_id = nullptr;

AngleReciever& AngleReciever::Instance()
{
	static AngleReciever instance;
	return instance;
}

void AngleReciever::ThreadEntry(void* argument)
{
	auto* self = static_cast<AngleReciever*>(argument);
	if (self != nullptr)
	{
		self->ThreadLoop();
	}
	osThreadExit();
}

void AngleReciever::StartThread()
{
	if (g_angle_thread_id != nullptr)
	{
		return;
	}

	static const osThreadAttr_t kAttr = {
		.name = "AngleReader",
		.stack_size = 512,
		.priority = (osPriority_t)osPriorityNormal,
	};

	g_angle_thread_id = osThreadNew(&AngleReciever::ThreadEntry, &Instance(), &kAttr);
}

void AngleReciever::OnRx(std::size_t channel, uint8_t* buffer, uint16_t length)
{
	if (channel >= kAngleChannels || buffer == nullptr || length == 0)
	{
		return;
	}

	auto& self = Instance();

	const uint16_t copy_len = (length > Angle::kRxBufferSize) ? static_cast<uint16_t>(Angle::kRxBufferSize) : length;
	std::memcpy(self.angles_[channel].rx.data(), buffer, copy_len);
	self.rx_len_[channel] = copy_len;

	if (g_angle_thread_id != nullptr)
	{
		(void)osThreadFlagsSet(g_angle_thread_id, (1UL << channel));
	}
}

Angle* AngleReciever::Angles()
{
	return angles_;
}

bool AngleReciever::TryParseAngleFromRx(const uint8_t* rx, std::size_t rx_len, float& out_angle)
{
	// 期望帧格式(按原屎山逻辑)：
	// rx[0] == 'A'(0x41) && rx[5] == ':'(0x3A)
	// 数字从 rx[6] 开始，到 '\r'(0x0D) 结束，中间允许一个 '.'
	constexpr std::size_t kHeaderAIndex = 0;
	constexpr std::size_t kHeaderColonIndex = 5;
	constexpr std::size_t kDataBegin = 6;
	constexpr std::size_t kDataEndExclusive = 16; // 原逻辑扫描到 rx[15]

	if (rx == nullptr || rx_len <= kHeaderColonIndex)
	{
		return false;
	}
	if (rx[kHeaderAIndex] != 0x41 || rx[kHeaderColonIndex] != 0x3A)
	{
		return false;
	}

	int integer_part = 0;
	int fractional_part = 0;
	float fractional_scale = 1.0f;
	bool has_dot = false;

	const std::size_t scan_end = (rx_len < kDataEndExclusive) ? rx_len : kDataEndExclusive;
	for (std::size_t j = kDataBegin; j < scan_end; ++j)
	{
		const uint8_t ch = rx[j];
		if (ch >= '0' && ch <= '9')
		{
			if (!has_dot)
			{
				integer_part = integer_part * 10 + static_cast<int>(ch - '0');
			}
			else
			{
				fractional_part = fractional_part * 10 + static_cast<int>(ch - '0');
				fractional_scale *= 0.1f;
			}
			continue;
		}
		if (ch == '.' && !has_dot)
		{
			has_dot = true;
			continue;
		}
		if (ch == 0x0D && has_dot)
		{
			out_angle = static_cast<float>(integer_part) + static_cast<float>(fractional_part) * fractional_scale;
			return true;
		}
	}

	return false;
}

void AngleReciever::ParseAndPublish(uint32_t updated_mask)
{
	auto try_parse_stream = [](const uint8_t* buf, std::size_t len, float& out_angle) -> bool {
		if (buf == nullptr || len == 0)
		{
			return false;
		}

		bool ok = false;
		float last = 0.0f;

		for (std::size_t start = 0; start + 6 <= len; ++start)
		{
			const std::size_t remain = len - start;
			float val = 0.0f;
			if (TryParseAngleFromRx(buf + start, remain, val))
			{
				ok = true;
				last = val;
			}
		}

		if (ok)
		{
			out_angle = last;
		}
		return ok;
	};

	auto* angles = Angles();

	for (std::size_t i = 0; i < kAngleChannels; ++i)
	{
		if ((updated_mask & (1UL << i)) == 0)
		{
			continue;
		}

		float angle = 0.0f;
		const uint16_t len = rx_len_[i];
		if (len >= kMinParseBytes && try_parse_stream(angles[i].rx.data(), len, angle))
		{
			angles[i].angle = angle;
		}
	}

	orb::AngleFrame frame{};
	for (std::size_t i = 0; i < kAngleChannels; ++i)
	{
		frame.angles[i] = angles[i].angle;
	}
	orb::angle_frame.publish(frame);
}

void AngleReciever::ThreadLoop()
{
	constexpr uint32_t kSampleMs = 10;
	const uint32_t sample_ticks = (uint32_t)((kSampleMs * osKernelGetTickFreq() + 999U) / 1000U);

	auto* b1 = bsp_gpio_get(BSP_GPIO_BUTTON1);
	auto* b2 = bsp_gpio_get(BSP_GPIO_BUTTON2);
	auto* b3 = bsp_gpio_get(BSP_GPIO_BUTTON3);
	auto* b4 = bsp_gpio_get(BSP_GPIO_BUTTON4);
	auto* jb = bsp_gpio_get(BSP_GPIO_JOYSTICK_BUTTON);

	Debouncer db1{};
	Debouncer db2{};
	Debouncer db3{};
	Debouncer db4{};
	Debouncer dbj{};

	constexpr uint32_t kAllFlagsMask = (1UL << 0) | (1UL << 1) | (1UL << 2) | (1UL << 3);
	for (;;)
	{
		const uint32_t flags = osThreadFlagsWait(kAllFlagsMask, osFlagsWaitAny, sample_ticks);
		if ((flags & 0x80000000UL) == 0)
		{
			ParseAndPublish(flags);
		}

		// 周期采样：按键消抖 + ADC 读取
		const auto pressed = [](BspGpioHandle h) -> bool {
			// active-low: pressed when reads low
			return (h != nullptr) && (!bsp_gpio_read(h));
		};
		db1.update(pressed(b1));
		db2.update(pressed(b2));
		db3.update(pressed(b3));
		db4.update(pressed(b4));
		dbj.update(pressed(jb));

		orb::InputState input{};
		input.buttons |= (db1.stable ? 1u : 0u) << 0;
		input.buttons |= (db2.stable ? 1u : 0u) << 1;
		input.buttons |= (db3.stable ? 1u : 0u) << 2;
		input.buttons |= (db4.stable ? 1u : 0u) << 3;
		input.buttons |= (dbj.stable ? 1u : 0u) << 4;

		const uint32_t kVrefMv = 3300;
		(uint8_t)bsp_adc_read_raw(BSP_ADC1_JOYSTICK_X, &input.adc_x_raw);
		(uint8_t)bsp_adc_read_raw(BSP_ADC1_JOYSTICK_Y, &input.adc_y_raw);
		(uint8_t)bsp_adc_read_raw(BSP_ADC1_JOYSTICK_Z, &input.adc_z_raw);
		uint32_t mv = 0;
		if (bsp_adc_read_millivolts(BSP_ADC1_JOYSTICK_X, kVrefMv, &mv)) input.adc_x_mv = static_cast<uint16_t>(mv);
		if (bsp_adc_read_millivolts(BSP_ADC1_JOYSTICK_Y, kVrefMv, &mv)) input.adc_y_mv = static_cast<uint16_t>(mv);
		if (bsp_adc_read_millivolts(BSP_ADC1_JOYSTICK_Z, kVrefMv, &mv)) input.adc_z_mv = static_cast<uint16_t>(mv);

		orb::input_state.publish(input);
	}
}

