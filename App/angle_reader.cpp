#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "angle_reader.h"

#include <cstring>

#include "angle_topic.hpp"
#include "bsp_uart_port.h"
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
			constexpr uint8_t kMax = 3; // 3 * 10ms = 30ms
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

namespace
{
	static BspUartHandle tx_uart_for_channel(std::size_t channel)
	{
		switch (channel)
		{
			case 0: return bsp_uart_get(BSP_UART3);
			case 1: return bsp_uart_get(BSP_UART4);
			case 2: return bsp_uart_get(BSP_UART6);
			default: return nullptr;
		}
	}

	static float wrap_deg_360(float deg)
	{
		// 归一化到 [0, 360)
		while (deg < 0.0f) deg += 360.0f;
		while (deg >= 360.0f) deg -= 360.0f;
		return deg;
	}

	static float unwrap_delta_deg(float cur_deg_0_360, float last_deg_0_360)
	{
		// 计算最短角差，范围约为 [-180, +180]
		float d = cur_deg_0_360 - last_deg_0_360;
		if (d > 180.0f) d -= 360.0f;
		if (d < -180.0f) d += 360.0f;
		return d;
	}

	static bool rx_contains_ok(const std::uint8_t* rx, std::size_t rx_len)
	{
		if (rx == nullptr || rx_len < 2)
		{
			return false;
		}
		for (std::size_t i = 0; i + 1 < rx_len; ++i)
		{
			const std::uint8_t a = rx[i];
			const std::uint8_t b = rx[i + 1];
			if ((a == 'O' && b == 'K') || (a == 'o' && b == 'k'))
			{
				return true;
			}
		}
		return false;
	}

	static void send_prate_10(BspUartHandle h)
	{
		if (h == nullptr)
		{
			return;
		}
		static constexpr char kCmd[] = "AT+PRATE=10\r\n";
		(void)bsp_uart_send(h, (std::uint8_t*)kCmd, static_cast<std::uint16_t>(sizeof(kCmd) - 1));
	}

	static void send_zero(BspUartHandle h)
	{
		if (h == nullptr)
		{
			return;
		}
		static constexpr char kCmd[] = "AT+ZERO\r\n";
		(void)bsp_uart_send(h, (std::uint8_t*)kCmd, static_cast<std::uint16_t>(sizeof(kCmd) - 1));
	}
}

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
	// 协议：ASCII AT 指令。
	// 回传角度格式："Angle:xxx.xx"（也可能是全角冒号 "Angle：xxx.xx"），以 \r/\n 结束。
	// 说明：本函数假设 rx[0..] 从 'A'/'a' 开始（外层会做滑窗扫描）。
	if (rx == nullptr || rx_len < 7)
	{
		return false;
	}

	// "Angle"（大小写不敏感）
	auto eq_ci = [](std::uint8_t a, char b) -> bool {
		return (a == static_cast<std::uint8_t>(b)) || (a == static_cast<std::uint8_t>(b - 'A' + 'a'));
	};
	if (!eq_ci(rx[0], 'A') || !eq_ci(rx[1], 'N') || !eq_ci(rx[2], 'G') || !eq_ci(rx[3], 'L') || !eq_ci(rx[4], 'E'))
	{
		return false;
	}

	std::size_t pos = 5;
	// 分隔符：':' 或 UTF-8 全角 '：' (EF BC 9A)
	if (pos < rx_len && rx[pos] == static_cast<std::uint8_t>(':'))
	{
		pos += 1;
	}
	else if (pos + 2 < rx_len && rx[pos] == 0xEF && rx[pos + 1] == 0xBC && rx[pos + 2] == 0x9A)
	{
		pos += 3;
	}
	else
	{
		return false;
	}

	while (pos < rx_len && (rx[pos] == ' ' || rx[pos] == '\t'))
	{
		++pos;
	}

	bool neg = false;
	if (pos < rx_len && rx[pos] == '-')
	{
		neg = true;
		++pos;
	}

	int integer_part = 0;
	int fractional_part = 0;
	float fractional_scale = 1.0f;
	bool has_digits = false;
	bool has_dot = false;

	for (; pos < rx_len; ++pos)
	{
		const std::uint8_t ch = rx[pos];
		if (ch >= '0' && ch <= '9')
		{
			has_digits = true;
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
		if (ch == '\r' || ch == '\n')
		{
			break;
		}
		// 遇到其它字符：如果已经读到数字就结束，否则认为无效
		break;
	}

	if (!has_digits)
	{
		return false;
	}

	float val = static_cast<float>(integer_part) + static_cast<float>(fractional_part) * fractional_scale;
	out_angle = neg ? -val : val;
	return true;
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

	Angle* angles = Angles();
	auto& self = Instance();

	for (std::size_t i = 0; i < kAngleChannels; ++i)
	{
		if ((updated_mask & (1UL << i)) == 0)
		{
			continue;
		}

		const uint16_t len = rx_len_[i];
		const auto* rx = angles[i].rx.data();

		// 未进入 Running 前：只做握手（等待 OK），收到 OK 后推进状态机。
		if (self.sensor_state_[i] != SensorState::Running)
		{
			if (rx_contains_ok(rx, len))
			{
				if (self.sensor_state_[i] == SensorState::WaitZeroOk || self.sensor_state_[i] == SensorState::NeedZero)
				{
					self.sensor_state_[i] = SensorState::NeedPrate;
					self.next_config_retry_tick_[i] = 0; // 立刻发送 PRATE
				}
				else if (self.sensor_state_[i] == SensorState::WaitPrateOk || self.sensor_state_[i] == SensorState::NeedPrate)
				{
					self.sensor_state_[i] = SensorState::Running;
					self.unwrap_inited_[i] = false;
					self.unwrapped_deg_[i] = 0.0f;
					self.last_raw_deg_[i] = 0.0f;
				}
			}
			continue;
		}

		float angle = 0.0f;
		if (try_parse_stream(rx, len, angle))
		{
			const float raw_deg = wrap_deg_360(angle);
			if (!self.unwrap_inited_[i])
			{
				self.unwrap_inited_[i] = true;
				self.last_raw_deg_[i] = raw_deg;
				// 初值：把 [0,360) 映射到 (-180,+180]，并作为“累计角度”的起点。
				self.unwrapped_deg_[i] = (raw_deg > 180.0f) ? (raw_deg - 360.0f) : raw_deg;
			}
			else
			{
				const float d = unwrap_delta_deg(raw_deg, self.last_raw_deg_[i]);
				self.unwrapped_deg_[i] += d;
				self.last_raw_deg_[i] = raw_deg;
			}

			constexpr float kDegToRad = 0.017453292519943295769f;
			angles[i].angle = self.unwrapped_deg_[i] * kDegToRad;
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
	constexpr uint32_t kConfigRetryMs = 500;
	const uint32_t retry_ticks = (uint32_t)((kConfigRetryMs * osKernelGetTickFreq() + 999U) / 1000U);

	// 上电配置：先设置零点 AT+ZERO（等待 OK），再设置回传频率 AT+PRATE=10（等待 OK），
	// 两步完成后才开始解析 Angle。
	for (std::size_t ch = 0; ch < kAngleChannels; ++ch)
	{
		unwrap_inited_[ch] = false;
		last_raw_deg_[ch] = 0.0f;
		unwrapped_deg_[ch] = 0.0f;

		if (tx_uart_for_channel(ch) != nullptr)
		{
			sensor_state_[ch] = SensorState::NeedZero;
			next_config_retry_tick_[ch] = 0;
		}
		else
		{
			sensor_state_[ch] = SensorState::Running;
			next_config_retry_tick_[ch] = 0;
		}
	}

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
		// 配置重试：直到收到 OK（分两步：ZERO -> PRATE）
		const uint32_t now = osKernelGetTickCount();
		for (std::size_t ch = 0; ch < kAngleChannels; ++ch)
		{
			if (sensor_state_[ch] == SensorState::Running)
			{
				continue;
			}
			if (now < next_config_retry_tick_[ch])
			{
				continue;
			}
			auto h = tx_uart_for_channel(ch);
			if (h == nullptr)
			{
				continue;
			}
			if (sensor_state_[ch] == SensorState::NeedZero || sensor_state_[ch] == SensorState::WaitZeroOk)
			{
				send_zero(h);
				sensor_state_[ch] = SensorState::WaitZeroOk;
			}
			else
			{
				send_prate_10(h);
				sensor_state_[ch] = SensorState::WaitPrateOk;
			}
			next_config_retry_tick_[ch] = now + retry_ticks;
		}

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
		std::uint16_t raw_x_phys = 0;
		std::uint16_t raw_y_phys = 0;
		std::uint16_t raw_z_phys = 0;
		(void)bsp_adc_read_raw(BSP_ADC1_JOYSTICK_X, &raw_x_phys);
		(void)bsp_adc_read_raw(BSP_ADC1_JOYSTICK_Y, &raw_y_phys);
		(void)bsp_adc_read_raw(BSP_ADC1_JOYSTICK_Z, &raw_z_phys);

		// 摇杆标定（分段线性）：保证 min->0、mid->2048、max->4095。
		// 你当前实测：
		// X: min=250 mid=1804 max=4095
		// Y: min=0   mid=2140 max=4000 (max 未到满量程)
		// Z: min=0   mid=2108 max=4034 (max 未到满量程)
		constexpr std::int32_t kOutMin = 0;
		constexpr std::int32_t kOutMid = 2048;
		constexpr std::int32_t kOutMax = 4095;

		auto clamp12 = [](std::int32_t v) -> std::uint16_t {
			if (v < 0) v = 0;
			if (v > 4095) v = 4095;
			return static_cast<std::uint16_t>(v);
		};

		auto map_axis = [&](std::uint16_t raw, std::int32_t in_min, std::int32_t in_mid, std::int32_t in_max) -> std::uint16_t {
			const std::int32_t r = static_cast<std::int32_t>(raw);
			if (in_max <= in_min)
			{
				return clamp12(r);
			}
			if (r <= in_min) return static_cast<std::uint16_t>(kOutMin);
			if (r >= in_max) return static_cast<std::uint16_t>(kOutMax);

			if (r <= in_mid)
			{
				const std::int32_t den = (in_mid - in_min);
				if (den <= 0) return static_cast<std::uint16_t>(kOutMin);
				const std::int32_t num = (r - in_min);
				const std::int32_t out = (num * kOutMid + den / 2) / den;
				return clamp12(out);
			}
			else
			{
				const std::int32_t den = (in_max - in_mid);
				if (den <= 0) return static_cast<std::uint16_t>(kOutMax);
				const std::int32_t num = (r - in_mid);
				const std::int32_t out = kOutMid + (num * (kOutMax - kOutMid) + den / 2) / den;
				return clamp12(out);
			}
		};

		input.adc_x_raw = map_axis(raw_x_phys, 250, 1804, 4095);
		input.adc_y_raw = map_axis(raw_y_phys, 0, 2140, 4000);
		input.adc_z_raw = map_axis(raw_z_phys, 0, 2108, 4034);

		// 中心死区：以 2048 为“零点”，±120 范围内钉到 2048。
		constexpr std::uint16_t kDeadzone = 120;
		auto apply_center_deadzone = [&](std::uint16_t v) -> std::uint16_t {
			if (v >= (kOutMid - kDeadzone) && v <= (kOutMid + kDeadzone))
			{
				return static_cast<std::uint16_t>(kOutMid);
			}
			return v;
		};
		input.adc_x_raw = apply_center_deadzone(input.adc_x_raw);
		input.adc_y_raw = apply_center_deadzone(input.adc_y_raw);
		input.adc_z_raw = apply_center_deadzone(input.adc_z_raw);

		auto raw_to_mv = [&](std::uint16_t raw) -> std::uint16_t {
			// 12-bit: 0..4095
			const std::uint32_t mv = (static_cast<std::uint32_t>(raw) * kVrefMv + 2047u) / 4095u;
			return static_cast<std::uint16_t>(mv);
		};
		// mV 保持物理含义：按“未标定的真实 raw”换算。
		input.adc_x_mv = raw_to_mv(raw_x_phys);
		input.adc_y_mv = raw_to_mv(raw_y_phys);
		input.adc_z_mv = raw_to_mv(raw_z_phys);

		orb::input_state.publish(input);
	}
}

