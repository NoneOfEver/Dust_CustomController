#pragma once

#include <cstdint>

#include "topic.hpp"

namespace orb
{
	struct InputState
	{
		// bit0..3: BUTTON1..4, bit4: JOYSTICK_BUTTON
		std::uint8_t buttons{0};

		std::uint16_t adc_x_raw{0};
		std::uint16_t adc_y_raw{0};
		std::uint16_t adc_z_raw{0};

		std::uint16_t adc_x_mv{0};
		std::uint16_t adc_y_mv{0};
		std::uint16_t adc_z_mv{0};
	};

	inline Topic<InputState> input_state;
}
