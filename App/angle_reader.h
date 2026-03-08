#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

struct Angle
{
	static constexpr std::size_t kRxBufferSize = 20;

	std::array<std::uint8_t, kRxBufferSize> rx{};
	float angle{0.0f};

	std::uint8_t* rx_data() { return rx.data(); }
	const std::uint8_t* rx_data() const { return rx.data(); }
};


class AngleReciever
{
public:
	AngleReciever() = default;
	static void OnRx(std::size_t channel, uint8_t* buffer, uint16_t length);
	static void StartThread();
	void ThreadLoop();

private:
	static AngleReciever& Instance();
	static void ThreadEntry(void* argument);

	static constexpr std::size_t kMinParseBytes = 16;
	static constexpr std::size_t kAngleChannels = 4;

	Angle angles_[kAngleChannels]{};
	uint16_t rx_len_[kAngleChannels]{};

	Angle* Angles();
	void ParseAndPublish(uint32_t updated_mask);
	static bool TryParseAngleFromRx(const uint8_t* rx, std::size_t rx_len, float& out_angle);
};

