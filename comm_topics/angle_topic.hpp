#pragma once

#include "topic.hpp"

namespace orb
{
    struct AngleFrame
    {
        static constexpr std::size_t kChannels = 4;
        float angles[kChannels]{};
    };

	inline Topic<AngleFrame> angle_frame;
}
