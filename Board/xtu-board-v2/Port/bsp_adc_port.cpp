#include "bsp_adc_port.h"

#include "adc.h"  // hadc1
#include "main.h" // HAL

#include <cstdint>

namespace
{
	bool config_channel(ADC_HandleTypeDef* hadc, uint32_t channel)
	{
		if (hadc == nullptr) return false;

		ADC_ChannelConfTypeDef sConfig{};
		sConfig.Channel = channel;
		sConfig.Rank = 1;
		// 摇杆/电位器类信号建议更长采样时间，降低噪声并提升通道切换后的稳定性。
		sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
		return HAL_ADC_ConfigChannel(hadc, &sConfig) == HAL_OK;
	}

	bool read_once(ADC_HandleTypeDef* hadc, uint16_t* out_raw)
	{
		if (hadc == nullptr || out_raw == nullptr) return false;

		// 多通道切换/快速连续读取时，第一笔可能受上一次通道残留影响；
		// 这里做“双采样取第二次”，代价很小但稳定性提升明显。
		for (int pass = 0; pass < 2; ++pass)
		{
			if (HAL_ADC_Start(hadc) != HAL_OK) return false;
			const HAL_StatusTypeDef st = HAL_ADC_PollForConversion(hadc, 10);
			if (st != HAL_OK)
			{
				(void)HAL_ADC_Stop(hadc);
				return false;
			}
			const uint32_t v = HAL_ADC_GetValue(hadc);
			(void)HAL_ADC_Stop(hadc);

			if (pass == 1)
			{
				*out_raw = static_cast<uint16_t>(v & 0x0FFFu);
				return true;
			}
		}

		return false;
	}

	bool map_channel(BspAdcChannelId ch, ADC_HandleTypeDef*& out_adc, uint32_t& out_channel)
	{
		out_adc = &hadc1;
		switch (ch)
		{
			case BSP_ADC1_JOYSTICK_Z: out_channel = ADC_CHANNEL_14; return true;
			case BSP_ADC1_JOYSTICK_X: out_channel = ADC_CHANNEL_15; return true;
			case BSP_ADC1_JOYSTICK_Y: out_channel = ADC_CHANNEL_8; return true;
			default: return false;
		}
	}
}

bool bsp_adc_read_raw(BspAdcChannelId ch, uint16_t* out_raw)
{
	ADC_HandleTypeDef* adc = nullptr;
	uint32_t channel = 0;
	if (!map_channel(ch, adc, channel)) return false;

	if (!config_channel(adc, channel)) return false;
	return read_once(adc, out_raw);
}

bool bsp_adc_read_millivolts(BspAdcChannelId ch, uint32_t vref_mv, uint32_t* out_mv)
{
	if (out_mv == nullptr || vref_mv == 0) return false;

	uint16_t raw = 0;
	if (!bsp_adc_read_raw(ch, &raw)) return false;

	// 12-bit: 0..4095
	*out_mv = (static_cast<uint32_t>(raw) * vref_mv) / 4095u;
	return true;
}
