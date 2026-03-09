/**
 * @file bsp_adc_port.h
 * @brief 上层可用的 ADC 抽象（HAL-free，C ABI）
 *
 * 说明：
 * - 本层提供简单的“单次采样”读取接口。
 * - ADC 初始化（时钟、通道配置、引脚模拟输入等）仍由 Board 层（CubeMX 生成代码）负责。
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	BSP_ADC1_JOYSTICK_Z = 1, // PC4 / ADC1_IN14
	BSP_ADC1_JOYSTICK_X = 2, // PC5 / ADC1_IN15
	BSP_ADC1_JOYSTICK_Y = 3, // PB0 / ADC1_IN8
} BspAdcChannelId;

// 读取 12-bit raw（0..4095）。成功返回 true。
bool bsp_adc_read_raw(BspAdcChannelId ch, uint16_t* out_raw);

// 读取电压值（mV）。vref_mv 通常传 3300。
bool bsp_adc_read_millivolts(BspAdcChannelId ch, uint32_t vref_mv, uint32_t* out_mv);

#ifdef __cplusplus
}
#endif
