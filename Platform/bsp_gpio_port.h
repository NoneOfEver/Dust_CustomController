/**
 * @file bsp_gpio_port.h
 * @brief 上层可用的 GPIO 端口抽象（HAL-free，C ABI）
 *
 * 说明：
 * - 本层只做“端口抽象 + 回调转发”，不负责硬件初始化。
 * - GPIO/EXTI 的模式、上下拉、NVIC 使能等仍由 Board 层（CubeMX 生成代码）配置。
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct BspGpioOpaque* BspGpioHandle;

// Board-level GPIO id（当前根据 Board/xtu-board-v2/Inc/main.h 的引脚定义）
typedef enum {
	BSP_GPIO_BUTTON1 = 1,
	BSP_GPIO_BUTTON2 = 2,
	BSP_GPIO_BUTTON3 = 3,
	BSP_GPIO_BUTTON4 = 4,
	BSP_GPIO_JOYSTICK_BUTTON = 5,
} BspGpioId;

// 单 bit 掩码（与 STM32 HAL GPIO_PIN_x 数值一致）
typedef uint16_t BspGpioPin;

enum {
	BSP_GPIO_PIN_0 = 0x0001u,
	BSP_GPIO_PIN_1 = 0x0002u,
	BSP_GPIO_PIN_2 = 0x0004u,
	BSP_GPIO_PIN_3 = 0x0008u,
	BSP_GPIO_PIN_4 = 0x0010u,
	BSP_GPIO_PIN_5 = 0x0020u,
	BSP_GPIO_PIN_6 = 0x0040u,
	BSP_GPIO_PIN_7 = 0x0080u,
	BSP_GPIO_PIN_8 = 0x0100u,
	BSP_GPIO_PIN_9 = 0x0200u,
	BSP_GPIO_PIN_10 = 0x0400u,
	BSP_GPIO_PIN_11 = 0x0800u,
	BSP_GPIO_PIN_12 = 0x1000u,
	BSP_GPIO_PIN_13 = 0x2000u,
	BSP_GPIO_PIN_14 = 0x4000u,
	BSP_GPIO_PIN_15 = 0x8000u,
};

typedef void (*BspGpioExtiCallback)(BspGpioPin pin);

BspGpioHandle bsp_gpio_get(BspGpioId id);

bool bsp_gpio_read(BspGpioHandle h);
void bsp_gpio_write(BspGpioHandle h, bool high);
void bsp_gpio_toggle(BspGpioHandle h);

// EXTI 回调注册：按“EXTI 线（Pin bit）”注册。
// 注意：EXTI 线是按 pin number（0..15）共享的（同一时间只能映射到某个 port）。
void bsp_gpio_exti_register(BspGpioPin pin, BspGpioExtiCallback cb);
void bsp_gpio_exti_unregister(BspGpioPin pin);

#ifdef __cplusplus
}
#endif
