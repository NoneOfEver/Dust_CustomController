#include "bsp_gpio_port.h"

#include "main.h" // stm32f4xx_hal.h + pin defines

#include <cstddef>

struct BspGpioOpaque {
	GPIO_TypeDef* port;
	uint16_t pin;
};

static BspGpioOpaque s_button1{BUTTON1_GPIO_Port, BUTTON1_Pin};
static BspGpioOpaque s_button2{BUTTON2_GPIO_Port, BUTTON2_Pin};
static BspGpioOpaque s_button3{BUTTON3_GPIO_Port, BUTTON3_Pin};
static BspGpioOpaque s_button4{BUTTON4_GPIO_Port, BUTTON4_Pin};
static BspGpioOpaque s_joystick_btn{JOYSTICK_BUTTON_GPIO_Port, JOYSTICK_BUTTON_Pin};

static inline BspGpioOpaque* to_impl(BspGpioHandle h)
{
	return reinterpret_cast<BspGpioOpaque*>(h);
}

BspGpioHandle bsp_gpio_get(BspGpioId id)
{
	switch (id)
	{
		case BSP_GPIO_BUTTON1: return reinterpret_cast<BspGpioHandle>(&s_button1);
		case BSP_GPIO_BUTTON2: return reinterpret_cast<BspGpioHandle>(&s_button2);
		case BSP_GPIO_BUTTON3: return reinterpret_cast<BspGpioHandle>(&s_button3);
		case BSP_GPIO_BUTTON4: return reinterpret_cast<BspGpioHandle>(&s_button4);
		case BSP_GPIO_JOYSTICK_BUTTON: return reinterpret_cast<BspGpioHandle>(&s_joystick_btn);
		default: return nullptr;
	}
}

bool bsp_gpio_read(BspGpioHandle h)
{
	auto* impl = to_impl(h);
	if (!impl || !impl->port || impl->pin == 0) return false;
	return HAL_GPIO_ReadPin(impl->port, impl->pin) == GPIO_PIN_SET;
}

void bsp_gpio_write(BspGpioHandle h, bool high)
{
	auto* impl = to_impl(h);
	if (!impl || !impl->port || impl->pin == 0) return;
	HAL_GPIO_WritePin(impl->port, impl->pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void bsp_gpio_toggle(BspGpioHandle h)
{
	auto* impl = to_impl(h);
	if (!impl || !impl->port || impl->pin == 0) return;
	HAL_GPIO_TogglePin(impl->port, impl->pin);
}

// ---- EXTI callback registry ----

static BspGpioExtiCallback s_exti_cb[16] = {0};

static int pin_to_index(BspGpioPin pin)
{
	if (pin == 0) return -1;
	// must be single bit
	if ((pin & (pin - 1u)) != 0) return -1;

	for (int i = 0; i < 16; ++i)
	{
		if (pin == (static_cast<BspGpioPin>(1u) << i))
		{
			return i;
		}
	}
	return -1;
}

void bsp_gpio_exti_register(BspGpioPin pin, BspGpioExtiCallback cb)
{
	const int idx = pin_to_index(pin);
	if (idx < 0) return;
	s_exti_cb[idx] = cb;
}

void bsp_gpio_exti_unregister(BspGpioPin pin)
{
	const int idx = pin_to_index(pin);
	if (idx < 0) return;
	s_exti_cb[idx] = nullptr;
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	const int idx = pin_to_index(static_cast<BspGpioPin>(GPIO_Pin));
	if (idx < 0) return;

	auto cb = s_exti_cb[idx];
	if (cb)
	{
		cb(static_cast<BspGpioPin>(GPIO_Pin));
	}
}
