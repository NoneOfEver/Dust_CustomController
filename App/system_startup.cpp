#include "system_startup.h"
#include "bsp_uart_port.h"
#include "bsp_gpio_port.h"
#include "bsp_adc_port.h"
#include "cmsis_os2.h"

#include "angle_reader.h"
#include "debug_tools.h"
#include "main.h"
#include "referee_sender.h"



void uart1_callback(uint8_t* buffer, uint16_t length) 
{
	AngleReciever::OnRx(0, buffer, length);
}
void uart2_callback(uint8_t* buffer, uint16_t length)
{

}
void uart3_callback(uint8_t* buffer, uint16_t length)
{

}
void uart4_callback(uint8_t* buffer, uint16_t length)
{
	AngleReciever::OnRx(1, buffer, length);
}
void uart5_callback(uint8_t* buffer, uint16_t length)
{
	AngleReciever::OnRx(2, buffer, length);
}
void uart6_callback(uint8_t* buffer, uint16_t length)
{

}
void startup_thread(void *argument)
{
	(void)argument;
    
    bsp_uart_init(bsp_uart_get(BSP_UART1), uart1_callback, 512);
    bsp_uart_init(bsp_uart_get(BSP_UART2), uart2_callback, 512);
    bsp_uart_init(bsp_uart_get(BSP_UART3), uart3_callback, 512);
    bsp_uart_init(bsp_uart_get(BSP_UART4), uart4_callback, 512);
    bsp_uart_init(bsp_uart_get(BSP_UART5), uart5_callback, 512);
    bsp_uart_init(bsp_uart_get(BSP_UART6), uart6_callback, 512);

	Vofa::InjectTxUart(bsp_uart_get(BSP_UART5));
	RefereeSender::InjectTxUart(bsp_uart_get(BSP_UART6));

	AngleReciever::StartThread();
	Vofa::StartThread();
	RefereeSender::StartThread();

	osThreadExit();
}