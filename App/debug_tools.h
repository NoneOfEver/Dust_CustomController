#pragma once

#include <cstdint>

#include "bsp_uart_port.h"

class Vofa
{
public:
	Vofa() = default;
	static void InjectTxUart(BspUartHandle uart);
	static void StartThread();
	void ThreadLoop();

private:
	static Vofa& Instance();
	static void ThreadEntry(void* argument);

	void SendFloat(float data);
	void SendTail();
	
	BspUartHandle tx_uart_{nullptr};
};
