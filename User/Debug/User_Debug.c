#include "User_Debug.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"

void Debug_Usart(void)
{
	usart_printf("hello\r\n");

}

void usart_printf(const char *fmt,...)
{
  static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
		
  va_start(ap, fmt);
  len = vsprintf((char *)tx_buf, fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart1,tx_buf,len,100);
//	HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
}

