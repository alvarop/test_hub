#include "uart.h"
#include "board.h"
#include "fifo.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_usart.h"

#define FIFO_BUFF_SIZE  (4096)

fifo_t txFifo;
fifo_t rxFifo;
static uint8_t outBuff[FIFO_BUFF_SIZE];
static uint8_t inBuff[FIFO_BUFF_SIZE];

uint32_t uart_init() {
	USART_InitTypeDef uartConfig;

	fifoInit(&txFifo, FIFO_BUFF_SIZE, outBuff);
	fifoInit(&rxFifo, FIFO_BUFF_SIZE, inBuff);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_Init(_USART3_TX_PORT, &(GPIO_InitTypeDef){(1 << _USART3_TX_PIN), GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_USART3_RX_PORT, &(GPIO_InitTypeDef){(1 << _USART3_RX_PIN), GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_PinAFConfig(_USART3_TX_PORT, _USART3_TX_PIN, GPIO_AF_4);
	GPIO_PinAFConfig(_USART3_RX_PORT, _USART3_RX_PIN, GPIO_AF_4);

	USART_StructInit(&uartConfig);

	uartConfig.USART_BaudRate = 115200;
	USART_Init(USART3, &uartConfig);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

	USART_Cmd(USART3, ENABLE);

	NVIC_EnableIRQ(USART3_4_IRQn);

	return 0;
}

int uartPutchar(USART_TypeDef *uart, fifo_t *fifo, char c) {
	if(uart->ISR & USART_ISR_TXE) {
		uart->TDR = c;
		USART_ITConfig(uart, USART_IT_TXE, ENABLE);
	} else {
		fifoPush(fifo, c);
	}

	return c;
}

//
// Retarget read/write to use usb/serial!
// Found in share/gcc-arm-none-eabi/samples/src/retarget/retarget.c
//
int _write (int fd, char *ptr, int len)
{
	//
	// If planning on supporting both serial and usb-serial, check fd here!
	//
	while(len--) {
		uartPutchar(USART3, &txFifo, *ptr++);
	}
	return len;
}

int _read (int fd, char *ptr, int len)
{
  int readChars = 0;

  //
  // If planning on supporting both serial and usb-serial, check fd here!
  //
  while(fifoSize(&rxFifo) && len--) {
    *ptr++ = fifoPop(&rxFifo);
  }

  return readChars;
}

void USART3_4_IRQHandler(void) {
	uint32_t irq = USART3->ISR;

	if(irq & USART_ISR_TXE) {
		// Tx new byte if available
		if(fifoSize(&txFifo) > 0) {
			USART3->TDR = fifoPop(&txFifo);

		} else {
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
	}

	if(irq & USART_ISR_RXNE) {
		// Add received char to buff
		fifoPush(&rxFifo, USART3->RDR);
	}

	USART3->ICR = irq;
}
