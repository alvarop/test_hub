#include <stdio.h>
#include <stdint.h>

#include "console.h"
#include "board.h"
#include "fifo.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"

#include "stm32f0xx_usart.h"

#define BLINK_DELAY_MS	(500)
#define FIFO_BUFF_SIZE  (4096)

fifo_t txFifo;
fifo_t rxFifo;
static uint8_t outBuff[FIFO_BUFF_SIZE];
static uint8_t inBuff[FIFO_BUFF_SIZE];

USB_CORE_HANDLE  USB_Device_dev;

volatile uint32_t tickMs = 0;

void init_uart() {
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

void init() {
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	GPIO_Init(_DBG_LED_PORT, &(GPIO_InitTypeDef){(1 << _DBG_LED_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});


	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(_LED0_PORT, &(GPIO_InitTypeDef){(1 << _LED0_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED1_PORT, &(GPIO_InitTypeDef){(1 << _LED1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED2_PORT, &(GPIO_InitTypeDef){(1 << _LED2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_Init(_5V_EN1_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_5V_EN2_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_5V_EN3_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN3_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	GPIO_ResetBits(_5V_EN1_PORT, (1 << _5V_EN1_PIN));
	GPIO_ResetBits(_5V_EN2_PORT, (1 << _5V_EN2_PIN));
	GPIO_ResetBits(_5V_EN3_PORT, (1 << _5V_EN3_PIN));

	init_uart();

	GPIO_ResetBits(_LED0_PORT, (1 << _LED0_PIN));
	GPIO_ResetBits(_LED1_PORT, (1 << _LED1_PIN));
	GPIO_ResetBits(_LED2_PORT, (1 << _LED2_PIN));

	// USBD_Init(&USB_Device_dev,
	// 		&USR_desc,
	// 		&USBD_CDC_cb,
	// 		&USR_cb);
}

int main(void) {
	uint32_t nextBlink;
	uint32_t blinkState = 0;

	init();

	nextBlink = tickMs + BLINK_DELAY_MS;
	for(;;) {

		if(tickMs > nextBlink) {
			nextBlink = tickMs + BLINK_DELAY_MS;
			if(blinkState) {
				GPIO_SetBits(_DBG_LED_PORT, (1 << _DBG_LED_PIN));
			} else {
				GPIO_ResetBits(_DBG_LED_PORT, (1 << _DBG_LED_PIN));
			}
			blinkState ^= 1;
		}

		consoleProcess();

		__WFI();
	}

	return 0;
}

void SysTick_Handler(void)
{
	tickMs++;
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

