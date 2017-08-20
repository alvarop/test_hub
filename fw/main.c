#include <stdio.h>
#include <stdint.h>
#include "console.h"
#include "board.h"
#include "i2c.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb2514b.h"
#include "uart.h"

#define BLINK_DELAY_MS	(500)

USB_CORE_HANDLE  USB_Device_dev;

volatile uint32_t tickMs = 0;

void gpio_init() {

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

	GPIO_Init(_DBG_LED_PORT, &(GPIO_InitTypeDef){(1 << _DBG_LED_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED1_PORT, &(GPIO_InitTypeDef){(1 << _LED1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED2_PORT, &(GPIO_InitTypeDef){(1 << _LED2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED3_PORT, &(GPIO_InitTypeDef){(1 << _LED3_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	GPIO_ResetBits(_LED1_PORT, (1 << _LED1_PIN));
	GPIO_ResetBits(_LED2_PORT, (1 << _LED2_PIN));
	GPIO_ResetBits(_LED3_PORT, (1 << _LED3_PIN));
}

int main(void) {
	uint32_t nextBlink;
	uint32_t blinkState = 0;

	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	gpio_init();

	uart_init();

	fprintf(stderr, "Starting Test Hub %s\n", FW_VERSION);

	USBD_Init(&USB_Device_dev,
			&USR_desc,
			&USBD_CDC_cb,
			&USR_cb);

	i2cSetup(100000);

	usb2514b_init();

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
