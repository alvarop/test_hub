#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "console.h"
#include "board.h"
#include "fifo.h"
#include "i2c.h"
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

const uint8_t hub_cfg[256] = {
    0x24, // Vendor ID LSB
    0x04, // Vendor ID MSB
    0x14, // Product ID LSB
    0x25, // Product ID MSB
    0xB3, // Device ID LSB
    0x0B, // Device ID MSB
    0x9B, // Configuration Data Byte 1
    0x20, // Configuration Data Byte 2
    0x02, // Configuration Data Byte 3
    0x00, // Non-Removable Devices
    0x00, // Port Disable (Self)
    0x00, // Port Disable (Bus)
    0x01, // Max Power (Self)
    0x32, // Max Power (Bus)
    0x01, // Hub Controller Max Current (Self)
    0x32, // Hub Controller Max Current (Bus)
    0x32, // Power-on Time
    0x00, // Language ID High
    0x00, // Language ID Low
    0x00, // Manufacturer String Length
    0x00, // Product String Length
    0x00, // Serial String Length
    0x00, // Manufacturer String
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, // Product String
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, // Serial String
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, // Battery Charging Enable
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, // rsvd
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, //
    0x00, // rsvd
    0x00, // Boost_Up
    0x00, // rsvd
    0x00, // Boost_x:0
    0x00, // rsvd
    0x00, // Port Swap
    0x00, // Port Map 12
    0x00, // Port Map 34
    0x00, // rsvd
    0x00, // rsvd
    0x01, // Status/Command
};

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

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_Init(_HUB_RST_N_PORT, &(GPIO_InitTypeDef){(1 << _HUB_RST_N_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_CFG_SEL1_PORT, &(GPIO_InitTypeDef){(1 << _CFG_SEL1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	// GPIO_Init(_SUSP_IND_PORT, &(GPIO_InitTypeDef){(1 << _SUSP_IND_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_SDA_PORT, &(GPIO_InitTypeDef){(1 << _SDA_PIN), GPIO_Mode_AF, GPIO_Speed_2MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL});
	GPIO_Init(_SCL_PORT, &(GPIO_InitTypeDef){(1 << _SCL_PIN), GPIO_Mode_AF, GPIO_Speed_2MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL});

	GPIO_Init(_OCS_N1_PORT, &(GPIO_InitTypeDef){(1 << _OCS_N1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_OCS_N2_PORT, &(GPIO_InitTypeDef){(1 << _OCS_N2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_OCS_N3_PORT, &(GPIO_InitTypeDef){(1 << _OCS_N3_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	GPIO_SetBits(_OCS_N1_PORT, (1 << _OCS_N1_PIN));
	GPIO_SetBits(_OCS_N2_PORT, (1 << _OCS_N2_PIN));
	GPIO_SetBits(_OCS_N3_PORT, (1 << _OCS_N3_PIN));

	// Hold USB Hub in reset for now
	GPIO_ResetBits(_HUB_RST_N_PORT, (1 << _HUB_RST_N_PIN));

	// While I get I2C working, set these low for default hub configuration
	GPIO_ResetBits(_CFG_SEL1_PORT, (1 << _CFG_SEL1_PIN));
	// GPIO_ResetBits(_SUSP_IND_PORT, (1 << _SUSP_IND_PIN));

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	GPIO_Init(_DBG_LED_PORT, &(GPIO_InitTypeDef){(1 << _DBG_LED_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_Init(_LED0_PORT, &(GPIO_InitTypeDef){(1 << _LED0_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED1_PORT, &(GPIO_InitTypeDef){(1 << _LED1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_LED2_PORT, &(GPIO_InitTypeDef){(1 << _LED2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	GPIO_Init(_5V_EN1_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_5V_EN2_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_5V_EN3_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN3_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	GPIO_ResetBits(_5V_EN1_PORT, (1 << _5V_EN1_PIN));
	GPIO_ResetBits(_5V_EN2_PORT, (1 << _5V_EN2_PIN));
	GPIO_ResetBits(_5V_EN3_PORT, (1 << _5V_EN3_PIN));

	init_uart();
	i2cSetup(100000);

	GPIO_ResetBits(_LED0_PORT, (1 << _LED0_PIN));
	GPIO_ResetBits(_LED1_PORT, (1 << _LED1_PIN));
	GPIO_ResetBits(_LED2_PORT, (1 << _LED2_PIN));

	for(uint32_t delay=0; delay < 100000; delay++) {
		__NOP();
	}

	printf("wake up hub\n");
	// Wake up the hub!
	GPIO_SetBits(_HUB_RST_N_PORT, (1 << _HUB_RST_N_PIN));

	for(uint32_t delay=0; delay < 100000; delay++) {
		__NOP();
	}

	printf("Write hub config\n"); // 32 bytes at a time...

	int32_t rval;
    for(uint32_t byte = 0; byte < sizeof(hub_cfg); byte += 32) {
        uint8_t cpy_buff[34];
        cpy_buff[0] = byte;
        cpy_buff[1] = 32;
        memcpy(&cpy_buff[2], &hub_cfg[byte], 32);
        rval = i2c(I2C1, (0x2C << 1), sizeof(cpy_buff), (uint8_t *)cpy_buff, 0, NULL);
    }

	USBD_Init(&USB_Device_dev,
			&USR_desc,
			&USBD_CDC_cb,
			&USR_cb);
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

