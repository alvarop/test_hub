#include <stdio.h>
#include <string.h>
#include "board.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "usb2514b.h"
#include "i2c.h"

extern const uint8_t hub_cfg[256];

uint32_t usb2514b_init() {
	GPIO_Init(_HUB_RST_N_PORT, &(GPIO_InitTypeDef){(1 << _HUB_RST_N_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	// Hold USB Hub in reset for now
	GPIO_ResetBits(_HUB_RST_N_PORT, (1 << _HUB_RST_N_PIN));

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

	GPIO_ResetBits(_CFG_SEL1_PORT, (1 << _CFG_SEL1_PIN));
	// GPIO_ResetBits(_SUSP_IND_PORT, (1 << _SUSP_IND_PIN));

	GPIO_Init(_5V_EN1_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN1_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_5V_EN2_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN2_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});
	GPIO_Init(_5V_EN3_PORT, &(GPIO_InitTypeDef){(1 << _5V_EN3_PIN), GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL});

	GPIO_ResetBits(_5V_EN1_PORT, (1 << _5V_EN1_PIN));
	GPIO_ResetBits(_5V_EN2_PORT, (1 << _5V_EN2_PIN));
	GPIO_ResetBits(_5V_EN3_PORT, (1 << _5V_EN3_PIN));

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

        if(rval != 0) {
        	printf("Error writing hub config %ld\n", rval);
        	break;
        }
    }

    return rval;
}
