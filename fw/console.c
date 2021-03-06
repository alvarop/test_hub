#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "dfu.h"
#include "fifo.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "adc.h"

typedef struct {
  char *commandStr;
  void (*fn)(uint8_t argc, char *argv[]);
  char *helpStr;
} command_t;

extern fifo_t rxFifo;

static uint8_t *uid = (uint8_t *)(0x1FFFF7AC);

static char cmdBuff[1024];
static uint8_t argc;
static char *argv[255];

static void helpFn(uint8_t argc, char *argv[]);
static void snCmd(uint8_t argc, char *argv[]);
static void versionCmd(uint8_t argc, char *argv[]);
static void portCmd(uint8_t argc, char *argv[]);
static void ResetCmd(uint8_t argc, char *argv[]);
static void DfuCmd(uint8_t argc, char *argv[]);
static void adcCmd(uint8_t argc, char *argv[]);


static const char versionStr[] = FW_VERSION;

static command_t commands[] = {
    {"sn", snCmd, "sn"},
    {"version", versionCmd, "version"},
    {"port", portCmd, "port <1-3> <0|1> - Enable/disable port"},
    {"reset", ResetCmd, "System reset"},
    {"dfu", DfuCmd, "Switch to USB DFU bootloader"},
    {"adc", adcCmd, "adc <1-4> - Read adc channel"},
    // Add new commands here!
    {"help", helpFn, "Print this!"},
    {NULL, NULL, NULL}};

//
// Print the help menu
//
static void helpFn(uint8_t argc, char *argv[]) {
  command_t *command = commands;

  if (argc < 2) {
    while (command->commandStr != NULL) {
      printf("%s - %s\n", command->commandStr, command->helpStr);
      command++;
    }
  } else {
    while (command->commandStr != NULL) {
      if (strcmp(command->commandStr, argv[1]) == 0) {
        printf("%s - %s\n", command->commandStr, command->helpStr);
        break;
      }
      command++;
    }
  }
}

static void snCmd(uint8_t argc, char *argv[]) {
  printf("OK ");

  // Print 96-bit serial number
  for (uint8_t byte = 0; byte < 12; byte++) {
    printf("%02X ", uid[byte]);
  }
  printf("\n");
}

static void versionCmd(uint8_t argc, char *argv[]) {
  printf("OK %s\n", versionStr);
}

static void portCmd(uint8_t argc, char *argv[]) {
  if (argc == 3) {
    bool enable = !(argv[2][0] == '0');
    switch (argv[1][0]) {
      case '1': {
        if (enable) {
          GPIO_SetBits(_5V_EN1_PORT, (1 << _5V_EN1_PIN));
        } else {
          GPIO_ResetBits(_5V_EN1_PORT, (1 << _5V_EN1_PIN));
        }
        printf("OK\n");
        break;
      }
      case '2': {
        if (enable) {
          GPIO_SetBits(_5V_EN2_PORT, (1 << _5V_EN2_PIN));
        } else {
          GPIO_ResetBits(_5V_EN2_PORT, (1 << _5V_EN2_PIN));
        }
        printf("OK\n");
        break;
      }
      case '3': {
        if (enable) {
          GPIO_SetBits(_5V_EN3_PORT, (1 << _5V_EN3_PIN));
        } else {
          GPIO_ResetBits(_5V_EN3_PORT, (1 << _5V_EN3_PIN));
        }
        printf("OK\n");
        break;
      }
      default: { printf("ERR\n"); }
    }
  } else {
    printf("ERR\n");
  }
}

static void ResetCmd(uint8_t argc, char *argv[]) { NVIC_SystemReset(); }

static void DfuCmd(uint8_t argc, char *argv[]) {
  printf("OK\n");
  EnterDfu();
}


static void adcCmd(uint8_t argc, char *argv[]) {
 if (argc == 2) {
   uint8_t ch = (uint8_t)strtoul(argv[1], NULL, 10);
   adc_read_single(ch); // Dummy read
   // TODO: Fix this ^ ^ ^
   int32_t adcval = adc_read_single(ch);
   // adcval = (adcval * 100 * 330) / 409600;
   // printf("OK %ld.%02ld\n", adcval / 100, (adcval - (adcval/100) * 100));
   printf("OK %ld\n", adcval);
 } else {
   printf("ERR\n");
 }
}


void consoleProcess() {
  uint32_t inBytes = fifoSize(&rxFifo);
  if (inBytes > 0) {
    uint32_t newLine = 0;
    for (int32_t index = 0; index < inBytes; index++) {
      if ((fifoPeek(&rxFifo, index) == '\n') ||
          (fifoPeek(&rxFifo, index) == '\r')) {
        newLine = index + 1;
        GPIO_SetBits(GPIOB, GPIO_Pin_2);
        break;
      }
    }

    if (newLine > sizeof(cmdBuff)) {
      newLine = sizeof(cmdBuff) - 1;
    }

    if (newLine) {
      uint8_t *pBuf = (uint8_t *)cmdBuff;
      while (newLine--) {
        *pBuf++ = fifoPop(&rxFifo);
      }

      // If it's an \r\n combination, discard the second one
      if ((fifoPeek(&rxFifo, 0) == '\n') || (fifoPeek(&rxFifo, 0) == '\r')) {
        fifoPop(&rxFifo);
      }

      *(pBuf - 1) = 0;  // String terminator

      argc = 0;

      // Get command
      argv[argc] = strtok(cmdBuff, " ");

      // Get arguments (if any)
      while ((argc < sizeof(argv) / sizeof(char *)) && (argv[argc] != NULL)) {
        argc++;
        argv[argc] = strtok(NULL, " ");
      }

      if (argc > 0) {
        command_t *command = commands;
        while (command->commandStr != NULL) {
          if (strcmp(command->commandStr, argv[0]) == 0) {
            command->fn(argc, argv);
            break;
          }
          command++;
        }

        if (command->commandStr == NULL) {
          printf("ERR Unknown command '%s'\n", argv[0]);
        }
      }
    }
  }
}
