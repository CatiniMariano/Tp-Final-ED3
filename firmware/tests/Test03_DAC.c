/*
 * Test03_DAC.c — verificar el DAC en P0.26.
 *
 * Como usar:
 *   - Mandar un numero 0-3300 + ENTER por UART (interpretado como mV)
 *   - El DAC sale a esa tension
 *
 * Verificacion fisica (P0.26 contra GND):
 *   - Multimetro en DC: deberia leer LA TENSION QUE PEDISTE en mV
 *       mande "1500"  -> ~1.50 V
 *       mande "3300"  -> ~3.30 V (saturacion)
 *       mande "0"     -> 0 V
 *
 * Diagnostico:
 *   - El multimetro mide siempre 0 V o algo random: PINSEL del P0.26 mal
 *     (debe ser funcion 2 -> PINSEL_FUNC_10) o PCONP del DAC no encendido
 *   - El multimetro mide bien pero a la mitad: error en la cuenta de mV->codigo
 */

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_uart.h"
#include <string.h>

#define UART_PC          ((LPC_UART_TypeDef *) LPC_UART0)
#define UART_BAUD        9600
#define RX_BUFFER_SIZE   8
#define DAC_MV_REF       3300UL

volatile char     rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t  rx_index = 0;
volatile uint8_t  comando_listo = 0;

void UART_SendString(const char *s) {
    UART_Send(UART_PC, (uint8_t *)s, strlen(s), BLOCKING);
}
void UART_SendUInt(uint32_t num) {
    char buf[11]; int8_t i = 0, j;
    if (num == 0) { UART_Send(UART_PC, (uint8_t *)"0", 1, BLOCKING); return; }
    while (num > 0 && i < 10) { buf[i++] = (char)((num % 10) + '0'); num /= 10; }
    for (j = i - 1; j >= 0; j--) UART_Send(UART_PC, (uint8_t *)&buf[j], 1, BLOCKING);
}

void dac_cfg(void) {
    PINSEL_CFG_T p = {
        .port = 0, .pin = 26, .func = PINSEL_FUNC_10,
        .mode = PINSEL_TRISTATE, .openDrain = DISABLE
    };
    PINSEL_ConfigPin(&p);
    DAC_Init();
}

void uart_cfg(void) {
    UART_CFG_T cfg = {
        .baudRate = UART_BAUD, .dataBits = UART_DBITS_8,
        .stopBits = UART_STOPBIT_1, .parity = UART_PARITY_NONE
    };
    UART_FIFO_CFG_T fifo = {
        .level = UART_FIFO_TRGLEV0,
        .resetRxBuf = ENABLE, .resetTxBuf = ENABLE, .dmaMode = DISABLE
    };
    UART_PinConfig(UART_TX0_P0_2);
    UART_PinConfig(UART_RX0_P0_3);
    UART_Init(UART_PC, &cfg);
    UART_FIFOConfig(UART_PC, &fifo);
    UART_TxEnable(UART_PC);
    UART_IntConfig(UART_PC, UART_INT_RBR, ENABLE);
    NVIC_EnableIRQ(UART0_IRQn);
}

void UART0_IRQHandler(void) {
    uint8_t dato;
    while (UART_Receive(UART_PC, &dato, 1, NONE_BLOCKING) == 1) {
        if (comando_listo) continue;
        if (dato == '\r' || dato == '\n') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                rx_index = 0;
                comando_listo = 1;
            }
        } else if (dato >= '0' && dato <= '9' && rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = dato;
        }
    }
}

uint16_t parsear_mv(volatile char *str) {
    uint32_t v = 0; uint8_t i = 0;
    if (str[0] == '\0') return 0xFFFF;
    while (str[i] != '\0' && i < RX_BUFFER_SIZE) {
        if (str[i] < '0' || str[i] > '9') return 0xFFFF;
        v = v * 10 + (uint32_t)(str[i] - '0');
        if (v > DAC_MV_REF) return 0xFFFF;
        i++;
    }
    return (uint16_t)v;
}

int main(void) {
    uart_cfg();
    dac_cfg();
    DAC_UpdateValue(0);

    UART_SendString("\r\n=== TEST 3: DAC en P0.26 ===\r\n");
    UART_SendString("Mande mV (0-3300) + ENTER, medi con multimetro en P0.26.\r\n\r\n");

    char local[RX_BUFFER_SIZE];

    while (1) {
        if (comando_listo) {
            uint8_t i;
            NVIC_DisableIRQ(UART0_IRQn);
            for (i = 0; i < RX_BUFFER_SIZE; i++) {
                local[i] = rx_buffer[i];
                if (rx_buffer[i] == '\0') break;
            }
            local[RX_BUFFER_SIZE - 1] = '\0';
            rx_buffer[0] = '\0'; rx_index = 0; comando_listo = 0;
            NVIC_EnableIRQ(UART0_IRQn);

            uint16_t mv = parsear_mv(local);
            if (mv <= DAC_MV_REF) {
                uint32_t dac_val = ((uint32_t)mv * 1023UL) / DAC_MV_REF;
                if (dac_val > 1023) dac_val = 1023;
                DAC_UpdateValue(dac_val);
                UART_SendString("DAC objetivo: ");
                UART_SendUInt(mv);
                UART_SendString(" mV (codigo ");
                UART_SendUInt(dac_val);
                UART_SendString("/1023)\r\n");
            } else {
                UART_SendString("ERROR: rango 0-3300 mV\r\n");
            }
        }
    }
}
