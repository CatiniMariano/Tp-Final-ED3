/*
 * Test01_UART.c — verificar que el UART anda en ambos sentidos.
 *
 * Que esperar en Tera Term (9600 8N1):
 *   - Al arrancar: "=== TEST 1: UART ==="
 *   - Cada 1 segundo: "TICK n"
 *   - Lo que tipees vuelve como eco caracter por caracter
 *
 * Diagnostico:
 *   - No aparece nada: problema de TX (PCLK, baud, P0.2, terminal mal config)
 *   - Sale el TICK pero no hace eco: problema de RX (P0.3, IRQ, FIFO)
 *   - Aparecen caracteres raros: PCLK mal o baud distinto al de Tera Term
 */
 
#include "LPC17xx.h"
#include "lpc17xx_uart.h"
#include <string.h>
 
#define UART_PC    ((LPC_UART_TypeDef *) LPC_UART0)
#define UART_BAUD  9600
 
volatile uint32_t tick_ms = 0;
volatile uint8_t  enviar_tick = 0;
volatile uint32_t contador_ticks = 0;
 
void SysTick_Handler(void) {
    if (++tick_ms >= 1000) {
        tick_ms = 0;
        enviar_tick = 1;
    }
}
 
void UART_SendString(const char *s) {
    UART_Send(UART_PC, (uint8_t *)s, strlen(s), BLOCKING);
}
 
void UART_SendUInt(uint32_t num) {
    char buf[11]; int8_t i = 0, j;
    if (num == 0) { UART_Send(UART_PC, (uint8_t *)"0", 1, BLOCKING); return; }
    while (num > 0 && i < 10) { buf[i++] = (char)((num % 10) + '0'); num /= 10; }
    for (j = i - 1; j >= 0; j--) UART_Send(UART_PC, (uint8_t *)&buf[j], 1, BLOCKING);
}
 
void UART0_IRQHandler(void) {
    uint8_t dato;
    while (UART_Receive(UART_PC, &dato, 1, NONE_BLOCKING) == 1) {
        UART_Send(UART_PC, &dato, 1, BLOCKING);   /* eco directo */
    }
}
 
int main(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);
 
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
 
    UART_SendString("\r\n=== TEST 1: UART ===\r\n");
    UART_SendString("Lo que tipees vuelve como eco.\r\n");
    UART_SendString("Cada 1s deberia aparecer TICK n.\r\n\r\n");
 
    while (1) {
        if (enviar_tick) {
            enviar_tick = 0;
            UART_SendString("TICK ");
            UART_SendUInt(++contador_ticks);
            UART_SendString("\r\n");
        }
    }
}
