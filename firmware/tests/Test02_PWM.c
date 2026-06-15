/*
 * Test02_PWM.c — verificar que el PWM sale por P1.22.
 *
 * Como usar:
 *   - Mandar un numero 0-100 + ENTER por UART
 *   - El duty cambia y se confirma con "PWM:NN%"
 *   - Arranca al 25%
 *
 * Verificacion fisica (P1.22 contra GND):
 *   - Multimetro en DC: Vprom = (duty/100) * 3.3V
 *       duty=25  -> ~0.83 V
 *       duty=50  -> ~1.65 V
 *       duty=75  -> ~2.47 V
 *       duty=100 -> ~3.3 V
 *       duty=0   -> 0 V
 *   - Osciloscopio: onda cuadrada de 10 kHz, ancho variable
 *   - Sin scope ni multi: conecta un LED + R 220ohm entre P1.22 y GND
 *     y vas a ver que cambia el brillo con cada comando
 *
 * Diagnostico:
 *   - duty=50 da 0 V o 3.3 V todo el tiempo: TIMER1 no arranca o IRQ no entra
 *   - duty=50 da una tension errada (no ~1.65V): la frecuencia del PWM o el
 *     periodo estan mal, o el pin no esta configurado como GPIO output
 */

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_uart.h"
#include <string.h>

#define UART_PC          ((LPC_UART_TypeDef *) LPC_UART0)
#define UART_BAUD        9600
#define PWM_PERIODO      100
#define PWM_DUTY_INIT    25
#define RX_BUFFER_SIZE   16

volatile uint32_t current_duty = PWM_DUTY_INIT;
volatile char     rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t  rx_index = 0;
volatile uint8_t  comando_listo = 0;

/* ----------- UART helpers ----------- */
void UART_SendString(const char *s) {
    UART_Send(UART_PC, (uint8_t *)s, strlen(s), BLOCKING);
}
void UART_SendUInt(uint32_t num) {
    char buf[11]; int8_t i = 0, j;
    if (num == 0) { UART_Send(UART_PC, (uint8_t *)"0", 1, BLOCKING); return; }
    while (num > 0 && i < 10) { buf[i++] = (char)((num % 10) + '0'); num /= 10; }
    for (j = i - 1; j >= 0; j--) UART_Send(UART_PC, (uint8_t *)&buf[j], 1, BLOCKING);
}

/* ----------- PWM ----------- */
void pwm_cfg(void) {
    PINSEL_CFG_T pin = {
        .port = 1, .pin = 22, .func = PINSEL_FUNC_00,
        .mode = PINSEL_TRISTATE, .openDrain = DISABLE
    };
    PINSEL_ConfigPin(&pin);
    GPIO_SetDir(1, (1 << 22), GPIO_OUTPUT);
    GPIO_SetPinState(1, 22, RESET);

    TIM_TIMERCFG_T t = { .prescaleOpt = TIM_US, .prescaleValue = 1 };
    TIM_InitTimer(LPC_TIM1, &t);

    TIM_MATCHCFG_T m;
    m.channel = 0; m.intEn = ENABLE; m.stopEn = DISABLE; m.resetEn = DISABLE;
    m.extOpt = TIM_NOTHING; m.matchValue = PWM_DUTY_INIT;
    TIM_ConfigMatch(LPC_TIM1, &m);

    m.channel = 1; m.intEn = ENABLE; m.stopEn = DISABLE; m.resetEn = ENABLE;
    m.extOpt = TIM_NOTHING; m.matchValue = PWM_PERIODO;
    TIM_ConfigMatch(LPC_TIM1, &m);

    NVIC_EnableIRQ(TIMER1_IRQn);
    TIM_Enable(LPC_TIM1);
}

void TIMER1_IRQHandler(void) {
    if (TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT)) {
        GPIO_SetPinState(1, 22, RESET);
        TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
    }
    if (TIM_GetIntStatus(LPC_TIM1, TIM_MR1_INT)) {
        GPIO_SetPinState(1, 22, SET);
        TIM_ClearIntPending(LPC_TIM1, TIM_MR1_INT);
    }
}

void set_pwm_duty(uint32_t duty) {
    if (duty > 100) duty = 100;
    current_duty = duty;
    if (duty == 0)        { TIM_Disable(LPC_TIM1); GPIO_SetPinState(1, 22, RESET); }
    else if (duty == 100) { TIM_Disable(LPC_TIM1); GPIO_SetPinState(1, 22, SET);   }
    else {
        TIM_UpdateMatchValue(LPC_TIM1, TIM_MATCH_0, duty);
        LPC_TIM1->TC = 0;
        TIM_Enable(LPC_TIM1);
    }
}

/* ----------- UART config y handler ----------- */
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

uint8_t parsear_porcentaje(volatile char *str) {
    uint16_t v = 0; uint8_t i = 0;
    if (str[0] == '\0') return 255;
    while (str[i] != '\0' && i < RX_BUFFER_SIZE) {
        if (str[i] < '0' || str[i] > '9') return 255;
        v = v * 10 + (uint16_t)(str[i] - '0');
        if (v > 100) return 255;
        i++;
    }
    return (uint8_t)v;
}

int main(void) {
    uart_cfg();
    pwm_cfg();
    set_pwm_duty(PWM_DUTY_INIT);

    UART_SendString("\r\n=== TEST 2: PWM en P1.22 ===\r\n");
    UART_SendString("Arranco al 25%. Mande 0-100 + ENTER.\r\n\r\n");

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

            uint8_t v = parsear_porcentaje(local);
            if (v <= 100) {
                set_pwm_duty(v);
                UART_SendString("PWM:");
                UART_SendUInt(v);
                UART_SendString("%\r\n");
            } else {
                UART_SendString("ERROR: 0-100\r\n");
            }
        }
    }
}
