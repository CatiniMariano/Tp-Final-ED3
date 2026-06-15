/*
 * Test04_ADC_DMA.c — verificar la cadena Timer0 -> ADC -> DMA -> UART.
 *
 * Como usar:
 *   - Conectar un potenciometro como divisor: extremos a 3.3V y GND, wiper a P0.23
 *   - Si no tenes pot: conecta P0.23 directo a 3.3V o a GND (lectura fija)
 *   - Cada 500 ms se imprime "ADC:NNNN  mV:NNNN"
 *   - Girar el pot deberia hacer cambiar los valores
 *
 * Que esperar:
 *   - P0.23 a GND:     ADC ~ 0     mV ~ 0
 *   - P0.23 a 3.3V:    ADC ~ 4095  mV ~ 3300
 *   - Pot al medio:    ADC ~ 2048  mV ~ 1650
 *
 * Diagnostico:
 *   - Si "ADC:0" SIEMPRE aunque muevas el pot:
 *       a) El DMA no transfiere -> falta ADC_IntEnable(ADC_INT_CH0)
 *       b) El Timer0 no dispara el ADC
 *       c) ADC_PinConfig no configuro bien P0.23
 *   - Si los valores oscilan locos sin tocar nada:
 *       - El pin esta flotando (sin conectar)
 *       - Falta el cap de filtro
 *   - Si los valores cambian pero estan muy lejos del rango esperado:
 *       - Ruido de masa, o pin equivocado
 *
 * IMPORTANTE: este test NO usa ADC_INT_CH0 -> el DMA no va a recibir
 * request. Esta linea es la clave del test: si la comentas el DMA no
 * transfiere. La dejamos habilitada para que el test funcione bien.
 */

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"
#include <string.h>

#define UART_PC      ((LPC_UART_TypeDef *) LPC_UART0)
#define UART_BAUD    9600
#define ADC_MAX      4095UL
#define DAC_MV_REF   3300UL

uint32_t buffer_adc[10];

volatile uint32_t promedio = 0;
volatile uint32_t adc_mv = 0;
volatile uint32_t tick_ms = 0;
volatile uint8_t  imprimir = 0;

/* ----------- UART ----------- */
void UART_SendString(const char *s) {
    UART_Send(UART_PC, (uint8_t *)s, strlen(s), BLOCKING);
}
void UART_SendUInt(uint32_t num) {
    char buf[11]; int8_t i = 0, j;
    if (num == 0) { UART_Send(UART_PC, (uint8_t *)"0", 1, BLOCKING); return; }
    while (num > 0 && i < 10) { buf[i++] = (char)((num % 10) + '0'); num /= 10; }
    for (j = i - 1; j >= 0; j--) UART_Send(UART_PC, (uint8_t *)&buf[j], 1, BLOCKING);
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
}

/* ----------- Timer0 dispara ADC cada 5ms ----------- */
void timer0_cfg(void) {
    TIM_TIMERCFG_T t = { .prescaleOpt = TIM_US, .prescaleValue = 500 };
    TIM_InitTimer(LPC_TIM0, &t);

    TIM_MATCHCFG_T m;
    m.channel = 1; m.intEn = DISABLE; m.stopEn = DISABLE; m.resetEn = ENABLE;
    m.extOpt = TIM_TOGGLE; m.matchValue = 5;
    TIM_ConfigMatch(LPC_TIM0, &m);
    TIM_Enable(LPC_TIM0);
}

/* ----------- ADC ----------- */
void adc_cfg(void) {
    ADC_Init(200000);
    ADC_PinConfig(0);
    ADC_StartCmd(ADC_START_ON_MAT01);
    ADC_EdgeStartConfig(ADC_START_ON_FALLING);
    ADC_ChannelEnable(0);
    ADC_PowerUp();
    ADC_IntEnable(ADC_INT_CH0);  /* IMPRESCINDIBLE para que el DMA reciba request */
}

/* ----------- DMA ----------- */
void dma_cfg(void) {
    GPDMA_Init();

    static GPDMA_LLI_T lli1;
    lli1.srcAddr = (uint32_t)&(LPC_ADC->ADDR0);
    lli1.dstAddr = (uint32_t)buffer_adc;
    lli1.nextLLI = (uint32_t)&lli1;
    lli1.control = 10 | (2<<18) | (2<<21) | (1<<27) | (1<<31);

    NVIC_EnableIRQ(DMA_IRQn);d

    GPDMA_Channel_CFG_T d;
    d.channelNum    = 0;
    d.transferSize  = 10;
    d.type          = GPDMA_P2M;
    d.srcMemAddr    = 0;
    d.dstMemAddr    = (uint32_t)buffer_adc;
    d.srcConn       = GPDMA_ADC;
    d.dstConn       = 0;
    d.src.width = GPDMA_WORD; d.src.burst = GPDMA_BSIZE_1; d.src.increment = DISABLE;
    d.dst.width = GPDMA_WORD; d.dst.burst = GPDMA_BSIZE_1; d.dst.increment = ENABLE;
    d.intTC = ENABLE; d.intErr = DISABLE;
    d.linkedList = (uint32_t)&lli1;

    GPDMA_SetupChannel(&d);
    GPDMA_ChannelStart(0);
}

void DMA_IRQHandler(void) {
    if (GPDMA_IntGetStatus(GPDMA_INTTC, 0)) {
        GPDMA_ClearIntPending(GPDMA_CLR_INTTC, 0);
        GPDMA_ChannelPause(0);

        uint32_t suma = 0;
        for (int i = 0; i < 10; i++) {
            suma += (buffer_adc[i] >> 4) & 0xFFF;
        }
        promedio = suma / 10;
        adc_mv   = (promedio * DAC_MV_REF) / ADC_MAX;

        GPDMA_ChannelResume(0);
    }
}

void SysTick_Handler(void) {
    if (++tick_ms >= 500) {
        tick_ms = 0;
        imprimir = 1;
    }
}

int main(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);

    uart_cfg();
    adc_cfg();
    dma_cfg();
    timer0_cfg();

    UART_SendString("\r\n=== TEST 4: ADC + DMA + Timer0 ===\r\n");
    UART_SendString("Conecta un pot a P0.23 (entre 3.3V y GND).\r\n");
    UART_SendString("Imprime cada 500ms el ADC y los mV.\r\n\r\n");

    while (1) {
        if (imprimir) {
            imprimir = 0;
            UART_SendString("ADC:");
            UART_SendUInt(promedio);
            UART_SendString("  mV:");
            UART_SendUInt(adc_mv);
            UART_SendString("\r\n");
        }
    }
}
