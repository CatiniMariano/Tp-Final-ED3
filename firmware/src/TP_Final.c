/*
 * TrabajoFinal.c
 * Control de velocidad de motor DC con realimentacion por taco-generador
 */

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_uart.h"
#include <string.h>

/* ================================================================
   CONSTANTES
   ================================================================ */
#define PWM_PERIODO     100
#define PWM_DUTY_INIT   25

#define ADC_MAX                  4095UL
#define DAC_MV_REF               3300UL
#define UMBRAL_MOTOR_PARADO_MV   50UL

#define UART_PC          ((LPC_UART_TypeDef *) LPC_UART0)
#define UART_BAUD        9600
#define RX_BUFFER_SIZE   16
#define UART_REPORTE_MS  1000        // reporte periodico cada N ms

/* ================================================================
   GLOBALES
   ================================================================ */
uint32_t buffer_adc[10];

volatile uint32_t promedio      = 0;
volatile uint32_t adc_mv        = 0;
volatile uint32_t dac_mv        = 0;
volatile uint32_t rpm_estimado  = 0;
volatile uint32_t current_duty  = PWM_DUTY_INIT;

volatile char     rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t  rx_index      = 0;
volatile uint8_t  comando_listo = 0;

volatile uint8_t  enviar_estado    = 0;
volatile uint32_t contador_reporte = 0;

/* ================================================================
   PROTOTIPOS
   ================================================================ */
void timer0_cfg(void);
void pwm_cfg(void);
void set_pwm_duty(uint32_t duty);
void adc_cfg(void);
void dma_cfg(void);
void dac_cfg(void);
void uart_cfg(void);

void UART_SendString(const char *str);
void UART_SendUInt(uint32_t num);
void UART_ProcesarComando(void);
void UART_EnviarEstado(void);
uint8_t convertirTextoAPorcentaje(volatile char *str);

/* ================================================================
   TIMER0 - dispara el ADC cada 5 ms (200 Hz)
   ================================================================ */
void timer0_cfg(void){
    TIM_TIMERCFG_T t;
    t.prescaleOpt   = TIM_US;
    t.prescaleValue = 500;
    TIM_InitTimer(LPC_TIM0, &t);

    TIM_MATCHCFG_T m;
    m.channel    = 1;
    m.intEn      = DISABLE;
    m.stopEn     = DISABLE;
    m.resetEn    = ENABLE;
    m.extOpt     = TIM_TOGGLE;
    m.matchValue = 5;
    TIM_ConfigMatch(LPC_TIM0, &m);
    TIM_Enable(LPC_TIM0);
}

/* ================================================================
   PWM (TIMER1) - manejo manual via GPIO, 10 kHz, 0..100%
   ================================================================ */
void pwm_cfg(void){
    PINSEL_CFG_T pin;
    pin.port      = 1;
    pin.pin       = 22;
    pin.func      = PINSEL_FUNC_00;
    pin.mode      = PINSEL_TRISTATE;
    pin.openDrain = DISABLE;
    PINSEL_ConfigPin(&pin);

    GPIO_SetDir(1, (1 << 22), GPIO_OUTPUT);
    GPIO_SetPinState(1, 22, RESET);

    TIM_TIMERCFG_T t;
    t.prescaleOpt   = TIM_US;
    t.prescaleValue = 1;
    TIM_InitTimer(LPC_TIM1, &t);

    TIM_MATCHCFG_T m;

    m.channel    = 0;
    m.intEn      = ENABLE;
    m.stopEn     = DISABLE;
    m.resetEn    = DISABLE;
    m.extOpt     = TIM_NOTHING;
    m.matchValue = PWM_DUTY_INIT;
    TIM_ConfigMatch(LPC_TIM1, &m);

    m.channel    = 1;
    m.intEn      = ENABLE;
    m.stopEn     = DISABLE;
    m.resetEn    = ENABLE;
    m.extOpt     = TIM_NOTHING;
    m.matchValue = PWM_PERIODO;
    TIM_ConfigMatch(LPC_TIM1, &m);

    NVIC_SetPriority(TIMER1_IRQn, 2);
    NVIC_EnableIRQ(TIMER1_IRQn);
    TIM_Enable(LPC_TIM1);
}

void TIMER1_IRQHandler(void){
    if(TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT)){
        GPIO_SetPinState(1, 22, RESET);
        TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
    }
    if(TIM_GetIntStatus(LPC_TIM1, TIM_MR1_INT)){
        GPIO_SetPinState(1, 22, SET);
        TIM_ClearIntPending(LPC_TIM1, TIM_MR1_INT);
    }
}

void set_pwm_duty(uint32_t duty){
    if(duty > 100) duty = 100;
    current_duty = duty;

    if(duty == 0){
        TIM_Disable(LPC_TIM1);
        GPIO_SetPinState(1, 22, RESET);
    }
    else if(duty == 100){
        TIM_Disable(LPC_TIM1);
        GPIO_SetPinState(1, 22, SET);
    }
    else {
        TIM_UpdateMatchValue(LPC_TIM1, TIM_MATCH_0, duty);
        LPC_TIM1->TC = 0;
        TIM_Enable(LPC_TIM1);
    }
}

/* ================================================================
   ADC
   ================================================================ */
void adc_cfg(void){
    ADC_Init(200000);
    ADC_PinConfig(0);
    ADC_StartCmd(ADC_START_ON_MAT01);
    ADC_EdgeStartConfig(ADC_START_ON_FALLING);
    ADC_ChannelEnable(0);
    ADC_PowerUp();
    ADC_IntEnable(ADC_INT_CH0);
}

/* ================================================================
   GPDMA
   ================================================================ */
void dma_cfg(void){
    GPDMA_Init();

    static GPDMA_LLI_T lli1;
    lli1.srcAddr = (uint32_t)&(LPC_ADC->ADDR0);
    lli1.dstAddr = (uint32_t)buffer_adc;
    lli1.nextLLI = (uint32_t)&lli1;
    lli1.control = 10 | (2<<18) | (2<<21) | (1<<27) | (1<<31);

    NVIC_SetPriority(DMA_IRQn, 1);
    NVIC_EnableIRQ(DMA_IRQn);

    GPDMA_Channel_CFG_T d;
    d.channelNum    = 0;
    d.transferSize  = 10;
    d.type          = GPDMA_P2M;
    d.srcMemAddr    = 0;
    d.dstMemAddr    = (uint32_t)buffer_adc;
    d.srcConn       = GPDMA_ADC;
    d.dstConn       = 0;
    d.src.width     = GPDMA_WORD;
    d.src.burst     = GPDMA_BSIZE_1;
    d.src.increment = DISABLE;
    d.dst.width     = GPDMA_WORD;
    d.dst.burst     = GPDMA_BSIZE_1;
    d.dst.increment = ENABLE;
    d.intTC         = ENABLE;
    d.intErr        = DISABLE;
    d.linkedList    = (uint32_t)&lli1;

    GPDMA_SetupChannel(&d);
    GPDMA_ChannelStart(0);
}

/* ================================================================
   DAC
   ================================================================ */
void dac_cfg(void){
    PINSEL_CFG_T p;
    p.port      = 0;
    p.pin       = 26;
    p.func      = PINSEL_FUNC_10;
    p.mode      = PINSEL_TRISTATE;
    p.openDrain = DISABLE;
    PINSEL_ConfigPin(&p);
    DAC_Init();
}

/* ================================================================
   UART0 (con el driver de catedra)
   ================================================================ */
void uart_cfg(void){
    UART_CFG_T uartCfg = {
        .baudRate = UART_BAUD,
        .dataBits = UART_DBITS_8,
        .stopBits = UART_STOPBIT_1,
        .parity   = UART_PARITY_NONE
    };
    UART_FIFO_CFG_T fifoCfg = {
        .level      = UART_FIFO_TRGLEV0,
        .resetRxBuf = ENABLE,
        .resetTxBuf = ENABLE,
        .dmaMode    = DISABLE
    };

    UART_PinConfig(UART_TX0_P0_2);
    UART_PinConfig(UART_RX0_P0_3);
    UART_Init(UART_PC, &uartCfg);
    UART_FIFOConfig(UART_PC, &fifoCfg);
    UART_TxEnable(UART_PC);
    UART_IntConfig(UART_PC, UART_INT_RBR, ENABLE);
    UART_IntConfig(UART_PC, UART_INT_RLS, ENABLE);
    NVIC_SetPriority(UART0_IRQn, 0);
    NVIC_EnableIRQ(UART0_IRQn);
}

void UART_SendString(const char *str){
    UART_Send(UART_PC, (uint8_t *)str, strlen(str), BLOCKING);
}

void UART_SendUInt(uint32_t num){
    char buf[11];
    int8_t i = 0, j;
    if (num == 0) {
        UART_Send(UART_PC, (uint8_t *)"0", 1, BLOCKING);
        return;
    }
    while (num > 0 && i < 10) {
        buf[i++] = (char)((num % 10) + '0');
        num /= 10;
    }
    for (j = i - 1; j >= 0; j--)
        UART_Send(UART_PC, (uint8_t *)&buf[j], 1, BLOCKING);
}

uint8_t convertirTextoAPorcentaje(volatile char *str){
    uint16_t valor = 0;
    uint8_t i = 0;
    if (str[0] == '\0') return 255;
    while (str[i] != '\0' && i < RX_BUFFER_SIZE) {
        if (str[i] < '0' || str[i] > '9') return 255;
        valor = valor * 10 + (uint16_t)(str[i] - '0');
        if (valor > 100) return 255;
        i++;
    }
    if (i >= RX_BUFFER_SIZE) return 255;
    return (uint8_t)valor;
}

/* ================================================================
   HANDLERS
   ================================================================ */
void DMA_IRQHandler(void){
    if(GPDMA_IntGetStatus(GPDMA_INTTC, 0)){
        GPDMA_ClearIntPending(GPDMA_CLR_INTTC, 0);
        GPDMA_ChannelPause(0);

        uint32_t suma = 0;
        for(int i = 0; i < 10; i++){
            suma += (buffer_adc[i] >> 4) & 0xFFF;
        }
        promedio = suma / 10;

        adc_mv = (promedio * DAC_MV_REF) / ADC_MAX;

        if (adc_mv < UMBRAL_MOTOR_PARADO_MV) {
            rpm_estimado = 0;
        } else {
            rpm_estimado = ((16875UL * adc_mv) / 10000UL) + 131UL;
        }

        /* DAC con escala 1 mV por RPM. Si tu motor pasa los 3300 RPM,
           cambia el divisor a (2 * DAC_MV_REF) para 0.5 mV/RPM. */
        uint32_t dac_val = (rpm_estimado * 1023UL) / DAC_MV_REF;
        if (dac_val > 1023) dac_val = 1023;
        DAC_UpdateValue(dac_val);

        dac_mv = (dac_val * DAC_MV_REF) / 1023UL;

        GPDMA_ChannelResume(0);
    }
}

void UART0_IRQHandler(void){
    uint32_t intId = UART_GetIntId(UART_PC);
    uint32_t tipo_int;
    uint8_t  estado;
    uint8_t  dato;
    uint8_t  leidos = 0;

    if (intId & UART_IIR_INTSTAT_PEND) return;

    tipo_int = intId & UART_IIR_INTID_MASK;
    estado   = UART_GetLineStatus(UART_PC);

    /* Receive Line Status: error de linea. Vaciamos el FIFO y descartamos. */
    if (tipo_int == UART_IIR_INTID_RLS) {
        if (!comando_listo) {
            rx_index = 0;
            rx_buffer[0] = '\0';
        }
        while ((leidos < RX_BUFFER_SIZE) &&
               (UART_Receive(UART_PC, &dato, 1, NONE_BLOCKING) == 1)) {
            leidos++;
        }
        return;
    }

    /* Receive Data Available o Character Timeout: hay bytes esperando. */
    if (tipo_int == UART_IIR_INTID_RDA || tipo_int == UART_IIR_INTID_CTI) {

        /* Si hay flags de error en LSR, reseteamos el buffer en construccion. */
        if ((estado & (UART_LINESTAT_OE | UART_LINESTAT_PE |
                       UART_LINESTAT_FE | UART_LINESTAT_BI |
                       UART_LINESTAT_RXFE)) && !comando_listo) {
            rx_index = 0;
            rx_buffer[0] = '\0';
        }

        /* Drenamos el FIFO completo en una sola pasada de la ISR. */
        while ((leidos < RX_BUFFER_SIZE) &&
               (UART_Receive(UART_PC, &dato, 1, NONE_BLOCKING) == 1)) {
            leidos++;

            if (comando_listo) continue;     // esperando a que main lo procese

            if (dato == '\r' || dato == '\n') {
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0';
                    rx_index = 0;
                    comando_listo = 1;
                }
            } else if (dato == 8 || dato == 127) {
                /* Backspace / Delete */
                if (rx_index > 0) {
                    rx_index--;
                    rx_buffer[rx_index] = '\0';
                }
            } else {
                if (dato < '0' || dato > '9') continue;   // solo digitos
                if (rx_index < (RX_BUFFER_SIZE - 1)) {
                    rx_buffer[rx_index++] = dato;
                    rx_buffer[rx_index]   = '\0';
                } else {
                    /* Overflow: reseteamos */
                    rx_index = 0;
                    rx_buffer[0] = '\0';
                }
            }
        }
    }
}

void SysTick_Handler(void) {
    contador_reporte++;
    if (contador_reporte >= UART_REPORTE_MS) {
        contador_reporte = 0;
        enviar_estado = 1;
    }
}

/* ================================================================
   UART - Procesamiento de comandos y reporte de estado
   ================================================================ */
void UART_ProcesarComando(void){
    char    comando_local[RX_BUFFER_SIZE];
    uint8_t nuevo_valor;
    uint8_t i;

    /* Copia segura del buffer compartido */
    NVIC_DisableIRQ(UART0_IRQn);
    for (i = 0; i < RX_BUFFER_SIZE; i++) {
        comando_local[i] = rx_buffer[i];
        if (rx_buffer[i] == '\0') break;
    }
    comando_local[RX_BUFFER_SIZE - 1] = '\0';
    rx_buffer[0]  = '\0';
    rx_index      = 0;
    comando_listo = 0;
    NVIC_EnableIRQ(UART0_IRQn);

    /* Reiniciamos el reporte para no inundar despues de un comando */
    enviar_estado     = 0;
    contador_reporte  = 0;

    nuevo_valor = convertirTextoAPorcentaje(comando_local);

    if (nuevo_valor <= 100) {
        set_pwm_duty((uint32_t)nuevo_valor);
        UART_SendString("\r\nOK: PWM = ");
        UART_SendUInt(nuevo_valor);
        UART_SendString(" %\r\n");
    } else {
        UART_SendString("\r\nERROR: ingresar un valor entre 0 y 100\r\n");
    }
}

void UART_EnviarEstado(void) {
    UART_SendString("\r\n======== ESTADO ========\r\n");

    UART_SendString("PWM duty       : ");
    UART_SendUInt(current_duty);
    UART_SendString(" %\r\n");

    UART_SendString("ADC entrada    : ");
    UART_SendUInt(adc_mv);
    UART_SendString(" mV\r\n");

    UART_SendString("RPM estimadas  : ");
    UART_SendUInt(rpm_estimado);
    UART_SendString("\r\n");

    UART_SendString("DAC salida     : ");
    UART_SendUInt(dac_mv);
    UART_SendString(" mV\r\n");
}

/* ================================================================
   MAIN
   ================================================================ */
int main(void){
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);    // tick cada 1 ms

    dac_cfg();
    uart_cfg();
    pwm_cfg();
    adc_cfg();
    dma_cfg();
    timer0_cfg();

    UART_SendString("\r\n=== Control de velocidad de motor DC ===\r\n");
    UART_SendString("Ingrese duty del PWM (0-100) y ENTER\r\n");

    while (1) {
        if (comando_listo) {
            UART_ProcesarComando();
            continue;
        }
        if (enviar_estado) {
            enviar_estado = 0;
            UART_EnviarEstado();
        }
    }
}
