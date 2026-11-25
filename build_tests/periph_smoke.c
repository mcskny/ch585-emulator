#include "CH58x_common.h"

volatile uint32_t g_gpioa_irqs = 0;
volatile uint32_t g_gpiob_irqs = 0;
volatile uint32_t g_tmr0_irqs = 0;
volatile uint32_t g_adc_scan_done = 0;
volatile uint32_t g_rtc_irqs = 0;

__attribute__((aligned(4))) uint16_t scan_buf[4];

__INTERRUPT __HIGH_CODE void GPIOA_IRQHandler(void) {
    uint16_t flag = GPIOA_ReadITFlagPort();
    GPIOA_ClearITFlagBit(flag);
    g_gpioa_irqs++;
    GPIOB_InverseBits(GPIO_Pin_0);
}

__INTERRUPT __HIGH_CODE void GPIOB_IRQHandler(void) {
    uint32_t flag = GPIOB_ReadITFlagPort();
    GPIOB_ClearITFlagBit(flag);
    g_gpiob_irqs++;
}

__INTERRUPT __HIGH_CODE void TMR0_IRQHandler(void) {
    if (TMR0_GetITFlag(TMR0_3_IT_CYC_END)) {
        TMR0_ClearITFlag(TMR0_3_IT_CYC_END);
        g_tmr0_irqs++;
    }
}

__INTERRUPT __HIGH_CODE void ADC_IRQHandler(void) {
    if (R8_ADC_INT_FLAG & RB_ADC_IF_EOC) {
        uint8_t c = R8_ADC_CONVERT;
        R8_ADC_CONVERT = c;
    }
    if (R32_ADC_SCAN_CFG2 & RB_ADC_IF_SCAN_END) {
        R32_ADC_SCAN_CFG2 |= RB_ADC_IF_SCAN_END;
        g_adc_scan_done = 1;
    }
}

__INTERRUPT __HIGH_CODE void RTC_IRQHandler(void) {
    R8_RTC_FLAG_CTRL = RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR;
    g_rtc_irqs++;
}

int main(void) {
    SetSysClock(SYSCLK_FREQ);

    // UART0 init
    GPIOB_SetBits(GPIO_Pin_7);
    GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();

    // GPIOA/B setup: PB0 output=0, PA0 PU falling-edge IRQ, PB22 remapped INTX falling-edge IRQ
    GPIOB_ResetBits(GPIO_Pin_0);
    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(GPIO_Pin_0, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(GPIO_Pin_0, GPIO_ITMode_FallEdge);
    PFIC_EnableIRQ(GPIO_A_IRQn);

    GPIOPinRemap(ENABLE, RB_PIN_INTX);
    GPIOB_ModeCfg(GPIO_Pin_22, GPIO_ModeIN_PU);
    GPIOB_ITModeCfg(GPIO_Pin_22, GPIO_ITMode_FallEdge);
    PFIC_EnableIRQ(GPIO_B_IRQn);

    // PWM4 on PA12 (25% duty)
    GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);
    PWMX_CLKCfg(1);
    PWMX_CycleCfg(PWMX_Cycle_256);
    PWMX_ACTOUT(CH_PWM4, 64, High_Level, ENABLE);

    // TMR0 periodic every 10000 cycles (~160us)
    TMR0_TimerInit(10000);
    TMR0_ClearITFlag(TMR0_3_IT_CYC_END);
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
    PFIC_EnableIRQ(TMR0_IRQn);

    // SPI0 send 3 bytes
    SPI0_MasterDefInit();
    uint8_t spi_tx[3] = {0xAA, 0x55, 0x93};
    SPI0_MasterTrans(spi_tx, 3);

    // ADC 4-channel scan (A0, A1, A2, A3) with PGA -6dB + DMA
    ADC_ExtSingleChSampInit(SampleFreq_4_or_2, ADC_PGA_1_2);
    ADC_ChannelCfg(0);
    R32_ADC_SCAN_CFG1 = (1 << 0) | (2 << 4) | (3 << 8);
    R32_ADC_SCAN_CFG2 = ((4 - 1) << 24) | RB_ADC_SCAN_SEL | RB_ADC_IE_SCAN_END | RB_ADC_IF_SCAN_END;
    R32_ADC_DMA_BEG = (uint32_t)&scan_buf[0] & 0x1FFFF;
    R32_ADC_DMA_END = (uint32_t)&scan_buf[4] & 0x1FFFF;
    R8_ADC_CTRL_DMA |= RB_ADC_DMA_ENABLE | RB_SCAN_AUTO_TYPE;
    PFIC_EnableIRQ(ADC_IRQn);
    R8_ADC_CONVERT |= RB_ADC_START;

    while (!g_adc_scan_done);

    // RTC timer 0.125s
    RTC_TMRFunCfg(Period_0_125_S);
    PFIC_EnableIRQ(RTC_IRQn);

    printf("SMOKE OK scan=%u,%u,%u,%u\n", scan_buf[0], scan_buf[1], scan_buf[2], scan_buf[3]);

    while (1) {
        __WFI();
    }
}
