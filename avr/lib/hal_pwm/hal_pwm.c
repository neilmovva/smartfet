#include "hal_pwm.h"

static inline int MAX(int a, int b) {
    if (a > b)
        return a;
    return b;
}


void pwm_update_ch(uint8_t powerlevel, uint8_t channel) {
    uint16_t pwm_level = 0;
    
    if (channel < 3) {
        pwm_level = powerlevel * PWR_TO_PWM_MULT;
    } else {
        pwm_level = (powerlevel * 5) / 2;
    }

    if(pwm_level > PWM_PERIOD_CYCLES) {
        pwm_level = PWM_PERIOD_CYCLES;
    }
    pwm_update_raw8(pwm_level, channel);
}


void pwm_ch_disable(uint8_t channel) {
    switch(channel) {
        case 1:
        CLR(TCCR1A, COM1A1);
        OCR1A = 0;
        break;

        case 2:
        CLR(TCCR1A, COM1B1);
        OCR1B = 0;
        break;

        case 3:
        CLR(TCCR0A, COM0A1);
        OCR0A = 0;
        break;
    }
}

void pwm_ch_enable(uint8_t channel) {
    switch(channel) {
        case 1:
        SET(TCCR1A, COM1A1);
        OCR1A = 0;
        break;

        case 2:
        SET(TCCR1A, COM1B1);
        OCR1B = 0;
        break;

        case 3:
        SET(TCCR0A, COM0A1);
        OCR0A = 0;
        break;
    }
}


void pwm_update_raw8(uint8_t pwm_level, uint8_t channel) {
    #ifdef DRV_INVERTED
    pwm_level = PWM_PERIOD_CYCLES - pwm_level;
    #endif

    if(pwm_level == 0) {
        pwm_ch_disable(channel);
        return;
    } 
    pwm_ch_enable(channel);

    switch(channel) {
        case 1:
        OCR1A = pwm_level;
        break;

        case 2:
        OCR1B = pwm_level;
        break;

        case 3:
        OCR0A = pwm_level;
        break;
    }
}


void pwm_init() {
    #ifdef DRV_INVERTED
    SET(PORT_PWM_CH1, PIN_PWM_CH1);
    SET(PORT_PWM_CH2, PIN_PWM_CH2);
    SET(PORT_IO_CH3, PIN_IO_CH3);
    #else 
    CLR(PORT_PWM_CH1, PIN_PWM_CH1);
    CLR(PORT_PWM_CH2, PIN_PWM_CH2);
    CLR(PORT_PWM_CH3, PIN_PWM_CH3);
    #endif

    SET(DDR_PWM, PIN_PWM_CH1);
    SET(DDR_PWM, PIN_PWM_CH2);
    SET(DDR_PWM_CH3, PIN_PWM_CH3);

    // Timer1 setup (CH A and B)
    // WGM 14, FastPWM, TOP=ICR1, 1x prescaler
    TCCR1A = _BV(WGM11);     
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    ICR1   = PWM_PERIOD_CYCLES;
    TCNT1  = 0x0000;

    // Timer0 setup (CH C)
    // WGM 03, FastPWM, TOP=0xFF, 1x prescaler (8 MHz tick / 256 clock period == 32KHz effective)
    TCCR0A = _BV(WGM00) | _BV(WGM01);     
    TCCR0B = _BV(CS00);
    TCNT0  = 0x00;
}

void pwm_stop() {
    pwm_ch_disable(1);
    pwm_ch_disable(2);
    pwm_ch_disable(3);

    #ifdef DRV_INVERTED
    SET(PORT_PWM_CH1, PIN_PWM_CH1);
    SET(PORT_PWM_CH2, PIN_PWM_CH2);
    SET(PORT_IO_CH3, PIN_IO_CH3);
    #else 
    CLR(PORT_PWM_CH1, PIN_PWM_CH1);
    CLR(PORT_PWM_CH2, PIN_PWM_CH2);
    CLR(PORT_PWM_CH3, PIN_PWM_CH3);
    #endif
}