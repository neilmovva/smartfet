#pragma once

#include <avr/delay.h>
#include "hal_pwm.h"

#include "sinetable.h"
#define SINETABLE sin_a100_p200

void phase_test_loop() {
	pwm_update_ch(5, 3);
	//phase1
	pwm_update_ch(1, 1);
	pwm_update_ch(1, 2);
	//endphase
	_delay_ms(6000);
	pwm_update_ch(10, 3);
	_delay_ms(6000);
	pwm_update_ch(15, 3);
	_delay_ms(6000);
}


void triangle() {

	for(uint16_t step = 0; step < HAL_MAX_PWR_LEVEL; step++) {
		uint8_t pwr_a = step;
		uint8_t pwr_b = (step + 127) % HAL_MAX_PWR_LEVEL;

		pwm_update_ch(pwr_a, 1);
		pwm_update_ch(pwr_b, 2);
		_delay_ms(20);
	}

	for(uint16_t step = HAL_MAX_PWR_LEVEL; step > 0 ; step--) {
		uint8_t pwr_a = step;
		uint8_t pwr_b = (step - 127) % HAL_MAX_PWR_LEVEL;

		pwm_update_ch(pwr_a, 1);
		pwm_update_ch(pwr_b, 2);
		_delay_ms(20);
	}

	// turn off both channels
	pwm_update_ch(0, 1);
	pwm_update_ch(0, 2);
}

void sinewave_allphase() {
	const uint16_t table_pts = sizeof(SINETABLE);
	const uint16_t phase_stride = table_pts / NUM_CHANNELS;

	for(uint16_t theta = 0; theta < table_pts; theta++) {
		for(uint8_t idx_ch = 0; idx_ch < NUM_CHANNELS; idx_ch++) {
			uint16_t table_idx = (theta + idx_ch * phase_stride) % table_pts;
			uint8_t pwr_ch =  pgm_read_byte(&SINETABLE[table_idx]);
			pwm_update_ch(pwr_ch, idx_ch + 1);
		}
		_delay_ms(1);
	}

	// turn off all channels
	pwm_update_ch(0, 1);
	pwm_update_ch(0, 2);
	pwm_update_ch(0, 3);
}