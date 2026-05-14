#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

#define CAR_THRESHOLD 400


uint8_t ped_request = 0;


void init_hardware(void);
uint16_t get_sensor_data(uint8_t channel);
void stop_all_motors(void);
void update_timer(uint8_t road, uint8_t value);
uint8_t run_countdown(uint8_t road, uint8_t start_val);
void run_road1_cycle(void);
void run_road2_cycle(void);
void pedestrian_walk(void);
void check_button(void);

void init_hardware() {
	
	MCUCSR = (1 << JTD);
	MCUCSR = (1 << JTD);
	
	DDRC = 0xFF;
	DDRB = 0xFF;
	
	// Set PD6 as input for Pedestrian Button
	DDRD &= ~(1 << PD6);
	PORTD |= (1 << PD6);
	
	
	DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
	DDRA = 0xFC;
	
	
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}


void check_button() {
	if (!(PIND & (1 << PD6))) {
		ped_request = 1;
	}
}

uint16_t get_sensor_data(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	_delay_us(50);
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	return ADC;
}

void update_timer(uint8_t road, uint8_t value) {
	if (road == 1) {
		
		PORTA = (PORTA & 0xC3) | ((value & 0x0F) << 2);
		} else {
		
		PORTA = (PORTA & 0x3F) | ((value & 0x03) << 6);
		PORTD = (PORTD & 0xF3) | ((value & 0x0C));
	}
}

uint8_t run_countdown(uint8_t road, uint8_t start_val) {
	for (int i = start_val; i >= 0; i--) {
		update_timer(road, i);
		
		for(int j = 0; j < 100; j++) {
			check_button();
			if (ped_request) return 1;
			_delay_ms(10);
		}
	}
	update_timer(road, 0);
	return 0;
}

void stop_all_motors() {
	PORTB &= ~((1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4));
	PORTD &= ~((1 << PD4) | (1 << PD5));
}

void pedestrian_walk() {
	stop_all_motors();
	PORTC = (1 << PC0) | (1 << PC3);
	PORTB |= (1 << PB5);
	
	
	for(int i = 0; i < 500; i++) {
		_delay_ms(10);
	}
	
	PORTB &= ~(1 << PB5);
	ped_request = 0;
}

void run_road1_cycle() {
	
	PORTC = (1 << PC2) | (1 << PC3);
	PORTD |= (1 << PD4); PORTB |= (1 << PB1);
	if (run_countdown(1, 8)) return;
	
	
	PORTC = (1 << PC1) | (1 << PC3);
	stop_all_motors();
	if (run_countdown(1, 3)) return;
	
	PORTC = (1 << PC0) | (1 << PC3);
}

void run_road2_cycle() {
	
	PORTC = (1 << PC5) | (1 << PC0);
	PORTD |= (1 << PD5); PORTB |= (1 << PB3);
	if (run_countdown(2, 8)) return;
	
	
	PORTC = (1 << PC4) | (1 << PC0);
	stop_all_motors();
	if (run_countdown(2, 3)) return;
	
	PORTC = (1 << PC0) | (1 << PC3);
}

int main(void) {
	init_hardware();
	
	while (1) {
		check_button();
		
		
		if (ped_request) {
			pedestrian_walk();
		}

		uint16_t d1 = get_sensor_data(0);
		_delay_ms(5);
		uint16_t d2 = get_sensor_data(1);

		
		if (d1 < CAR_THRESHOLD && d2 < CAR_THRESHOLD) {
			if (d1 <= d2) {
				run_road1_cycle();
				if (ped_request) continue;
				d2 = get_sensor_data(1);
				if (d2 < CAR_THRESHOLD) run_road2_cycle();
				} else {
				run_road2_cycle();
				if (ped_request) continue;
				d1 = get_sensor_data(0);
				if (d1 < CAR_THRESHOLD) run_road1_cycle();
			}
		}
		else if (d1 < CAR_THRESHOLD) {
			run_road1_cycle();
		}
		else if (d2 < CAR_THRESHOLD) {
			run_road2_cycle();
		}
		else {
			
			PORTC = (1 << PC0) | (1 << PC3);
			stop_all_motors();
			update_timer(1, 0);
			update_timer(2, 0);
			_delay_ms(100);
		}
	}
}