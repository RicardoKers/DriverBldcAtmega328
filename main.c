#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL

// Hall sensor pins
#define HALL1 PC1
#define HALL2 PC2
#define HALL3 PC3

// High side MOSFET pins (digital on/off)
#define AH_PIN PD2
#define BH_PIN PD3
#define CH_PIN PD4

// Low side MOSFET pins driven with PWM
#define AL_PIN PB1 // OC1A
#define BL_PIN PB2 // OC1B
#define CL_PIN PD5 // OC0B

// ADC channel for throttle potentiometer
#define THROTTLE_CH 0

static inline void adc_init(void) {
    ADMUX = (1 << REFS0) | (1 << ADLAR) | THROTTLE_CH; // AVcc ref, left adjust
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

static inline uint8_t adc_read(void) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCH; // read 8 MSBs
}

static inline void pwm_init(void) {
    // Timer1 for phase A and B low sides
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11); // prescaler 8
    OCR1A = 0;
    OCR1B = 0;
    // Timer0 for phase C low side (uses OC0B on PD5)
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // fast PWM, non-inverting
    TCCR0B = (1 << CS01); // prescaler 8
    OCR0B = 0;
}

static inline uint8_t hall_state(void) {
    uint8_t state = 0;
    if (PINC & (1 << HALL1)) state |= 1;
    if (PINC & (1 << HALL2)) state |= 2;
    if (PINC & (1 << HALL3)) state |= 4;
    return state;
}

static void commutate(uint8_t hall, uint8_t duty) {
    // Turn off all high sides
    PORTD &= ~((1 << AH_PIN) | (1 << BH_PIN) | (1 << CH_PIN));

    // Default: all low side PWMs off
    OCR1A = 0;
    OCR1B = 0;
    OCR0B = 0;

    switch (hall) {
        case 1: // 001
            PORTD |= (1 << AH_PIN); // A high
            OCR1B = duty;          // B low PWM
            break;
        case 5: // 101
            PORTD |= (1 << AH_PIN); // A high
            OCR0B = duty;          // C low PWM
            break;
        case 4: // 100
            PORTD |= (1 << BH_PIN); // B high
            OCR0B = duty;          // C low PWM
            break;
        case 6: // 110
            PORTD |= (1 << BH_PIN); // B high
            OCR1A = duty;          // A low PWM
            break;
        case 2: // 010
            PORTD |= (1 << CH_PIN); // C high
            OCR1A = duty;          // A low PWM
            break;
        case 3: // 011
            PORTD |= (1 << CH_PIN); // C high
            OCR1B = duty;          // B low PWM
            break;
        default:
            break;
    }
}

int main(void) {
    // Configure MOSFET pins as outputs
    DDRD |= (1 << AH_PIN) | (1 << BH_PIN) | (1 << CH_PIN) | (1 << CL_PIN);
    DDRB |= (1 << AL_PIN) | (1 << BL_PIN);

    // Configure Hall sensor pins as inputs with pull-ups
    DDRC &= ~((1 << HALL1) | (1 << HALL2) | (1 << HALL3));
    PORTC |= (1 << HALL1) | (1 << HALL2) | (1 << HALL3);

    adc_init();
    pwm_init();

    while (1) {
        uint8_t throttle = adc_read();
        uint8_t hall = hall_state();
        commutate(hall, throttle);
        _delay_ms(1); // small delay to limit loop speed
    }
}

