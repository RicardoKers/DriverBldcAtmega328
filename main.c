/**
 * BLDC Motor Control on ATmega328P @ 16 MHz
 *
 * - Reads ADC0 (PC0) in Free-Running mode, converting 0 – 5 V to 0 – 100 % PWM
 *   (0 – 199 in OCR0B) and generating a 10 kHz PWM on OC0B (PD5).
 * - Hall sensors on PC1, PC2 and PC3 define the rotor position (3 bits).
 * - High-side switches on PB0, PB1 and PB2 — digital on/off control.
 * - Low-side switches on PB3, PB4 and PB5 — gating via external AND with PWM on PD5.
 * - Every PWM duty update occurs in the ADC ISR; the commutation routine runs in
 *   the main loop, reading the Hall sensors and enabling the switches according
 *   to rotor position as long as adc_value > threshold (~10).
 *
 * Each PB3/4/5 line must be externally ANDed with PD5 (PWM OC0B) so that a MOSFET
 * is driven only when both inputs of the gate are high.
 *
 * To compile in Microchip Studio (Atmel Studio):
 * - Create an "AVR C Executable" project for the ATmega328P.
 * - Define F_CPU as 16000000UL (in the code or in the project symbols).
 * - Program fuses for the 16 MHz external crystal if applicable.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/*
 * ----------------------------------------------------------------------------
 * Global variable that stores the ADC value (10 bits, 0–1023).
 * It is updated in ISR(ADC_vect) and read inside the commutation
 * function using cli()/sei() to guarantee an atomic 16‑bit access.
 * ----------------------------------------------------------------------------
 */
uint16_t adc_value;

/*
 * ----------------------------------------------------------------------------
 * Pin definitions (using AVR register naming):
 * ----------------------------------------------------------------------------
 *   - Hall sensors: PC1, PC2, PC3 (digital inputs — ADC0 uses PC0 so these pins
 *     are free)
 *   - High-side MOSFETs: PB0 (Aʰ), PB1 (Bʰ), PB2 (Cʰ) → digital on/off control
 *   - Low-side MOSFETs: PB3 (Aˡ), PB4 (Bˡ), PB5 (Cˡ) → gated externally with the PWM on PD5
 */
#define HALL1   PC1
#define HALL2   PC2
#define HALL3   PC3

#define AH_PIN  PB0
#define BH_PIN  PB1
#define CH_PIN  PB2

#define AL_PIN  PB3
#define BL_PIN  PB4
#define CL_PIN  PB5

/*
 * ----------------------------------------------------------------------------
 * Timer0 initialization to generate a 10 kHz PWM on channel OC0B (PD5)
 * ----------------------------------------------------------------------------
 *
 * Settings:
 *  - Fast PWM mode with TOP = OCR0A → WGM02:0 = 1 1 1 (mode 7)
 *  - Non‑inverting output on OC0B → COM0B1:0 = 1 0
 *  - Prescaler = 8 → CS02:0 = 0 1 0
 *  - OCR0A = 199 → sets TOP = 199 → f_PWM = F_CPU / (N·(TOP+1))
 *                       = 16 MHz / (8·200) = 10 000 Hz
 *  - OCR0B adjusts duty cycle between 0 and 199 (0 – 100 %)
 *  - PD5 must be configured as output
 */
static void timer0_pwm_10kHz_init(void)
{
    // Configure PD5 (OC0B) as output
    DDRD |= (1 << PD5);

    // TCCR0A: WGM01=1, WGM00=1 → Fast PWM; COM0B1=1, COM0B0=0 → non-inverting on OC0B
    TCCR0A = (1 << WGM01) | (1 << WGM00)
           | (1 << COM0B1);

    // TCCR0B: WGM02=1 → completes WGM02:0 = 7; CS01=1 → prescaler = 8
    TCCR0B = (1 << WGM02)
           | (1 << CS01);

    // Set TOP in OCR0A for exactly 10 kHz
    OCR0A = 199;
    // Start duty cycle at 0 (output stays LOW until first ADC ISR)
    OCR0B = 0;
}

/*
 * ----------------------------------------------------------------------------
 * ADC initialization in Free-Running mode (channel ADC0, PC0) with interrupt
 * ----------------------------------------------------------------------------
 *
 * Settings:
 *  - REFS0=1, REFS1=0 → reference AVcc (5 V)
 *  - MUX3:0 = 0000 → channel ADC0 (pin PC0)
 *  - ADLAR = 0 → result right adjusted (10 bits in ADC[9:0])
 *  - ADPS2:0 = 1 1 1 → prescaler = 128 → F_ADC ≈ 16 MHz / 128 ≈ 125 kHz
 *  - ADATE = 1 → auto trigger (Free-Running Mode)
 *  - ADIE = 1 → enables interrupt at end of conversion (ADC_vect)
 *  - ADTS2:0 = 0 0 0 → trigger source = Free-Running
 *  - ADSC = 1 → starts the first conversion; each end triggers the next
 */
static void adc_free_running_init(void)
{
    // Select AVcc reference and ADC0 (PC0); ADLAR=0 → right adjusted
    ADMUX = (1 << REFS0)
          | (0 << REFS1)
          | (0 << ADLAR)
          | (0 << MUX3)
          | (0 << MUX2)
          | (0 << MUX1)
          | (0 << MUX0);

    // Enable ADC, Auto Trigger, Interrupt on Conversion, prescaler = 128
    ADCSRA = (1 << ADEN)
           | (1 << ADATE)
           | (1 << ADIE)
           | (1 << ADPS2)
           | (1 << ADPS1)
           | (1 << ADPS0);

    // Trigger mode = Free-Running (ADTS2:0 = 0 0 0)
    ADCSRB = (0 << ADTS2)
           | (0 << ADTS1)
           | (0 << ADTS0);

    // Start the first conversion; after that the ADC runs continuously
    ADCSRA |= (1 << ADSC);
}

/*
 * ----------------------------------------------------------------------------
 * Reads the state of the Hall sensors (3 bits) and returns a value from 0 to 7:
 *   bit2 = HALL1 (PC1), bit1 = HALL2 (PC2), bit0 = HALL3 (PC3)
 * ----------------------------------------------------------------------------
 * Returns:
 *   0b000 to 0b111 according to the digital signals on pins PC1, PC2, PC3
 */
static inline uint8_t hall_state(void)
{
    uint8_t state = 0;
    if (PINC & (1 << HALL1)) state |= 0b100;
    if (PINC & (1 << HALL2)) state |= 0b010;
    if (PINC & (1 << HALL3)) state |= 0b001;
    return state;
}

/*
 * ----------------------------------------------------------------------------
 * Commutation function based on Hall sensor readings (6-step rotor)
 * ----------------------------------------------------------------------------
 *
 * 1) Atomically copies 'adc_value' (cli()/sei()) to 'tmp_adc_value'.
 * 2) If tmp_adc_value > 10 (~0.05 V) enables the High-Side and Low-Side switches
 *    corresponding to the phase according to the Gray code from the Hall sensors.
 * 3) If tmp_adc_value ≤ 10 all outputs stay low (motor off).
 * 4) In hardware each Low-Side line (PB3, PB4, PB5) must be ANDed with the PWM
 *    signal on PD5 to modulate the motor voltage.
 *
 * Default commutation sequence (assuming 120° electrical between sensors):
 *   Hall = 0b001 (1): Cʰ, Bˡ
 *   Hall = 0b101 (5): Aʰ, Bˡ
 *   Hall = 0b100 (4): Aʰ, Cˡ
 *   Hall = 0b110 (6): Bʰ, Cˡ
 *   Hall = 0b010 (2): Bʰ, Aˡ
 *   Hall = 0b011 (3): Cʰ, Aˡ

 * Change the order if the rotation direction is reversed.
 */
static void commutate(uint8_t hall)
{
    uint16_t tmp_adc_value;

    //-Atomic copy of 16 bits from 'adc_value'----------------------------------
    cli();                  // disable interrupts
    tmp_adc_value = adc_value;
    sei();                  // enable interrupts

    //-Read current PORTB to avoid overwriting unused pins-----------------------
    uint8_t tmp_PORTB = PORTB;

    // Clear all High-Side and Low-Side bits (0 → off)
    tmp_PORTB &= ~((1 << AH_PIN) | (1 << BH_PIN) | (1 << CH_PIN)
                 | (1 << AL_PIN) | (1 << BL_PIN) | (1 << CL_PIN));

    if (tmp_adc_value > 10)
    {
        // If above the threshold, commutate according to 'hall'
        switch (hall)
        {
            case 0b001:  // Hall = 1 → Cˡ and Bʰ
                tmp_PORTB |= (1 << CH_PIN);  // High-Side C = 1
                tmp_PORTB |= (1 << BL_PIN);  // Low-Side B = 1 (gated with PWM)
                break;

            case 0b101:  // Hall = 5 → Bˡ and Aʰ
                tmp_PORTB |= (1 << AH_PIN);  // High-Side A = 1
                tmp_PORTB |= (1 << BL_PIN);  // Low-Side B = 1
                break;

            case 0b100:  // Hall = 4 → Cˡ and Aʰ
                tmp_PORTB |= (1 << AH_PIN);  // High-Side A = 1
                tmp_PORTB |= (1 << CL_PIN);  // Low-Side C = 1
                break;

            case 0b110:  // Hall = 6 → Cˡ and Bʰ
                tmp_PORTB |= (1 << BH_PIN);  // High-Side B = 1
                tmp_PORTB |= (1 << CL_PIN);  // Low-Side C = 1
                break;

            case 0b010:  // Hall = 2 → Aˡ and Bʰ
                tmp_PORTB |= (1 << BH_PIN);  // High-Side B = 1
                tmp_PORTB |= (1 << AL_PIN);  // Low-Side A = 1
                break;

            case 0b011:  // Hall = 3 → Aˡ and Cʰ
                tmp_PORTB |= (1 << CH_PIN);  // High-Side C = 1
                tmp_PORTB |= (1 << AL_PIN);  // Low-Side A = 1
                break;

            default:
                // If reading invalid (0, 7 or others) do not commutate
                break;
        }
    }
    // If tmp_adc_value <= 10 keep all switches off (tmp_PORTB already cleared)

    // Update PORTB with the new configuration
    PORTB = tmp_PORTB;
}

/*
 * ----------------------------------------------------------------------------
 * ADC ISR: updates the PWM duty cycle (OCR0B) proportional to the ADC value
 * ----------------------------------------------------------------------------
 *
 * Runs at every end of conversion (Free-Running mode):
 * 1) Reads the 10‑bit ADC value (0–1023).
 * 2) Calculates duty = floor(adc_value * 199 / 1023) → maps 0 – 1023 to 0 – 199.
 * 3) Updates OCR0B (8 bits) to set the Timer0 duty cycle.
 */
ISR(ADC_vect)
{
    uint16_t duty;

    // Read 10-bit result from ADC (macro ADC reads ADCL and ADCH)
    adc_value = ADC;

    // Duty calculation (0–199) for 0–100 % at 10 kHz
    duty = (uint16_t)((adc_value * 199UL) / 1023UL);

    // Update OCR0B (8 bits). Implicit cast discards upper bits.
    OCR0B = (uint8_t)duty;
}

int main(void)
{
    //----------------------------------------------------------------------------- 
    // Pin configuration:
    //  - Set PB0–PB5 as outputs (High-Side and Low-Side MOSFETs)
    //  - PD5 will be configured in timer0_pwm_10kHz_init()
    //  - PC1–PC3 are inputs by default (Hall sensors)
    //-----------------------------------------------------------------------------
    DDRB |= (1 << AH_PIN) | (1 << BH_PIN) | (1 << CH_PIN)
          | (1 << AL_PIN) | (1 << BL_PIN) | (1 << CL_PIN);

    //----------------------------------------------------------------------------- 
    // Disable global interrupts while configuring peripherals
    //-----------------------------------------------------------------------------
    cli();

    //-----------------------------------------------------------------------------
    // Initialize Timer0 to generate 10 kHz PWM on PD5 (OC0B)
    //-----------------------------------------------------------------------------
    timer0_pwm_10kHz_init();

    //-----------------------------------------------------------------------------
    // Initialize ADC in Free-Running mode on channel ADC0 (PC0) with interrupt
    //-----------------------------------------------------------------------------
    adc_free_running_init();

    //-----------------------------------------------------------------------------
    // Enable global interrupts to allow the ADC ISR
    //-----------------------------------------------------------------------------
    sei();

    //----------------------------------------------------------------------------- 
    // Empty main loop:
    //   - Just reads the Hall sensors and calls commutate() continuously.
    //   - All duty cycle modulation is handled in the ADC ISR.
    //-----------------------------------------------------------------------------
    while (1)
    {
        uint8_t hall = hall_state();
        commutate(hall);
        // A small _NOP_ or other noncritical logic could be placed here
    }

    // Never reached
    return 0;
}
